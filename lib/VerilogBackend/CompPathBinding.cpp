//===--- CompPathBinding.cpp - Compatibility Path Based Binding  --*- C++ -*-===//
//
//                            The Verilog Backend
//
// Copyright: 2010 by Hongbin Zheng. all rights reserved.
// IMPORTANT: This software is supplied to you by Hongbin Zheng in consideration
// of your agreement to the following terms, and your use, installation, 
// modification or redistribution of this software constitutes acceptance
// of these terms.  If you do not agree with these terms, please do not use, 
// install, modify or redistribute this software. You may not redistribute, 
// install copy or modify this software without written permission from 
// Hongbin Zheng. 
//
//===----------------------------------------------------------------------===//
//
// This file implement the
// Compatibility Path Based Binding algorithm for interconnect reduction in HLS
//
//===----------------------------------------------------------------------===//

#include "HWAtomPasses.h"
#include "HWAtomInfo.h"

#include "vbe/ResourceConfig.h"

#include "llvm/ADT/PointerIntPair.h"
#include "llvm/Support/Allocator.h"
#include "llvm/Support/CommandLine.h"

#define DEBUG_TYPE "vbe-comp-path"
#include "llvm/Support/Debug.h"

using namespace llvm;
using namespace esyn;

static cl::opt<bool>
NoFUBinding("disable-resource-binding",
         cl::desc("vbe - Do not bind PostBind atom to Function Unit"),
         cl::Hidden, cl::init(false));

namespace esyn {
template<class DataTy>
class CompGraphNode {
public:
  typedef typename CompGraphNode<DataTy> _Self;
  typedef typename SmallPtrSetIterator<_Self*> pred_iterator;
  typedef typename SmallPtrSetIterator<_Self*> const_pred_iterator;

  typedef typename SmallPtrSetIterator<_Self*> succ_iterator;
  typedef typename SmallPtrSetIterator<_Self*> const_succ_iterator;

private:
  CompGraphNode(const _Self &);             // DO NOT IMPLEMENT
  void operator=(const _Self &);            // DO NOT IMPLEMENT

  SmallPtrSet<_Self*, 8> Preds, Succs;
  std::map<_Self*, unsigned> WeightMap;
  // { Data, isVREntry }
  PointerIntPair<DataTy*, 2> Data;
public:
  explicit CompGraphNode(DataTy* D = 0, bool isVREntry = false)
    : Data(D, isVREntry) {}

  bool isVRoot() const { return Data.getPointer() == 0; }
  bool isVREntry() const { return isVRoot() && Data.getInt(); }
  bool isVRExit() const { return isVRoot() && !Data.getInt(); }
  void clear() {
    Preds.clear();
    Succs.clear();
    WeightMap.clear();
  }

  DataTy *getData() const {
    assert(Data.getPointer() && "Can not get data for virtual root!");
    return Data.getPointer();
  }
  void updateData(DataTy *D) { Data.setPointer(D); }
  DataTy *operator ->() const { return Data.getPointer(); }
  DataTy *operator *() const { return Data.getPointer(); }

  void addPred(_Self* N) { Preds.insert(N); }
  void addSucc(_Self* N) { Succs.insert(N); }

  unsigned getWeightTo(_Self* N) const {
    assert(Succs.count(N) && "Only can get weight to Succs!");
    std::map<_Self*, unsigned>::const_iterator at = WeightMap.find(N);
    if (at != WeightMap.end())
      return at->second;

    return const_cast<_Self*>(this)->updateWeightTo(N);
  }

  unsigned updateWeightTo(_Self* N);

  // If the current node compatible with N?
  bool isCompatible(_Self *N) { 
    if (isVRoot() || N->isVRoot())
      return true;

    return computeCompatible(N);  
  } 

  bool computeCompatible(_Self *N);

  pred_iterator pred_begin() { return Preds.begin(); }
  const_pred_iterator pred_begin() const { return Preds.begin(); }

  pred_iterator pred_end() { return Preds.end(); }
  const_pred_iterator pred_end() const { return Preds.end(); }

  unsigned num_pred() const { return Preds.size(); }

  succ_iterator succ_begin() { return Succs.begin(); }
  const_succ_iterator succ_begin() const { return Succs.begin(); }

  succ_iterator succ_end() { return Succs.end(); }
  const_succ_iterator succ_end() const { return Succs.end(); }

  unsigned num_succ() const { return Succs.size(); }

  //
  void removeFromGraph() {
    for (succ_iterator I = succ_begin(), E = succ_end(); I != E; ++I) {
      _Self *Succ = *I;
      Succ->Preds.erase(this);
    }

    for (pred_iterator I = pred_begin(), E = pred_end(); I != E; ++I) {
      _Self *Pred = *I;
      Pred->Succs.erase(this);
    }
  }
  // 
  inline static bool isEalier(_Self *LHS, _Self *RHS) {
    if (LHS->isVREntry())
      return true;
    
    if (RHS->isVREntry())
      return false;

    if (RHS->isVRExit())
      return true;

    if (LHS->isVRExit())
      return false;

    // TODO: this is not true in Modulo Schedule.
    // Infact ealier is some kind of ealier in dependencies graph? So this is ok
    // in Modulo Schedule.
    return LHS->getData()->getSlot() < RHS->getData()->getSlot();
  }

  // Make an edge of Src and Dst;
  inline static void makeEdge(_Self *Src, _Self *Dst) {
    if (isEalier(Dst, Src)) std::swap(Src, Dst);
    
    assert(isEalier(Src, Dst) && "Src and Dst not compatible!");

    Src->addSucc(Dst);
    Dst->addPred(Src);
  }
};

template<class DataTy>
class CompGrapPath {
public:
  typedef CompGrapPath<DataTy> _Self;
  typedef CompGraphNode<DataTy> _NodeTy;
  typedef typename SmallPtrSetIterator<_NodeTy*> path_iterator;
  typedef typename SmallPtrSetIterator<_NodeTy*> const_path_iterator;
private:
  SmallPtrSet<_NodeTy*, 8> Nodes;
public:
  CompGrapPath(_NodeTy *Entry, _NodeTy *Exit) {
    findLongestPath(Entry, Exit, Nodes);
  }
 
  path_iterator path_begin() { return Nodes.begin(); }
  path_iterator path_end() { return Nodes.end(); }

  const_path_iterator path_begin() const { return Nodes.begin(); }
  const_path_iterator path_end() const { return Nodes.end(); }

  inline static void findLongestPath(_NodeTy *Entry, _NodeTy *Exit,
                                     SmallPtrSet<_NodeTy*, 8> &PathSet) {
    std::map<_NodeTy*, unsigned> LenMap;

    std::map<_NodeTy*, _NodeTy*> LongestPathPred;
    std::map<_NodeTy*, unsigned> LongestPathWeight;

    //for each vertex v in topOrder(G) do
    typedef _NodeTy::succ_iterator ChildIt;
    SmallVector<std::pair<_NodeTy*, ChildIt>, 32> WorkStack;
    std::map<_NodeTy*, unsigned> VisitCount;

    WorkStack.push_back(std::make_pair(Entry, Entry->succ_begin()));
    LongestPathWeight[Entry] = 0;

    while (!WorkStack.empty()) {
      _NodeTy *Node = WorkStack.back().first;
      ChildIt It = WorkStack.back().second;

      if (It == Node->succ_end())
        WorkStack.pop_back();
      else {
        // 
        _NodeTy *ChildNode = *It;
        ++WorkStack.back().second;
        unsigned VC = ++VisitCount[ChildNode];

        // for each edge (Node, ChildNode) in E(G) do
        if (LongestPathWeight[ChildNode] <
            LongestPathWeight[Node] + Node->getWeightTo(ChildNode)) {
          // Update the weight
          LongestPathWeight[ChildNode] =
            LongestPathWeight[Node] + Node->getWeightTo(ChildNode);
          // And the pred
          LongestPathPred[ChildNode] = Node;
        }

        // Only move forward when we visit the node from all its preds.
        if (VC == ChildNode->num_pred())
          WorkStack.push_back(std::make_pair(ChildNode, ChildNode->succ_begin()));
      }
      
    }

    // Build the path.
    for (_NodeTy *I = LongestPathPred[Exit]; I != Entry; I = LongestPathPred[I])
      PathSet.insert(I);
  }
};

typedef CompGraphNode<HWAOpInst> PostBindNodeType;
typedef CompGrapPath<HWAOpInst> PostBindNodePath;

typedef CompGraphNode<PostBindNodePath> PathGraphNodeType;

template<>
unsigned CompGraphNode<HWAOpInst>::updateWeightTo(PostBindNodeType* N) {
  // compute the weight.
  unsigned weight = 1;
  if (!isVREntry() && !N->isVRExit()) {
    // Wij = alpha * Fij + MINij + 1;
    HWAOpInst *Src = getData(), *Dst = N->getData();
    // Find flow dependency.
    if (Dst->isDepOn(Src))
      weight += 2;
    // Common inputs.
    SmallPtrSet<HWAtom*, 8> Inputs;
    Inputs.insert(Src->dep_begin(), Src->dep_end());
    Inputs.insert(Dst->dep_begin(), Dst->dep_end());

    weight += Src->getNumDeps() + Dst->getNumDeps() - Inputs.size();
  }

  WeightMap.insert(std::make_pair(N, weight));

  return weight;
}

template<>
bool CompGraphNode<HWAOpInst>::computeCompatible(_Self *N) {
  unsigned ThisSlot = getData()->getSlot(),
           NSlot = N->getData()->getSlot();

  if (unsigned II = getData()->getParent()->getII()) {
    ThisSlot = ThisSlot % II;
    NSlot = NSlot % II;
  }

  // FIXME: Handle multi-cycle atom.
  return (ThisSlot != NSlot);
}
} // end namespace

namespace llvm {

template<class DataTy> struct GraphTraits<Inverse<esyn::CompGraphNode<DataTy>*> > {
  typedef esyn::CompGraphNode<DataTy> NodeType;
  typedef typename esyn::CompGraphNode<DataTy>::pred_iterator ChildIteratorType;
  static NodeType *getEntryNode(NodeType* N) { return N; }
  static inline ChildIteratorType child_begin(NodeType *N) {
    return N->pred_begin();
  }
  static inline ChildIteratorType child_end(NodeType *N) {
    return N->pred_end();
  }
};

template<class DataTy> struct GraphTraits<Inverse<const esyn::CompGraphNode<DataTy>*> > {
  typedef esyn::CompGraphNode<DataTy> NodeType;
  typedef typename esyn::CompGraphNode<DataTy>::const_pred_iterator
    ChildIteratorType;
  static NodeType *getEntryNode(NodeType* N) { return N; }
  static inline ChildIteratorType child_begin(NodeType *N) {
    return N->pred_begin();
  }
  static inline ChildIteratorType child_end(NodeType *N) {
    return N->pred_end();
  }
};

template<class DataTy> struct GraphTraits<esyn::CompGraphNode<DataTy>*> {
  typedef esyn::CompGraphNode<DataTy> NodeType;
  typedef typename esyn::CompGraphNode<DataTy>::succ_iterator ChildIteratorType;
  static NodeType *getEntryNode(NodeType* N) { return N; }
  static inline ChildIteratorType child_begin(NodeType *N) {
    return N->succ_begin();
  }
  static inline ChildIteratorType child_end(NodeType *N) {
    return N->succ_end();
  }
};

template<class DataTy> struct GraphTraits<const esyn::CompGraphNode<DataTy>*> {
  typedef esyn::CompGraphNode<DataTy> NodeType;
  typedef typename esyn::CompGraphNode<DataTy>::const_succ_iterator
    ChildIteratorType;
  static NodeType *getEntryNode(NodeType* N) { return N; }
  static inline ChildIteratorType child_begin(NodeType *N) {
    return N->succ_begin();
  }
  static inline ChildIteratorType child_end(NodeType *N) {
    return N->succ_end();
  }
};

}

namespace esyn {
struct CompPathBinding : public BasicBlockPass {
  // Iterators
  typedef df_iterator<PostBindNodeType*, SmallPtrSet<PostBindNodeType*, 8>, false,
    GraphTraits<PostBindNodeType*> > wocg_df_it;
  typedef df_iterator<const PostBindNodeType*, SmallPtrSet<const PostBindNodeType*, 8>,
    false, GraphTraits<const PostBindNodeType*> > wocg_const_df_it;

  //PathGraphNodeType
  typedef df_iterator<PathGraphNodeType*, SmallPtrSet<PathGraphNodeType*, 8>, false,
    GraphTraits<PathGraphNodeType*> > pg_df_it;
  typedef df_iterator<const PathGraphNodeType*, SmallPtrSet<const PathGraphNodeType*, 8>,
    false, GraphTraits<const PathGraphNodeType*> > pg_const_df_it;


  // Types
  typedef std::pair<PostBindNodeType*, PostBindNodeType*> FUWOCGType;

  typedef std::map<enum HWResource::ResTypes, FUWOCGType> WOCGMapType;

  BumpPtrAllocator NodeAllocator;

  // WOCG
  WOCGMapType WOCG;
  // Path Graph
  PathGraphNodeType PGEntry, PGExit;

  HWAtomInfo *HI;

  FSMState *CurStage;

  unsigned ResCount;

  void clear();

  PostBindNodeType *getGraphEntry(enum HWResource::ResTypes T) {
    return WOCG[T].first;
  }

  PostBindNodeType *getGraphExit(enum HWResource::ResTypes T) {
    return WOCG[T].second;
  }
  // Build the Weighted Compatibility Graph.
  void buildWOCGForRes();
  void insertToWOCG(HWAPostBind *PB);
  // Build all operation in longest path to a function unit.
  void buildLongestPostBindPath();
  // Bind register to function unit.
  void bindFunUnitReg();

  static char ID;
  CompPathBinding();
  ~CompPathBinding();

  bool runOnBasicBlock(BasicBlock &BB);
  void getAnalysisUsage(AnalysisUsage &AU) const;
  void releaseMemory();
  bool doFinalization(Function &) { 
    ResCount = 0;
    return false;
  }
};
}

CompPathBinding::CompPathBinding()
  : BasicBlockPass(&ID), PGEntry(0, 1), PGExit(0, 0), ResCount(0) {
  for (unsigned i = HWResource::FirstResourceType,
      e = HWResource::LastResourceType; i != e; ++i) {
    PostBindNodeType *Entry = new PostBindNodeType(0, 1),
                     *Exit = new PostBindNodeType(0, 0);
    WOCG[(HWResource::ResTypes)i] = std::make_pair(Entry, Exit);
  }
}

CompPathBinding::~CompPathBinding() {
  clear();
   for (WOCGMapType::iterator I = WOCG.begin(), E = WOCG.end(); I != E; ++I) {
      FUWOCGType FWOCG = I->second;
      delete FWOCG.first;
      delete FWOCG.second;
  }
}

void CompPathBinding::clear() {
  for (WOCGMapType::iterator I = WOCG.begin(), E = WOCG.end(); I != E; ++I) {
    FUWOCGType FWOCG = I->second;
    FWOCG.first->clear();
    FWOCG.second->clear();
  }
  
  PGEntry.clear();
  PGExit.clear();

  NodeAllocator.Reset();
}

void CompPathBinding::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<HWAtomInfo>();
  AU.addRequired<ResourceConfig>();
  AU.setPreservesAll();
}

bool CompPathBinding::runOnBasicBlock(llvm::BasicBlock &BB) {
  // Are we disable resource binding?
  if (NoFUBinding)  return false;

  HI = &getAnalysis<HWAtomInfo>();
  CurStage = &HI->getStateFor(BB);

  DEBUG(
    dbgs() << "\n\nBefore binding:\n";
    for (usetree_iterator I = CurStage->usetree_begin(),
      E = CurStage->usetree_end(); I != E; ++I) {
        HWAtom *A = *I;
        dbgs() << "Schedule\n";
        A->dump();
        dbgs() << "To slot: " << A->getSlot() << '\n';
    }
    dbgs() << "\n\n";
  );

  // 1. Build WOCG_FUNTYPE.
  buildWOCGForRes();
  // 2. Find the longest path.
  buildLongestPostBindPath();
  // 3. Bind a register to the function unit.
  bindFunUnitReg();


  DEBUG(
    dbgs() << "\n\nAfter binding:\n";
    for (usetree_iterator I = CurStage->usetree_begin(),
      E = CurStage->usetree_end(); I != E; ++I) {
        HWAtom *A = *I;
        dbgs() << "Schedule\n";
        A->dump();
        dbgs() << "To slot: " << A->getSlot() << '\n';
    }
    dbgs() << "\n\n";
  );
  return false;
}

void CompPathBinding::bindFunUnitReg() {
  HWAOpInst &Exit = CurStage->getExitRoot();
  // For each Function unit(longest path graph node), bind a FU register.
  // For each nodes in
  for (pg_df_it I = pg_df_it::begin(&PGEntry), E = pg_df_it::end(&PGExit);
      I != E; ++I) {
    PathGraphNodeType &Node = **I;
    if (Node.isVRoot())
      continue;
    
    for (PostBindNodePath::path_iterator PI = Node->path_begin(),
        PE = Node->path_end(); PI != PE; ++PI) {
      HWAPreBind *A =cast<HWAPreBind>((*PI)->getData());
      DEBUG(dbgs() << "For PostBind Node: ");
      DEBUG(A->dump());
      Instruction *Inst = &A->getInst<Instruction>();
      // Bind a register to this function unit.
      HWReg *FUR = HI->allocaFURegister(A);
      HWAWrStg *WR = HI->getWrStg(A, FUR);
      DEBUG(dbgs() << "Create FU Register: ");
      DEBUG(WR->dump());

      bool FURegRead = false;
      std::vector<HWAtom *> WorkStack(A->use_begin(), A->use_end());
      while(!WorkStack.empty()) {
        HWAtom *Use = WorkStack.back();
        WorkStack.pop_back();

        // Do not make self loop.
        if (Use == WR) continue;
        // Replace the delay atom.
        if (HWADelay *Delay = dyn_cast<HWADelay>(Use)) {
          Delay->replaceAllUseBy(WR);
          A->removeFromList(Delay);
          continue;
        }

        DEBUG(dbgs() << "Replace Use: ");
        DEBUG(Use->dump());
        // Read the result for From this Register.
        Use->setDep(Use->getDepIt(A), WR);
      }
    }
  }
}

void CompPathBinding::buildLongestPostBindPath() {
  for (WOCGMapType::iterator I = WOCG.begin(), E = WOCG.end(); I != E; ++I) {
    FUWOCGType FWOCG = I->second;
    PostBindNodeType *Entry = FWOCG.first, *Exit = FWOCG.second;
    while (Entry->num_succ() != 0) {
      PostBindNodePath *Path = new (NodeAllocator) PostBindNodePath(Entry, Exit);
      // Increase the resource count, so we got a difference resource id;
      ++ResCount;

      DEBUG(dbgs() << "Longest Path:\n");
      for (PostBindNodePath::path_iterator I = Path->path_begin(),
        E = Path->path_end(); I != E; ++I ) {
          PostBindNodeType *Node = *I;
          // Bind to resource, fix me: the bitwitdh of the resource?
          HWAPreBind *PreBind =
            HI->bindToResource(*cast<HWAPostBind>(Node->getData()), ResCount);
          Node->updateData(PreBind);
          // Bind to resource
          DEBUG(PreBind->print(dbgs()));
          DEBUG(dbgs() << " at " << PreBind->getSlot() << '\n');
          Node->removeFromGraph();
      }

      PathGraphNodeType *Node = new (NodeAllocator) PathGraphNodeType(Path);

      // Instert to the path graph, but ignore the oreder at this moment;
      PGEntry.addSucc(Node);
    }
  }
}

void CompPathBinding::buildWOCGForRes() {
  for (usetree_iterator I = CurStage->usetree_begin(),
      E = CurStage->usetree_end(); I != E; ++I) {
    if (HWAPostBind *PB = dyn_cast<HWAPostBind>(*I))
      if (PB->getResClass() != HWResource::Trivial)      
        insertToWOCG(PB);
  } 
}

void CompPathBinding::insertToWOCG(HWAPostBind *PB) {
  PostBindNodeType *Entry = getGraphEntry(PB->getResClass()),
                   *Exit = getGraphExit(PB->getResClass()),
                   *Node = new (NodeAllocator) PostBindNodeType(PB);
  
  
  for (PostBindNodeType::succ_iterator I = Entry->succ_begin(), E = Entry->succ_end();
      I != E; ++I)
    if ((*I)->isCompatible(Node)) {
      DEBUG(
        dbgs() << "Find compatible: ";
        (**I)->print(dbgs());
        dbgs() << " at " << (**I)->getSlot() << ", ";
        PB->print(dbgs());
        dbgs() << " at " << PB->getSlot() << '\n';
      );

      // FIXME: Do mix difference type in a function unit.
      if (PB->getValue().getType() == (**I)->getValue().getType())      
        PostBindNodeType::makeEdge(*I, Node);
    }
  // Insert the edge
  PostBindNodeType::makeEdge(Entry, Node);
  PostBindNodeType::makeEdge(Node, Exit);
}

void CompPathBinding::releaseMemory() {
  clear();
}

char CompPathBinding::ID = 0;

Pass *esyn::createCompPathBindingPass() {
  return new CompPathBinding();
}
