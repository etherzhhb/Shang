//===- ModuloScheduleInfo.cpp - ModuleSchedule information analyze -*- C++ -*-===//
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
// This file implement the ModuleSchedule information computation pass describe
// in
// Josep, L. (1996). Swing Modulo Scheduling: A Lifetime-Sensitive Approach.
//
//===----------------------------------------------------------------------===//

#include "ModuloScheduleInfo.h"
#include "HWAtomPasses.h"

#include "llvm/Analysis/LoopInfo.h"
#include "llvm/ADT/SCCIterator.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/Support/CommandLine.h"
#define DEBUG_TYPE "vbe-ms-info"
#include "llvm/Support/Debug.h"

#include <algorithm>

using namespace llvm;
using namespace esyn;

static cl::opt<bool>
NoModuloSchedule("disable-modulo-schedule",
          cl::desc("vbe - Do not preform modulo schedule"),
          cl::Hidden, cl::init(false));

//===----------------------------------------------------------------------===//
class HWSubGraph;

struct SubGraphNode {
  unsigned FirstNode;
  const HWAtom *A;
  HWSubGraph *SubGraph;

  SubGraphNode(unsigned First, const HWAtom *Atom,
                   HWSubGraph *subGraph)
    : FirstNode(First), A(Atom), SubGraph(subGraph) {}

  SubGraphNode(const SubGraphNode &O)
    : FirstNode(O.FirstNode), A(O.A), SubGraph(O.SubGraph) {}

  const SubGraphNode &operator=(const SubGraphNode &RHS);

  const HWAtom *getAtom() const { return A; }
  unsigned getIdx() const { return A->getIdx(); }

  typedef SubGraphNode *result_type;
  result_type operator()(const HWAtom *Atom) const;

  typedef mapped_iterator<HWAtom::const_dep_iterator, SubGraphNode> ChildIt;

  ChildIt child_begin() const;
  ChildIt child_end() const;

  void dump() const {
    if (A) {
      dbgs() << A->getIdx() << " {";

      for (HWAtom::const_dep_iterator DI = A->dep_begin(), DE = A->dep_end(); DI != DE;
           ++DI)
        dbgs() << " [" << DI->getIdx() << "]";

      dbgs() << "}\n";
    } else
      dbgs() << "dummy\n";
  }
};

// GraphTraits
namespace llvm {
  template<> struct GraphTraits<SubGraphNode*> {
    typedef SubGraphNode NodeType;
    typedef SubGraphNode::ChildIt ChildIteratorType;
    static NodeType *getEntryNode(NodeType* N) { return N; }
    static inline ChildIteratorType child_begin(NodeType *N) {
      return N->child_begin();
    }
    static inline ChildIteratorType child_end(NodeType *N) {
      return N->child_end();
    }
  };
}

typedef GraphTraits<SubGraphNode*> HWAtomSccGT;

typedef scc_iterator<SubGraphNode*, HWAtomSccGT> dep_scc_iterator;

//===----------------------------------------------------------------------===//
// Find all recurrents with Johnson's algorithm.
class HWSubGraph {
  typedef std::vector<SubGraphNode*> SubGrapNodeVec;
  typedef std::set<SubGraphNode*> SubGrapNodeSet;

  const FSMState *GraphEntry;
  ModuloScheduleInfo *MSInfo;

  //Set of blocked nodes
  SubGrapNodeSet blocked;
  //Stack holding current circuit
  SubGrapNodeVec stack;
  //Map for B Lists
  std::map<SubGraphNode*, SubGrapNodeSet> B;
  //
  SubGrapNodeSet Visited;
  // SCC with least vertex.
  SubGrapNodeVec Vk;

  // SubGraph stuff
  typedef std::map<const HWAtom*, SubGraphNode*> CacheTy;
  CacheTy ExtCache;
  
  unsigned CurIdx;
  unsigned RecMII;
public:
  HWSubGraph(FSMState *Entry, ModuloScheduleInfo *MS)
    : GraphEntry(Entry), MSInfo(MS), CurIdx(GraphEntry->getIdx()), RecMII(0) {}

  unsigned getRecMII() const { return RecMII; }

  ~HWSubGraph() { releaseCache(); }

  HWAtom::const_dep_iterator dummy_end() const { return GraphEntry->dep_end(); }

  // Create or loop up a node.
  SubGraphNode *getNode(const HWAtom *A) {
    CacheTy::iterator at = ExtCache.find(A);
    if (at != ExtCache.end())
      return at->second;

    SubGraphNode *NewNode = new SubGraphNode(CurIdx, A, this);

    ExtCache.insert(std::make_pair(A, NewNode));
    return NewNode;
  }

  void releaseCache() {
    // Release the temporary nodes in subgraphs.
    for (CacheTy::iterator I = ExtCache.begin(),
      E = ExtCache.end(); I != E; ++I) {
        delete I->second;
    }
    ExtCache.clear();
  }

  // Iterate over the subgraph start from idx.
  // Node: The atom list of FSMState already sort by getIdx.
  typedef mapped_iterator<FSMState::AtomVecTy::const_iterator, SubGraphNode>
    nodes_iterator;

  nodes_iterator sub_graph_begin() {
    FSMState::AtomVecTy::const_iterator I = GraphEntry->begin();
    while ((*I)->getIdx() < CurIdx && I != GraphEntry->end())
      ++I;

    return nodes_iterator(I, *getNode(*I));
  }
  nodes_iterator sub_graph_end() {
    return nodes_iterator(GraphEntry->end(), *getNode(0));
  }

  void findAllCircuits();
  bool circuit(SubGraphNode *CurNode, SubGraphNode *LeastVertex);
  void addRecurrence();
  void unblock(SubGraphNode *N);
};

void HWSubGraph::unblock(SubGraphNode *N) {
  blocked.erase(N);

  while (!B[N].empty()) {
    SubGraphNode *W = *B[N].begin();
    B[N].erase(W);
    if(blocked.count(W))
      unblock(W);
  }
}

void HWSubGraph::addRecurrence() {
  DEBUG(dbgs() << "\nRecurrence:\n");
  //std::vector<HWAtom*> Recurrence;
  unsigned TotalLatency = 0;
  unsigned TotalDistance = 0;
  const HWAtom *LastAtom = stack.back()->getAtom();
  
  for (SubGrapNodeVec::iterator I = stack.begin(), E = stack.end(); I != E; ++I) {
    SubGraphNode *N = *I;

    const HWAtom *A = N->getAtom();
    TotalLatency += A->getLatency();
    
    HWEdge *Edge = LastAtom->getEdgeFrom(A);
    if (Edge->isBackEdge()) {
      //  assert(TotalDistance == 0 && "Multiple back edge?"); 
      DEBUG(dbgs() << "Backedge --> ");
    }
    TotalDistance += Edge->getItDst();
  
    DEBUG(N->dump());
    LastAtom = A;
    // Dirty Hack.
    //Recurrence.push_back(const_cast<HWAtom*>(A));
  }

  unsigned RecII = TotalLatency / TotalDistance;
  //MSInfo->addRecurrence(RecII, Recurrence);
  RecMII = std::max(RecMII, RecII);
  DEBUG(dbgs() << "RecII: " << RecII << '\n');
}

bool HWSubGraph::circuit(SubGraphNode *CurNode, SubGraphNode *LeastVertex) {
  bool ret = false;

  stack.push_back(CurNode);
  blocked.insert(CurNode);

  SubGrapNodeSet AkV;
  for (SubGraphNode::ChildIt I = CurNode->child_begin(),
       E = CurNode->child_end(); I != E; ++I) {
    SubGraphNode *N = *I;
    if (std::find(Vk.begin(), Vk.end(), N) != Vk.end())
      AkV.insert(N);
  }

  for (SubGrapNodeSet::iterator I = AkV.begin(), E = AkV.end(); I != E; ++I) {
    SubGraphNode *N = *I;
    if (N == LeastVertex) {
      //We have a circuit, so add it to recurrent list.
      addRecurrence();
      ret = true;
    } else if (!blocked.count(N) && circuit(N, LeastVertex))
      ret = true;
  }

  if (ret)
    unblock(CurNode);
  else
    for (SubGrapNodeSet::iterator I = AkV.begin(), E = AkV.end(); I != E; ++I) {
      SubGraphNode *N = *I;
      B[N].insert(CurNode);
    }
  
  // Pop current node.
  stack.pop_back();

  return ret;
}


void HWSubGraph::findAllCircuits() {
  HWAOpFU *ExitRoot = GraphEntry->getExitRoot();
  unsigned ExitIdx = ExitRoot->getIdx();

  // While the subgraph not empty.
  while (CurIdx < ExitIdx) {
    DEBUG(dbgs() << "Current Idx: " << CurIdx << '\n');
    // Initialize the subgraph induced by {CurIdx, ...., ExitIdx}
    SubGraphNode *RootNode = getNode(ExitRoot);

    //Iterate over all the SCCs in the graph
    Visited.clear();
    Vk.clear();
    // least vertex in Vk
    SubGraphNode *LeastVertex = 0;

    //Find scc with the least vertex
    for (nodes_iterator I = sub_graph_begin(), E = sub_graph_end();
        I != E; ++I) {
      SubGraphNode *Node = *I;
      // If the Node visited.
      if (!Visited.insert(Node).second)
        continue;

      for (dep_scc_iterator SCCI = dep_scc_iterator::begin(RootNode),
           SCCE = dep_scc_iterator::end(RootNode); SCCI != SCCE; ++SCCI) {
        SubGrapNodeVec &nextSCC = *SCCI;  
        SubGraphNode *FirstNode = nextSCC.front();
        // If FirstNode visited.
        if (!Visited.insert(FirstNode).second)
          continue;
        
        if (nextSCC.size() == 1) {
          assert(!SCCI.hasLoop() && "No self loop expect in DDG!");
          continue;
        }

        // The entire SCC visited.
        Visited.insert(nextSCC.begin() + 1, nextSCC.end());
        // Find the lest vetex
        SubGraphNode *OldLeastVertex = LeastVertex;
        for (SubGrapNodeVec::iterator I = (*SCCI).begin(),
             E = (*SCCI).end();I != E; ++I) {
          SubGraphNode *CurNode = *I;
          if (!LeastVertex || CurNode->getIdx() < LeastVertex->getIdx())
            LeastVertex = CurNode;
        }
        // Update Vk if leastVe
        if (OldLeastVertex != LeastVertex)
          Vk = nextSCC; 
      }
    }

    // No SCC?
    if (Vk.empty())
      break;
    
    // Now we have the SCC with the least vertex.
    CurIdx = LeastVertex->getIdx();
    // Do some clear up.
    for (SubGrapNodeVec::iterator I = Vk.begin(), E = Vk.end(); I != E; ++I) {
      SubGraphNode *N = *I;
      blocked.erase(N);
      B[N].clear();
    }
    // Find the circuit.
    circuit(LeastVertex, LeastVertex);

    // Move forward.
    ++CurIdx;
    
    //
    releaseCache();
  }
}

//===----------------------------------------------------------------------===//
SubGraphNode::result_type SubGraphNode::operator()(const HWAtom *Atom) const {
  return Atom->getIdx() < FirstNode ? SubGraph->getNode(0) 
                                    : SubGraph->getNode(Atom); 
}

const SubGraphNode & SubGraphNode::operator=(const SubGraphNode &RHS) {
  FirstNode = RHS.FirstNode;
  A = RHS.A;
  SubGraph = RHS.SubGraph;
  return *this;
}

SubGraphNode::ChildIt SubGraphNode::child_begin() const {
  if (A) return ChildIt(A->dep_begin(), *this);
  // The node outside the subgraph.
  return ChildIt(SubGraph->dummy_end(), *this);
}

SubGraphNode::ChildIt SubGraphNode::child_end() const {
  if (A) return ChildIt(A->dep_end(), *this);

  // The node outside the subgraph.
  return ChildIt(SubGraph->dummy_end(), *this);
}

//===----------------------------------------------------------------------===//
bool ModuloScheduleInfo::isModuloSchedulable() const {
  // Are we disable modulo schedule?
  if (NoModuloSchedule) return false;
  
  BasicBlock *BB = State->getBasicBlock();
  Loop *L = LI->getLoopFor(BB);
  // States that not in loops are not MSable.
  if (!L) return false;

  // Only schedule single block loops.
  if (L->getBlocks().size() > 1)
    return false;
  
  return true;
}

void ModuloScheduleInfo::addRecurrence(unsigned II, rec_vector Rec) {
  RecList.insert(std::make_pair(II, Rec));
}

unsigned ModuloScheduleInfo::computeRecMII() {
  //// Find all recurrents with Johnson's algorithm.
  HWSubGraph SubGraph(State, this);
  SubGraph.findAllCircuits();
  unsigned MaxRecII = SubGraph.getRecMII();
  DEBUG(dbgs() << "RecMII: " << MaxRecII << '\n');

  return MaxRecII;
}

unsigned ModuloScheduleInfo::computeMII() {
  unsigned RecMII = computeRecMII();
  unsigned ResMII = computeResMII();
  return std::max(RecMII, ResMII);
}

unsigned ModuloScheduleInfo::computeResMII() const {
  std::map<HWFUnit*, unsigned> TotalResUsage;
  for (FSMState::iterator I = State->begin(), E = State->end(); I != E; ++I)
    if (HWAOpFU *A = dyn_cast<HWAOpFU>(*I)) {
      ++TotalResUsage[A->getFUnit()];
    }

  unsigned MaxResII = 0;
  typedef std::map<HWFUnit*, unsigned>::iterator UsageIt;
  for (UsageIt I = TotalResUsage.begin(), E = TotalResUsage.end(); I != E; ++I){
      MaxResII = std::max(MaxResII,
                          I->second / I->first->getTotalFUs());
  }
  DEBUG(dbgs() << "ResMII: " << MaxResII << '\n');
  return MaxResII;
}

ModuloScheduleInfo::~ModuloScheduleInfo() {
  clear();
}

void ModuloScheduleInfo::clear() {
  RecList.clear();
}
