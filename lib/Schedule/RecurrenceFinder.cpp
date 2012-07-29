//===- RecurrenceFinder.cpp - Find recurrences in schedule graph -*- C++ -*-===//
//
// Copyright: 2011 by SYSU EDA Group. all rights reserved.
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
// This file implement the recurrence finder which find recurrences in schedule
// graph by johnson's alogrithm.
// The algorithm of Johnson is based on the search for strong connected
// components in a graph. For a description of this part see:<br>
// Robert Tarjan: Depth-first search and linear graph algorithms. In: SIAM
// Journal on Computing. Volume 1, Nr. 2 (1972), pp. 146-160.<br>
//
//===----------------------------------------------------------------------===//

#include "VSUnit.h"
#include "SchedulingBase.h"

#include "llvm/Function.h"
#include "llvm/ADT/SCCIterator.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/IndexedMap.h"
#include "llvm/ADT/SparseBitVector.h"
#include "llvm/Support/raw_ostream.h"
#define DEBUG_TYPE "vtm-rec-finder"
#include "llvm/Support/Debug.h"

#include <map>

using namespace llvm;

//===----------------------------------------------------------------------===//
namespace {
class SubGraph;

struct SubGraphNode {
  const VSUnit *U;
  SubGraph *subGraph;

  SubGraphNode(const VSUnit *SU, SubGraph *SG)
    : U(SU), subGraph(SG) {}

  SubGraphNode(const SubGraphNode &O)
    : U(O.U), subGraph(O.subGraph) {}

  const SubGraphNode &operator=(const SubGraphNode &RHS);

  const VSUnit *getSUnit() const { return U; }
  unsigned getIdx() const {
    return U ? U->getIdx() : unsigned(VSchedGraph::NullSUIdx);
  }

  typedef SubGraphNode *result_type;
  result_type operator()(const VSUnit *U) const;

  typedef mapped_iterator<VSUnit::const_dep_iterator, SubGraphNode> ChildIt;

  ChildIt child_begin() const;
  ChildIt child_end() const;

  void dump() const {
    if (U) {
      dbgs() << U->getIdx() << " {";

      typedef VSUnit::const_dep_iterator it;
      for (it DI = cp_begin(U), DE = cp_end(U); DI != DE;++DI)
        dbgs() << " [" << DI->getIdx() << "]";

      dbgs() << "}\n";
    } else
      dbgs() << "dummy\n";
  }
};

class SubGraph {
  typedef SmallVector<const VSUnit*, 32> VSUnitVec;
  typedef SmallPtrSet<const VSUnit*, 32> VSUnitSet;

  const VSchedGraph *G;
  const VSUnit *GraphEntry;
  //Set of blocked nodes
  typedef SparseBitVector<> VSUnitFlags;
  VSUnitFlags blocked;
#ifdef XDEBUG
  //Stack holding current circuit
  VSUnitVec CurPath;
#endif
  //Map for B Lists
  typedef SmallVector<VSUnitSet, 64> BMapTy;
  BMapTy B;

  // SubGraph stuff
  typedef std::map<const VSUnit*, SubGraphNode*> NodeVecTy;
  NodeVecTy Nodes;
  SubGraphNode DummyNode;
public:
  unsigned CurIdx;
  unsigned RecMII;

  SubGraph(VSchedGraph *SG)
    : G(SG), GraphEntry(SG->getEntryRoot()), B(G->getNextSUIdx()),
      DummyNode(0, this), CurIdx(G->getEntryRoot()->getIdx()), RecMII(0) {
    // Add the Create the nodes, node that we will address the Nodes by the
    // the InstIdx of the VSUnit and this only works if they are sorted in
    // the VSUnits vector of SG.
    typedef VSchedGraph::iterator it;
    for (it I = cp_begin(SG), E = cp_end(SG); I != E; ++I)
      Nodes.insert(std::make_pair(*I, new SubGraphNode(*I, this)));
  }

  unsigned getRecMII() const { return RecMII; }

  ~SubGraph() { DeleteContainerSeconds(Nodes); }

  VSUnit::const_dep_iterator dummy_end() const {
    return cp_end(GraphEntry);
  }

  // Create or loop up a node.
  SubGraphNode *getNode(const VSUnit *SU) {
    assert(SU && "SU should not be NULL!");
    if (SU->getIdx() < CurIdx) return &DummyNode;

    NodeVecTy::const_iterator at = Nodes.find(SU);
    assert(at != Nodes.end() && "SubGraphNode not exists!");
    return at->second;
  }

  // Iterate over the subgraph start from idx.
  // Node: The atom list of FSMState already sort by getIdx.
  typedef mapped_iterator<VSchedGraph::const_iterator, SubGraphNode>
    nodes_iterator;

  nodes_iterator sub_graph_begin() {
    VSchedGraph::const_iterator I = cp_begin(G);
    while ((*I)->getIdx() < CurIdx && I != cp_end(G))
      ++I;

    return nodes_iterator(I, *getNode(*I));
  }
  nodes_iterator sub_graph_end() {
    return nodes_iterator(cp_end(G), DummyNode);
  }

  bool findAllCircuits();
  bool circuit(const VSUnit *CurNode, const VSUnit *LeastVertex,
               const VSUnitFlags &SCC, int CurLat, int CurDist);
  void addRecurrence(int CurLat, int CurDist);
  void unblock(const VSUnit *N);
};
}

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

typedef GraphTraits<SubGraphNode*> VSUSccGT;

typedef scc_iterator<SubGraphNode*, VSUSccGT> dep_scc_iterator;

void SubGraph::unblock(const VSUnit *N) {
  blocked.reset(N->getIdx());
  VSUnitSet &BN = B[N->getIdx()];
  for (VSUnitSet::iterator I = BN.begin(), E = BN.end(); I != E; ++I) {
    const VSUnit *W = *I;
    if(blocked.test(W->getIdx())) unblock(W);
  }
  BN.clear();
}

void SubGraph::addRecurrence(int CurLat, int CurDist) {
  if (CurDist <= 0) return;

  unsigned RecII = ceil(double(CurLat) / double(CurDist));
  //MSInfo->addRecurrence(RecII, Recurrence);
  RecMII = std::max(RecMII, RecII);

#ifdef XDEBUG
  DEBUG(dbgs() << "RecII: " << RecII << '\n';
  const VSUnit *LastAtom = CurPath.back();

  for (VSUnitVec::iterator I = CurPath.begin(), E = CurPath.end(); I != E; ++I) {
    const VSUnit *A = *I;
    VDEdge *Edge = LastAtom->getEdgeFrom(A);

    if (Edge->isLoopCarried()) dbgs() << "Backedge --> ";
    A->dump();
    LastAtom = A;
    // Dirty Hack.
    //Recurrence.push_back(const_cast<VSUnit*>(A));
  });
#endif
}

bool SubGraph::circuit(const VSUnit *CurNode, const VSUnit *LeastVertex,
                       const VSUnitFlags &SCC, int CurLat, int CurDist) {
  bool closed = false;

#ifdef XDEBUG
  CurPath.push_back(CurNode);
#endif
  blocked.set(CurNode->getIdx());

  VSUnitVec AkV;
  typedef VSUnit::const_dep_iterator it;
  for (it I = cp_begin(CurNode), E = cp_end(CurNode); I != E; ++I) {
    const VSUnit *N = *I;

    if (!const_cast<VSUnitFlags&>(SCC).test(N->getIdx())) continue;

    int LatIncr = I.getLatency();
    int DistIncr = I.getDistance();
    AkV.push_back(N);
    if (N == LeastVertex) {
      //We have a circuit, so add it to recurrent list.
      addRecurrence(CurLat + LatIncr, CurDist + DistIncr);
      closed = true;
    } else if (!blocked.test(N->getIdx())) {
      if (circuit(N, LeastVertex, SCC, CurLat + LatIncr, CurDist + DistIncr))
        closed = true;
    }
  }

  if (closed)
    unblock(CurNode);
  else
    for (VSUnitVec::iterator I = AkV.begin(), E = AkV.end(); I != E; ++I) {
      const VSUnit *U = *I;
      B[U->getIdx()].insert(CurNode);
    }

#ifdef XDEBUG
  // Pop current node.
  CurPath.pop_back();
#endif

  return closed;
}

bool SubGraph::findAllCircuits() {
  VSUnit *ExitRoot = G->getExitRoot();
  unsigned ExitIdx = ExitRoot->getIdx();
  DEBUG(dbgs() << "-------------------------------------\nFind all circuits in "
               << G->getEntryBB()->getName()
               << '\n');

  typedef std::vector<SubGraphNode*> SCCTy;
  SCCTy LeastSCCVec;
  VSUnitFlags SCCNodes;
  // While the subgraph not empty.
  while (CurIdx < ExitIdx) {
    DEBUG(dbgs() << "Current Idx: " << CurIdx << '\n');
    // Initialize the subgraph induced by {CurIdx, ...., ExitIdx}
    SubGraphNode *RootNode = getNode(ExitRoot);

    LeastSCCVec.clear();
    // least vertex in Vk
    SubGraphNode *LeastVertex = 0;

    // Iterate over all the SCCs in the graph to find the scc with the least
    // vertex
    for (dep_scc_iterator SCCI = dep_scc_iterator::begin(RootNode),
         SCCE = dep_scc_iterator::end(RootNode); SCCI != SCCE; ++SCCI) {
      SCCTy &nextSCC = *SCCI;

      if (nextSCC.size() == 1) {
        assert(!SCCI.hasLoop() && "No self loop expect in DDG!");
        continue;
      }

      // Find the lest vetex
      SubGraphNode *OldLeastVertex = LeastVertex;
      for (SCCTy::iterator I = nextSCC.begin(), E = nextSCC.end();
           I != E; ++I) {
        SubGraphNode *CurNode = *I;
        if (!LeastVertex || CurNode->getIdx() < LeastVertex->getIdx())
          LeastVertex = CurNode;
      }
      // Update Vk if we have new leastVertex.
      if (OldLeastVertex != LeastVertex) LeastSCCVec = nextSCC;
    }

    // No SCC?
    if (LeastSCCVec.empty())
      break;

    // Now we have the SCC with the least vertex.
    CurIdx = LeastVertex->getIdx();
    uint64_t complexity = 1;

    SCCNodes.clear();
    // Do some clear up.
    for (SCCTy::iterator I = LeastSCCVec.begin(), E = LeastSCCVec.end(); I != E; ++I){
      SubGraphNode *N = *I;
      unsigned ChildNums = std::distance(N->child_begin(), N->child_end());
      complexity *= ChildNums;

      // FIXME: Read the threshold from user script.
      if (complexity > UINT64_C(0x0020000000000000)) {
        MachineBasicBlock *MBB = G->getEntryBB();
        errs() << "Cannot analysis RecII with complexity " << complexity
               << " in BB " << MBB->getName()
               << " in Function " << MBB->getParent()->getFunction()->getName()
               << "!\n";
        return false;
      }

      SCCNodes.set(N->getIdx());
      blocked.reset(N->getIdx());
      B[N->getIdx()].clear();
    }

    // Find the circuits.
    circuit(LeastVertex->getSUnit(), LeastVertex->getSUnit(), SCCNodes, 0, 0);

    // Move forward.
    ++CurIdx;
  }

  return true;
}

//===----------------------------------------------------------------------===//
SubGraphNode::result_type SubGraphNode::operator()(const VSUnit *U) const {
  return subGraph->getNode(U);
}

const SubGraphNode &SubGraphNode::operator=(const SubGraphNode &RHS) {
  U = RHS.U;
  subGraph = RHS.subGraph;
  return *this;
}

SubGraphNode::ChildIt SubGraphNode::child_begin() const {
  if (U) return ChildIt(cp_begin(U), *this);
  // The node outside the subgraph.
  return ChildIt(subGraph->dummy_end(), *this);
}

SubGraphNode::ChildIt SubGraphNode::child_end() const {
  if (U) return ChildIt(cp_end(U), *this);

  // The node outside the subgraph.
  return ChildIt(subGraph->dummy_end(), *this);
}

//===----------------------------------------------------------------------===//
unsigned SchedulingBase::computeRecMII() {
  //// Find all recurrences with Johnson's algorithm.
  SubGraph SG(&G);

  // Do not pipeline if we cannot compute RecMII.
  if (!SG.findAllCircuits()) return 0;

  unsigned MaxRecII = SG.getRecMII();
  DEBUG(dbgs() << "RecMII: " << MaxRecII << '\n');
  // Dirty Hack: RecII must bigger than zero.
  return std::max(MaxRecII, 1u);
}
