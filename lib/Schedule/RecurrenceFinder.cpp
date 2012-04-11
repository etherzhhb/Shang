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
  unsigned getIdx() const { return U->getIdx(); }

  typedef SubGraphNode *result_type;
  result_type operator()(const VSUnit *U) const;

  typedef mapped_iterator<VSUnit::const_dep_iterator, SubGraphNode> ChildIt;

  ChildIt child_begin() const;
  ChildIt child_end() const;

  void dump() const {
    if (U) {
      dbgs() << U->getIdx() << " {";

      for (VSUnit::const_dep_iterator DI = U->dep_begin(), DE = U->dep_end();
           DI != DE;++DI)
        dbgs() << " [" << DI->getIdx() << "]";

      dbgs() << "}\n";
    } else
      dbgs() << "dummy\n";
  }
};

class SubGraph {
  typedef SmallVector<SubGraphNode*, 32> SubGrapNodeVec;
  typedef SmallPtrSet<SubGraphNode*, 64> SubGrapNodeSet;

  const VSchedGraph *G;
  const VSUnit *GraphEntry;
  // ModuloScheduleInfo *MSInfo;

  //Set of blocked nodes
  typedef std::map<const SubGraphNode*, bool> SubGrapNodeFlags;
  SubGrapNodeFlags blocked;
  //Stack holding current circuit
  SubGrapNodeVec CurPath;
  //Map for B Lists
  typedef std::map<const SubGraphNode*, SubGrapNodeSet> BMapTy;
  BMapTy B;

  // SubGraph stuff
  typedef std::map<const VSUnit*, SubGraphNode*> NodeVecTy;
  NodeVecTy Nodes;
  SubGraphNode DummyNode;
public:
  unsigned CurIdx, NumNodes;
  unsigned RecMII;

  SubGraph(VSchedGraph *SG)
    : G(SG), GraphEntry(SG->getEntryRoot()), DummyNode(0, this),
      CurIdx(G->getEntryRoot()->getIdx()), NumNodes(G->num_scheds()),
      RecMII(0) {
    // Add the Create the nodes, node that we will address the Nodes by the
    // the InstIdx of the VSUnit and this only works if they are sorted in
    // the VSUnits vector of SG.
    typedef VSchedGraph::sched_iterator it;
    for (it I = SG->sched_begin(), E = SG->sched_end(); I != E; ++I)
      Nodes.insert(std::make_pair(*I, new SubGraphNode(*I, this)));
  }

  unsigned getRecMII() const { return RecMII; }

  ~SubGraph() { DeleteContainerSeconds(Nodes); }

  VSUnit::const_dep_iterator dummy_end() const { return GraphEntry->dep_end(); }

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
  typedef mapped_iterator<VSchedGraph::sched_iterator, SubGraphNode>
    nodes_iterator;

  nodes_iterator sub_graph_begin() {
    VSchedGraph::sched_iterator I = G->sched_begin();
    while ((*I)->getIdx() < CurIdx && I != G->sched_end())
      ++I;

    return nodes_iterator(I, *getNode(*I));
  }
  nodes_iterator sub_graph_end() {
    return nodes_iterator(G->sched_end(), DummyNode);
  }

  bool findAllCircuits();
  bool circuit(SubGraphNode *CurNode, SubGraphNode *LeastVertex,
               const SubGrapNodeSet &SCC);
  void addRecurrence();
  void unblock(SubGraphNode *N);
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

void SubGraph::unblock(SubGraphNode *N) {
  blocked[N] = false;
  SubGrapNodeSet &BN = B[N];
  while (!BN.empty()) {
    SubGraphNode *W = *BN.begin();
    BN.erase(W);
    if(blocked[W]) unblock(W);
  }
}

void SubGraph::addRecurrence() {
  DEBUG(dbgs() << "\nRecurrence:\n");
  //std::vector<VSUnit*> Recurrence;
  unsigned TotalLatency = 0;
  unsigned TotalDistance = 0;
  const VSUnit *LastAtom = CurPath.back()->getSUnit();

  for (SubGrapNodeVec::iterator I = CurPath.begin(), E = CurPath.end(); I != E; ++I) {
    SubGraphNode *N = *I;

    const VSUnit *A = N->getSUnit();
    VDEdge *Edge = LastAtom->getEdgeFrom(A);

    TotalLatency += Edge->getLatency();
    DEBUG(if (Edge->isLoopCarried()) dbgs() << "Backedge --> ";);

    TotalDistance += Edge->getItDst();

    DEBUG(N->dump());
    LastAtom = A;
    // Dirty Hack.
    //Recurrence.push_back(const_cast<VSUnit*>(A));
  }

  unsigned RecII = ceil((double)TotalLatency / TotalDistance);
  //MSInfo->addRecurrence(RecII, Recurrence);
  RecMII = std::max(RecMII, RecII);
  DEBUG(dbgs() << "RecII: " << RecII << '\n');
}

bool SubGraph::circuit(SubGraphNode *CurNode, SubGraphNode *LeastVertex,
                       const SubGrapNodeSet &SCC) {
  bool closed = false;

  CurPath.push_back(CurNode);
  blocked[CurNode] = true;

  SubGrapNodeVec AkV;
  for (SubGraphNode::ChildIt I = CurNode->child_begin(),
       E = CurNode->child_end(); I != E; ++I) {
    SubGraphNode *N = *I;
    if (SCC.count(N)) AkV.push_back(N);
  }

  for (SubGrapNodeVec::iterator I = AkV.begin(), E = AkV.end(); I != E; ++I) {
    SubGraphNode *N = *I;
    if (N == LeastVertex) {
      //We have a circuit, so add it to recurrent list.
      addRecurrence();
      closed = true;
    } else if (!blocked[N] && circuit(N, LeastVertex, SCC))
      closed = true;
  }

  if (closed)
    unblock(CurNode);
  else
    for (SubGrapNodeVec::iterator I = AkV.begin(), E = AkV.end(); I != E; ++I) {
      SubGraphNode *N = *I;
      B[N].insert(CurNode);
    }

  // Pop current node.
  CurPath.pop_back();

  return closed;
}

bool SubGraph::findAllCircuits() {
  VSUnit *ExitRoot = G->getExitRoot();
  unsigned ExitIdx = ExitRoot->getIdx();
  DEBUG(dbgs() << "-------------------------------------\nFind all circuits in "
               << G->getMachineBasicBlock()->getName()
               << '\n');

  SubGrapNodeSet SCCNodes;
  // While the subgraph not empty.
  while (CurIdx < ExitIdx) {
    DEBUG(dbgs() << "Current Idx: " << CurIdx << '\n');
    // Initialize the subgraph induced by {CurIdx, ...., ExitIdx}
    SubGraphNode *RootNode = getNode(ExitRoot);

    SCCNodes.clear();
    // least vertex in Vk
    SubGraphNode *LeastVertex = 0;

    // Iterate over all the SCCs in the graph to find the scc with the least
    // vertex
    for (dep_scc_iterator SCCI = dep_scc_iterator::begin(RootNode),
          SCCE = dep_scc_iterator::end(RootNode); SCCI != SCCE; ++SCCI) {
      typedef std::vector<SubGraphNode*> SCCTy;
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
      if (OldLeastVertex != LeastVertex) {
        SCCNodes.clear();
        SCCNodes.insert(nextSCC.begin(), nextSCC.end());
      }
    }

    // No SCC?
    if (SCCNodes.empty())
      break;

    // Now we have the SCC with the least vertex.
    CurIdx = LeastVertex->getIdx();
    uint64_t complexity = 1;

    // Do some clear up.
    for (SubGrapNodeSet::iterator I = SCCNodes.begin(), E = SCCNodes.end();
         I != E; ++I) {
      SubGraphNode *N = *I;
      unsigned ChildNums = std::distance(N->child_begin(), N->child_end());
      complexity *= ChildNums;

      // FIXME: Read the threshold from user script.
      if (complexity > 0x10000000) {
        MachineBasicBlock *MBB = G->getMachineBasicBlock();
        errs() << "Cannot analysis RecII with complexity " << complexity
               << " in BB " << MBB->getName()
               << " in Function " << MBB->getParent()->getFunction()->getName()
               << "!\n";
        return false;
      }

      blocked[N] = false;
      B[N].clear();
    }

    // Find the circuits.
    circuit(LeastVertex, LeastVertex, SCCNodes);

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
  if (U) return ChildIt(U->dep_begin(), *this);
  // The node outside the subgraph.
  return ChildIt(subGraph->dummy_end(), *this);
}

SubGraphNode::ChildIt SubGraphNode::child_end() const {
  if (U) return ChildIt(U->dep_end(), *this);

  // The node outside the subgraph.
  return ChildIt(subGraph->dummy_end(), *this);
}

//===----------------------------------------------------------------------===//
unsigned SchedulingBase::computeRecMII() {
  //// Find all recurrences with Johnson's algorithm.
  SubGraph SG(&State);

  // Do not pipeline if we cannot compute RecMII.
  if (!SG.findAllCircuits()) return 0;

  unsigned MaxRecII = SG.getRecMII();
  DEBUG(dbgs() << "RecMII: " << MaxRecII << '\n');
  // Dirty Hack: RecII must bigger than zero.
  return std::max(MaxRecII, 1u);
}
