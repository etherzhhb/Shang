//===- RecurrenceFinder.cpp - Find recurrences in schedule graph -*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implement the recurrence finder which find recurrences in schedule
// graph by johnson's alogrithm.
//
//===----------------------------------------------------------------------===//

#include "VSUnit.h"

#include "llvm/ADT/SCCIterator.h"
#include "llvm/ADT/STLExtras.h"
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
  typedef std::vector<SubGraphNode*> SubGrapNodeVec;
  typedef std::set<SubGraphNode*> SubGrapNodeSet;

  const VSchedGraph *G;
  const VSUnit *GraphEntry;
  // ModuloScheduleInfo *MSInfo;

  //Set of blocked nodes
  SubGrapNodeSet blocked;
  //Stack holding current circuit
  SubGrapNodeVec stack;
  //Map for B Lists
  std::map<SubGraphNode*, SubGrapNodeSet> B;

  // SCC with least vertex.
  SubGrapNodeVec Vk;

  // SubGraph stuff
  typedef std::vector<SubGraphNode*> NodeVecTy;
  NodeVecTy Nodes;
  SubGraphNode DummyNode;
public:
  unsigned CurIdx, NumNodes;
  unsigned RecMII;

  SubGraph(VSchedGraph *SG)
    : G(SG), GraphEntry(SG->getEntryRoot()), DummyNode(0, this),
      CurIdx(G->getEntryRoot()->getIdx()), NumNodes(G->getNumSUnits()),
      RecMII(0) {
    // Add the Create the nodes, node that we will address the Nodes by the
    // the InstIdx of the VSUnit and this only works if they are sorted in
    // the VSUnits vector of SG.
    for (VSchedGraph::const_iterator I = SG->begin(), E = SG->end(); I != E; ++I)
      Nodes.push_back(new SubGraphNode(*I, this));
  }

  unsigned getRecMII() const { return RecMII; }

  ~SubGraph() { DeleteContainerPointers(Nodes); }

  VSUnit::const_dep_iterator dummy_end() const { return GraphEntry->dep_end(); }

  // Create or loop up a node.
  SubGraphNode *getNode(const VSUnit *SU) {
    assert(SU && "SU should not be NULL!");
    if (SU->getIdx() < CurIdx) return &DummyNode;

    return Nodes[SU->getIdx()];
  }

  // Iterate over the subgraph start from idx.
  // Node: The atom list of FSMState already sort by getIdx.
  typedef mapped_iterator<VSchedGraph::const_iterator, SubGraphNode>
    nodes_iterator;

  nodes_iterator sub_graph_begin() {
    VSchedGraph::const_iterator I = G->begin();
    while ((*I)->getIdx() < CurIdx && I != G->end())
      ++I;

    return nodes_iterator(I, *getNode(*I));
  }
  nodes_iterator sub_graph_end() {
    return nodes_iterator(G->end(), DummyNode);
  }

  void findAllCircuits();
  bool circuit(SubGraphNode *CurNode, SubGraphNode *LeastVertex);
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
  blocked.erase(N);

  while (!B[N].empty()) {
    SubGraphNode *W = *B[N].begin();
    B[N].erase(W);
    if(blocked.count(W))
      unblock(W);
  }
}

void SubGraph::addRecurrence() {
  DEBUG(dbgs() << "\nRecurrence:\n");
  //std::vector<VSUnit*> Recurrence;
  unsigned TotalLatency = 0;
  unsigned TotalDistance = 0;
  const VSUnit *LastAtom = stack.back()->getSUnit();

  for (SubGrapNodeVec::iterator I = stack.begin(), E = stack.end(); I != E; ++I) {
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

bool SubGraph::circuit(SubGraphNode *CurNode, SubGraphNode *LeastVertex) {
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

void SubGraph::findAllCircuits() {
  VSUnit *ExitRoot = G->getExitRoot();
  unsigned ExitIdx = ExitRoot->getIdx();
  DEBUG(dbgs() << "-------------------------------------\nFind all circuits in "
               << G->getMachineBasicBlock()->getName()
               << '\n');

  // While the subgraph not empty.
  while (CurIdx < ExitIdx) {
    DEBUG(dbgs() << "Current Idx: " << CurIdx << '\n');
    // Initialize the subgraph induced by {CurIdx, ...., ExitIdx}
    SubGraphNode *RootNode = getNode(ExitRoot);

    Vk.clear();
    // least vertex in Vk
    SubGraphNode *LeastVertex = 0;

    // Iterate over all the SCCs in the graph to find the scc with the least
    // vertex
    for (dep_scc_iterator SCCI = dep_scc_iterator::begin(RootNode),
          SCCE = dep_scc_iterator::end(RootNode); SCCI != SCCE; ++SCCI) {
      SubGrapNodeVec &nextSCC = *SCCI;

      if (nextSCC.size() == 1) {
        assert(!SCCI.hasLoop() && "No self loop expect in DDG!");
        continue;
      }

      // Find the lest vetex
      SubGraphNode *OldLeastVertex = LeastVertex;
      for (SubGrapNodeVec::iterator I = nextSCC.begin(), E = nextSCC.end();
           I != E; ++I) {
        SubGraphNode *CurNode = *I;
        if (!LeastVertex || CurNode->getIdx() < LeastVertex->getIdx())
          LeastVertex = CurNode;
      }
      // Update Vk if leastVe
      if (OldLeastVertex != LeastVertex)
        Vk = nextSCC;
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
    // Find the circuits.
    circuit(LeastVertex, LeastVertex);

    // Move forward.
    ++CurIdx;
  }
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
unsigned VSchedGraph::computeRecMII() {
  //// Find all recurrences with Johnson's algorithm.
  SubGraph SG(this);
  SG.findAllCircuits();
  unsigned MaxRecII = SG.getRecMII();
  DEBUG(dbgs() << "RecMII: " << MaxRecII << '\n');
  // Dirty Hack: RecII must bigger than zero.
  return std::max(MaxRecII, 1u);
}
