//===---- CompGraph.h - Compatibility Graph ---------------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines the CompGraphNode and CompGraph, which are used in the
// resource allocation and binding algorithm.
//
//===----------------------------------------------------------------------===//
#ifndef COMPATIBILITY_GRAPH_H
#define COMPATIBILITY_GRAPH_H

#include "llvm/ADT/GraphTraits.h"
#include "llvm/ADT/SmallPtrSet.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/STLExtras.h"

#include <vector>
#include <map>

namespace llvm {
class raw_ostream;

template<class T>
class CompGraphNode {
  typedef CompGraphNode<T> Self;
  // The underlying data.
  T N;

  typedef SmallPtrSet<Self*, 8> NodeVecTy;
  // Predecessors and Successors.
  NodeVecTy Preds, Succs;

  typedef std::map<Self*, unsigned> WeightVecTy;
  WeightVecTy SuccWeights;

public:
  explicit CompGraphNode(T Node = T()) : N(Node) {}

  bool isTrivial() const { return N == T(); }

  T get() const { return N; }
  T operator->() const { return N; }

  //void print(raw_ostream &OS) const;
  //void dump() const;

  bool compatible(Self &Other, CompGraphQuery<T> &Q) const {
    return Q.compatible(get(), Other.get());
  }

  bool isEarlier(Self &Other, CompGraphQuery<T> &Q) const {
    return Q.isEarlier(get(), Other.get());
  }

  //typedef NodeVecTy::iterator iterator;
  typedef typename NodeVecTy::const_iterator iterator;

  iterator succ_begin() const { return Succs.begin(); }
  iterator succ_end()   const { return Succs.end(); }
  unsigned num_succ()   const { return Succs.size(); }
  bool     succ_empty() const { return Succs.empty(); }

  iterator pred_begin() const { return Preds.begin(); }
  iterator pred_end()   const { return Preds.end(); }
  unsigned num_pred()   const { return Preds.size(); }
  bool     pred_empty() const { return Preds.empty(); }

  unsigned getWeightTo(iterator I) const {
    return SuccWeights.find(*I)->second;
  }

  // Unlink the Succ from current node.
  void unlinkSucc(CompGraphNode *Succ) {
    Succs.erase(Succ);
    SuccWeights.erase(Succ);

    // Current node is not the predecessor of succ node too.
    Succ->Preds.erase(this);
  }

  // Unlink the Pred from current node.
  void unlinkPred(CompGraphNode *Pred) {
    Preds.erase(Pred);

    // Current node is not the successor of pred node too.
    Pred->Succs.erase(this);
  }

  void unlink() {
    for (iterator I = succ_begin(), E = succ_end(); I != E; ++I)
      unlinkSucc(*I);

    for (iterator I = pred_begin(), E = pred_end(); I != E; ++I)
      unlinkPred(*I);
  }

  static void MakeEdge(CompGraphNode &Src, CompGraphNode &Dst, unsigned Weight){
    Src.Succs.insert(&Dst);
    Src.SuccWeights.insert(std::make_pair(&Dst, Weight));
    Dst.Preds.insert(&Src);
  }

  static void MakeEdge(CompGraphNode &Src, CompGraphNode &Dst,
                       CompGraphQuery<T> &Q){
    MakeEdge(Src, Dst, Q.calcWeight(Src.get(), Dst.get()));
  }
};

template<class T> struct GraphTraits<CompGraphNode<T>*> {
  typedef CompGraphNode<T> NodeType;
  typedef typename NodeType::iterator ChildIteratorType;
  static NodeType *getEntryNode(NodeType* N) { return N; }
  static inline ChildIteratorType child_begin(NodeType *N) {
    return N->succ_begin();
  }
  static inline ChildIteratorType child_end(NodeType *N) {
    return N->succ_end();
  }
};

template<class T>
class CompGraph {
public:
  typedef CompGraphNode<T> NodeTy;
  typedef CompGraphQuery<T> QueryTy;
private:
  QueryTy &Q;
  typedef std::map<T, NodeTy*> NodeMapTy;
  // The dummy entry node of the graph.
  NodeTy Entry, Exit;
  // Nodes vector.
  NodeMapTy Nodes;

  struct second {
    typedef NodeTy *result_type;

    result_type operator()(typename NodeMapTy::value_type v) const {
      return v.second;
    }
  }; 

public:
  CompGraph(QueryTy &q) : Q(q) {}

  ~CompGraph() {
    DeleteContainerSeconds(Nodes);
  }

  typedef typename NodeTy::iterator iterator;

  // All nodes (except exit node) are successors of the entry node.
  iterator begin() { return Entry.succ_begin(); }
  iterator end()   { return Entry.succ_end(); }

  NodeTy *GetOrCreateNode(T N) {
    assert(N && "Unexpected null pointer pass to GetOrCreateNode!");
    NodeTy *&Node = Nodes[N];
    // Create the node if it not exisits yet.
    if (Node == 0) Node = new NodeTy(N);

    return Node;
  }



  void buildGraph() {
    typedef std::vector<NodeTy*> NodeVecTy;
    typedef typename NodeVecTy::iterator NodeVecIt;
    NodeVecTy Visited;

    for (iterator I = begin(), E = end(); I != E; ++I) {
      NodeTy *LHS = *I;

      for (NodeVecIt NI = Visited.begin(), NE = Visited.end(); NI != NE;++NI) {
        NodeTy *RHS = *NI;

        if (!LHS->compatible(*RHS, Q)) continue;

        if (LHS->isEarlier(*RHS, Q))
          NodeTy::MakeEdge(*LHS, *RHS, Q);
        else
          NodeTy::MakeEdge(*RHS, *LHS, Q);
      }

      Visited.push_back(LHS);
    }

    // Add the edge from entry node to all other nodes.
    for (NodeVecIt NI = Visited.begin(), NE = Visited.end(); NI != NE;++NI) {
      NodeTy::MakeEdge(Entry, **NI, Q.getVirtualEdgeWeight());
      NodeTy::MakeEdge(**NI, Exit, Q.getVirtualEdgeWeight());
    }

    // Also insert the entry node to the map.
    //Nodes.insert(std::make_pair((T*)0, &Entry));
  }

  void deleteNode(NodeTy *N) {
    Nodes.erase(N->get());
    N->unlink();
    delete N;
  }

  // Return true if the longest path is not trivial (have more than 1 nodes).
  bool findLongestPath(SmallVectorImpl<T> &Path, bool DelNodes = false) {
    std::map<NodeTy*, unsigned> LenMap;

    std::map<NodeTy*, NodeTy*> PathPred;
    std::map<NodeTy*, unsigned> PathWeight;

    //for each vertex v in topOrder(G) do
    typedef typename NodeTy::iterator ChildIt;
    SmallVector<std::pair<NodeTy*, ChildIt>, 32> WorkStack;
    std::map<NodeTy*, unsigned> VisitCount;

    WorkStack.push_back(std::make_pair(&Entry, Entry.succ_begin()));
    PathWeight[&Entry] = 0;

    while (!WorkStack.empty()) {
      NodeTy *Node = WorkStack.back().first;
      ChildIt It = WorkStack.back().second;

      if (It == Node->succ_end())
        WorkStack.pop_back();
      else {
        //
        NodeTy *ChildNode = *It;
        ++WorkStack.back().second;
        unsigned VC = ++VisitCount[ChildNode];

        // for each edge (Node, ChildNode) in E(G) do
        if (PathWeight[ChildNode] < PathWeight[Node] + Node->getWeightTo(It)) {
          // Update the weight
          PathWeight[ChildNode] =
            PathWeight[Node] + Node->getWeightTo(It);
          // And the pred
          PathPred[ChildNode] = Node;
        }

        // Only move forward when we visit the node from all its preds.
        if (VC == ChildNode->num_pred())
          WorkStack.push_back(std::make_pair(ChildNode, ChildNode->succ_begin()));
      }
    }

    unsigned NumNodes = 0;
    // Build the path.
    for (NodeTy *I = PathPred[&Exit]; I && I != &Entry; I = PathPred[I]) {
      Path.push_back(I->get());
      if (DelNodes) deleteNode(I);
      ++NumNodes;
    }

    return NumNodes > 1;
  }

  void viewGraph();
};

template <class T> struct GraphTraits<CompGraph<T>*>
  : public GraphTraits<CompGraphNode<T>*> {
  
  typedef typename CompGraph<T>::iterator nodes_iterator;
  static nodes_iterator nodes_begin(CompGraph<T> *G) {
    return G->begin();
  }
  static nodes_iterator nodes_end(CompGraph<T> *G) {
    return G->end();
  }
};

}

#endif
