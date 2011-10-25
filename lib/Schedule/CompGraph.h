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
  T *N;

  typedef SmallVector<Self*, 8> NodeVecTy;

  // Predecessors and Successors.
  NodeVecTy Preds, Succs;

public:
  explicit CompGraphNode(T *Node = 0) : N(Node) {}

  bool isEntry() const { return N == 0; }

  T *get() const { return N; }
  T *operator->() const { return N; }

  //void print(raw_ostream &OS) const;
  //void dump() const;

  bool compatible(Self &Other) const {
    return CompGraphTraits<T>::compatible(get(), Other.get());
  }

  bool isEarlier(Self &Other) const {
    return CompGraphTraits<T>::isEarlier(get(), Other.get());
  }

  //typedef NodeVecTy::iterator iterator;
  typedef typename NodeVecTy::const_iterator iterator;

  iterator succ_begin() { return Succs.begin(); }
  iterator succ_end() { return Succs.end(); }

  static void MakeEdge(CompGraphNode &Src, CompGraphNode &Dst) {
    assert(!Dst.isEntry() && "Entry node cannot be destination!");
    Src.Succs.push_back(&Dst);
    Dst.Preds.push_back(&Src);
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

private:
  typedef std::map<T*, NodeTy*> NodeMapTy;
  // The dummy entry node of the graph.
  NodeTy Entry;
  // Nodes vector.
  NodeMapTy Nodes;

  struct second {
    typedef NodeTy *result_type;

    result_type operator()(typename NodeMapTy::value_type v) const {
      return v.second;
    }
  }; 

public:
  CompGraph() : Entry(0) {}

  ~CompGraph() {
    DeleteContainerSeconds(Nodes);
  }

  NodeTy *GetOrCreateNode(T *N) {
    assert(N && "Unexpected null pointer pass to GetOrCreateNode!");
    NodeTy *&Node = Nodes[N];
    // Create the node if it not exisits yet.
    if (Node == 0) Node = new NodeTy(N);

    return Node;
  }

  typedef typename NodeMapTy::iterator map_iterator;
  typedef mapped_iterator<map_iterator, second> iterator;

  iterator begin() { return iterator(Nodes.begin(), second()); }
  iterator end()   { return iterator(Nodes.end(), second()); }

  void buildGraph() {
    typedef std::vector<NodeTy*> NodeVecTy;
    typedef typename NodeVecTy::iterator NodeVecIt;
    NodeVecTy Visited;

    for (iterator I = begin(), E = end(); I != E; ++I) {
      NodeTy *LHS = *I;

      for (NodeVecIt NI = Visited.begin(), NE = Visited.end(); NI != NE;++NI) {
        NodeTy *RHS = *NI;

        if (!LHS->compatible(*RHS)) continue;

        if (LHS->isEarlier(*RHS))
          NodeTy::MakeEdge(*LHS, *RHS);
        else
          NodeTy::MakeEdge(*RHS, *LHS);
      }

      Visited.push_back(LHS);
    }

    // Add the edge from entry node to all other nodes.
    for (NodeVecIt NI = Visited.begin(), NE = Visited.end(); NI != NE;++NI)
      NodeTy::MakeEdge(Entry, **NI);

    // Also insert the entry node to the map.
    //Nodes.insert(std::make_pair((T*)0, &Entry));
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
