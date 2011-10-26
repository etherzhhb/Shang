//===-- CompGraphQuery.h - Compatibility Graph Query Template ---*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines the little CompGraphTraits<X> template class that should be
// specialized by classes that want to be the data type of Compatibility Graph.
//
//===----------------------------------------------------------------------===//
#ifndef COMPATIBILITY_GRAPH_QUERY_H
#define COMPATIBILITY_GRAPH_QUERY_H

namespace llvm {
// CompGraphQuery - This class should be specialized by different node types...
// which is why the default version is empty.
template<class NodeType>
struct CompGraphQuery {
  // Elements to provide:

  // static bool isEarlier(NodeType *LHS, NodeType *RHS);
  //    Return if LHS is earlier than RHS.

  // static bool compatible(NodeType *LHS, NodeType *RHS);
  //    Return if LHS is compatible with RHS.

  typedef typename NodeType::UnknownNodeTypeError T;
};
}

#endif
