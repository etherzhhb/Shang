//===-- CompGraphTraits.h - Compatibility Graph Traits Template ---*- C++ -*-===//
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
// This file defines the little CompGraphTraits<X> template class that should be
// specialized by classes that want to be the data type of Compatibility Graph.
//
//===----------------------------------------------------------------------===//
#ifndef COMPATIBILITY_GRAPH_TRAITS_H
#define COMPATIBILITY_GRAPH_TRAITS_H

namespace llvm {
// CompGraphTraits - This class should be specialized by different node types...
// which is why the default version is empty.
template<class NodeType>
struct CompGraphTraits {
  // Elements to provide:

  // static bool isEarlier(NodeType *LHS, NodeType *RHS);
  //    Return if LHS is earlier than RHS.

  // static bool compatible(NodeType *LHS, NodeType *RHS);
  //    Return if LHS is compatible with RHS.

  typedef typename NodeType::UnknownNodeTypeError T;
};
}

#endif
