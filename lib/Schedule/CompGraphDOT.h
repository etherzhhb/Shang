//===---- ScheduleDOT.h - DOTGraphTraits for Schedule Graph -------*- C++ -*-===//
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
// This file define the DOTGraphTraits for Schedule Graph.
//
//===----------------------------------------------------------------------===//
//

#ifndef VTM_SCHEDULE_DOT
#define VTM_SCHEDULE_DOT
#include "CompGraph.h"

#include "llvm/Target/TargetRegisterInfo.h"
#include "llvm/CodeGen/LiveInterval.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/Support/GraphWriter.h"

namespace llvm {

template<>
struct DOTGraphTraits<CompGraph<LiveInterval*, unsigned>*>
  : public DefaultDOTGraphTraits{
  typedef CompGraphNode<LiveInterval*> NodeTy;
  typedef CompGraph<LiveInterval*, unsigned> GraphTy;

  DOTGraphTraits(bool isSimple=false) : DefaultDOTGraphTraits(isSimple) {}

  static std::string getEdgeSourceLabel(const NodeTy *Node, NodeTy::iterator I){
    return itostr(Node->getWeightTo(*I));
  }

  std::string getNodeLabel(const NodeTy *Node, const GraphTy *Graph) {
    std::string Str;
    raw_string_ostream ss(Str);
    ss << PrintReg(Node->get()->reg);
    return ss.str();
  }

  static std::string getNodeAttributes(const NodeTy *Node,
                                       const GraphTy *Graph) {
    return "shape=Mrecord";
  }
};

template<>
void CompGraph<LiveInterval*, unsigned>::viewGraph() {
  ViewGraph(this, "CompatibilityGraph" + utostr_32(ID));
}

}

#endif
