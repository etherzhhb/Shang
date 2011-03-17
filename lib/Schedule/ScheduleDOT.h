//===---- ScheduleDOT.h - DOTGraphTraits for Schedule Graph -------*- C++ -*-===//
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
// This file define the DOTGraphTraits for Schedule Graph.
//
//===----------------------------------------------------------------------===//
//

#ifndef VTM_SCHEDULE_DOT
#define VTM_SCHEDULE_DOT
#include "VSUnit.h"
#include "SchedulingBase.h"

#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/Support/GraphWriter.h"

namespace llvm {

template<>
struct DOTGraphTraits<VSchedGraph*> : public DefaultDOTGraphTraits {

  DOTGraphTraits(bool isSimple=false) : DefaultDOTGraphTraits(isSimple) {}

  static std::string getGraphName(const VSchedGraph *G) {
    return G->getMachineBasicBlock()->getName();
  }

  /// If you want to override the dot attributes printed for a particular
  /// edge, override this method.
  template<typename GraphType>
  static std::string getEdgeAttributes(const VSUnit *Node,
                                       VSUnit::use_iterator EI,
                                       const GraphType &Grap) {
    const VSUnit *Use = *EI;
    VDEdge *UseEdge = Use->getEdgeFrom(Node);

    switch (UseEdge->getEdgeType()) {
    case VDEdge::edgeValDep:    return "";
    case VDEdge::edgeMemDep:    return "color=blue,style=dashed";
    case VDEdge::edgeCtrlDep:   return "color=green,style=dashed";
    }

    return "";
  }

  static std::string getEdgeSourceLabel(const VSUnit *Node,
                                        VSUnit::use_iterator EI) {
    const VSUnit *Use = *EI;
    VDEdge *UseEdge = Use->getEdgeFrom(Node);

    return utostr(UseEdge->getLatency()) + ',' + utostr(UseEdge->getItDst());
  }

  std::string getNodeLabel(const VSUnit *Node, const VSchedGraph *Graph) {
    std::string Str;
    raw_string_ostream ss(Str);
    Node->print(ss);
    return ss.str();
  }

  static std::string getNodeAttributes(const VSUnit *Node,
                                       const VSchedGraph *Graph) {
    return "shape=Mrecord";
  }
};

template<>
struct DOTGraphTraits<SchedulingBase*>
  : public DOTGraphTraits<VSchedGraph*> {

  DOTGraphTraits(bool isSimple = false) : DOTGraphTraits<VSchedGraph*>(isSimple) {}

  static std::string getGraphName(const SchedulingBase *G) {
    return  DOTGraphTraits<VSchedGraph*>::getGraphName(&G->getState());
  }

  std::string getNodeLabel(const VSUnit *Node,
                           const SchedulingBase *Graph) {
    std::string Str;
    raw_string_ostream ss(Str);
    Node->print(ss);
    ss << '[' << Graph->getASAPStep(Node) << ", " << Graph->getALAPStep(Node)
       << ']';
    return ss.str();
  }

  static std::string getNodeAttributes(const VSUnit *Node,
                                       const SchedulingBase *Graph)
  {
    return "shape=Mrecord";
  }
};
}

#endif
