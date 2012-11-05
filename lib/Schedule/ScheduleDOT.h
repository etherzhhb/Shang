//===---- ScheduleDOT.h - DOTGraphTraits for Schedule Graph -------*- C++ -*-===//
//
//                      The Shang HLS frameowrk                               //
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
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
template<bool IsCtrlPath>
struct DOTGraphTraits<VSchedGraphWrapper<IsCtrlPath> >
  : public DefaultDOTGraphTraits {
  typedef VSchedGraphWrapper<IsCtrlPath> GraphType;
  typedef GraphTraits<VSchedGraphWrapper<IsCtrlPath> > GT;
  DOTGraphTraits(bool isSimple=false) : DefaultDOTGraphTraits(isSimple) {}

  static std::string getGraphName(const GraphType &G) {
    return "Dependency Graph";
  }

  /// If you want to override the dot attributes printed for a particular
  /// edge, override this method.
  template<typename GraphType_>
  static std::string getEdgeAttributes(typename GT::NodeType *Node,
                                       typename GT::ChildIteratorType EI,
                                       const GraphType_ &) {
    const VSUnit *Use = *EI;
    VDEdge UseEdge = Use->getEdgeFrom<IsCtrlPath>(Node);

    switch (UseEdge.getEdgeType()) {
    case VDEdge::ValDep:          return "";
    case VDEdge::ChainSupporting: return "color=blue";
    case VDEdge::MemDep:          return "color=blue,style=dashed";
    case VDEdge::LinearOrder:     return "color=green";
    case VDEdge::CtrlDep:         return "color=green,style=dashed";
    case VDEdge::FixedTiming:     return "color=red";
    }

    llvm_unreachable("Unexpected edge type!");
    return "";
  }

  static std::string getEdgeSourceLabel(typename GT::NodeType *Node,
                                        typename GT::ChildIteratorType EI) {
    const VSUnit *Use = *EI;
    VDEdge UseEdge = Use->getEdgeFrom<IsCtrlPath>(Node);

    return utostr(UseEdge.getLatency()) + ',' + itostr(UseEdge.getDistance());
  }

  std::string getNodeLabel(typename GT::NodeType *Node, const GraphType &) {
    std::string Str;
    raw_string_ostream ss(Str);
    Node->print(ss);
    return ss.str();
  }

  static std::string getNodeAttributes(const void *,
                                       const GraphType &) {
    return "shape=Mrecord";
  }
};

template<bool IsCtrlPath>
struct DOTGraphTraits<Scheduler<IsCtrlPath>*>
  : public DOTGraphTraits<VSchedGraphWrapper<IsCtrlPath> > {
  typedef Scheduler<IsCtrlPath> *GraphType;
  typedef GraphTraits<VSchedGraphWrapper<IsCtrlPath> > GT;

  DOTGraphTraits(bool isSimple = false)
    : DOTGraphTraits<VSchedGraphWrapper<IsCtrlPath> >(isSimple) {}

  static std::string getGraphName(const GraphType &G) {
    return  DOTGraphTraits<VSchedGraphWrapper<IsCtrlPath> >::getGraphName(**G);
  }

  std::string getNodeLabel(typename GT::NodeType *Node, const GraphType &G) {
    std::string Str;
    raw_string_ostream ss(Str);
    Node->print(ss);
    ss << '[' << G->getASAPStep(Node) << ", " << G->getALAPStep(Node) << ']';
    return ss.str();
  }

  static std::string getNodeAttributes(typename GT::NodeType *Node,
                                       const GraphType &G) {
    return "shape=Mrecord";
  }
};
}

#endif
