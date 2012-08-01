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
#include "VSUnit.h"
#include "SchedulingBase.h"

#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/Support/GraphWriter.h"

namespace llvm {
// Control path graph.
template<bool IsCtrlPath>
struct VSchedGraphWrapper {
  const VSchedGraph *G;
  std::vector<const VSUnit*> SUs;

  /*implicit*/ inline VSchedGraphWrapper(const VSchedGraph *G);

  const VSchedGraph *operator->() const { return G; }

  typedef std::vector<const VSUnit*>::const_iterator const_iterator;
  const_iterator begin() const { return SUs.begin(); }
  const_iterator end() const { return SUs.end(); }
};

template<>
inline VSchedGraphWrapper<true>::VSchedGraphWrapper(const VSchedGraph *G)
  : G(G), SUs(cp_begin(G), cp_end(G)) {}

template<>
inline VSchedGraphWrapper<false>::VSchedGraphWrapper(const VSchedGraph *G)
  : G(G), SUs(dp_begin(G), dp_end(G))
{
  // The control-path scheduling units are also in the data-path
  // dependencies graph.
  SUs.insert(SUs.end(), cp_begin(G), cp_end(G));
}

template <bool IsCtrlPath>
struct GraphTraits<VSchedGraphWrapper<IsCtrlPath> >{
  typedef VSchedGraphWrapper<IsCtrlPath> GraphType;
  typedef const VSUnit NodeType;
  typedef VSUnit::const_use_iterator ChildIteratorType;

  static NodeType *getEntryNode(const GraphType &G) {
    return G->getEntryRoot();
  }

  static ChildIteratorType child_begin(NodeType *N) {
    return N->use_begin<IsCtrlPath>();
  }

  static ChildIteratorType child_end(NodeType *N) {
    return N->use_end<IsCtrlPath>();
  }

  typedef typename GraphType::const_iterator nodes_iterator;
  static nodes_iterator nodes_begin(const GraphType &G) {
    return G.begin();
  }

  static nodes_iterator nodes_end(const GraphType &G) {
    return G.end();
  }

  static unsigned size(const GraphType *G) {
    return (*G)->size<IsCtrlPath>();
  }
};

template<bool IsCtrlPath>
struct DOTGraphTraits<VSchedGraphWrapper<IsCtrlPath> >
  : public DefaultDOTGraphTraits {
  typedef VSchedGraphWrapper<IsCtrlPath> GraphType;
  typedef GraphTraits<VSchedGraphWrapper<IsCtrlPath> > GT;
  DOTGraphTraits(bool isSimple=false) : DefaultDOTGraphTraits(isSimple) {}

  static std::string getGraphName(const GraphType &G) {
    return G->getEntryBB()->getName();
  }

  /// If you want to override the dot attributes printed for a particular
  /// edge, override this method.
  static std::string getEdgeAttributes(typename GT::NodeType *Node,
                                       typename GT::ChildIteratorType EI,
                                       const GraphType &) {
    const VSUnit *Use = *EI;
    VDEdge UseEdge = Use->getEdgeFrom<IsCtrlPath>(Node);

    switch (UseEdge.getEdgeType()) {
    case VDEdge::ValDep:          return "";
    case VDEdge::MemDep:          return "color=blue,style=dashed";
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

    return utostr(UseEdge.getLatency()) + ',' + utostr(UseEdge.getDistance());
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

//template<>
//struct DOTGraphTraits<SchedulingBase*>
//  : public DOTGraphTraits<VSchedGraph*> {
//
//  DOTGraphTraits(bool isSimple = false) : DOTGraphTraits<VSchedGraph*>(isSimple) {}
//
//  static std::string getGraphName(const SchedulingBase *G) {
//    return  DOTGraphTraits<VSchedGraph*>::getGraphName(**G);
//  }
//
//  std::string getNodeLabel(const VSUnit *Node,
//                           const SchedulingBase *Graph) {
//    std::string Str;
//    raw_string_ostream ss(Str);
//    Node->print(ss);
//    ss << '[' << Graph->getASAPStep(Node) << ", "
//       << Graph->getALAPStep(Node)
//       << ']';
//    return ss.str();
//  }
//
//  static std::string getNodeAttributes(const VSUnit *Node,
//                                       const SchedulingBase *Graph)
//  {
//    return "shape=Mrecord";
//  }
//};
}

#endif
