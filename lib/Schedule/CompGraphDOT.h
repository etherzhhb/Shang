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
#include "CompGraph.h"

#include "llvm/Target/TargetRegisterInfo.h"
#include "llvm/CodeGen/LiveInterval.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/Support/GraphWriter.h"

namespace llvm {

template<typename T1, typename T2>
struct DOTGraphTraits<CompGraph<T1, T2>*> : public DefaultDOTGraphTraits{
  typedef CompGraph<T1, T2> GraphTy;
  typedef typename GraphTy::NodeTy NodeTy;
  typedef typename NodeTy::iterator NodeIterator;
  typedef CompGraphTraits<T1> Traits;

  DOTGraphTraits(bool isSimple=false) : DefaultDOTGraphTraits(isSimple) {}

  static std::string getEdgeSourceLabel(const NodeTy *Node,NodeIterator I){
    return itostr(Node->getWeightTo(*I));
  }

  std::string getNodeLabel(const NodeTy *Node, const GraphTy *Graph) {
    return Traits::getNodeLabel(Node->get());
  }

  static std::string getNodeAttributes(const NodeTy *Node,
                                       const GraphTy *Graph) {
    return "shape=Mrecord";
  }
};

template<typename T1, typename T2>
void CompGraph<T1, T2>::viewGraph() {
  ViewGraph(this, "CompatibilityGraph" + utostr_32(ID));
}

}

#endif
