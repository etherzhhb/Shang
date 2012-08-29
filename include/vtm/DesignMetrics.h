//===--- DesignMetrics.h - Estimate the metrics of the design ---*- C++ -*-===//
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
// This file define the DesignMetrics class, which estimate the resource
// usage and speed performance of the design at early stage.
//
//===----------------------------------------------------------------------===//

#ifndef VTM_DESIGN_METRICS
#define VTM_DESIGN_METRICS

#include "llvm/Support/DataTypes.h"

namespace llvm {
class DesignMetricsImpl;
class Instruction;
class BasicBlock;
class Function;
class TargetData;
class raw_ostream;

class DesignMetrics {
  DesignMetricsImpl *Impl;
public:
  class DesignCost {
    uint64_t DatapathCost;
    unsigned NumAddrBusFanin;
    unsigned NumDataBusFanin;
    unsigned NumNontrivialBB;
  public:
    DesignCost(uint64_t DatapathCost = 0, unsigned NumAddrBusFanin = 0,
               unsigned NumDataBusFanin = 0, unsigned NumNontrivialBB = 0);

    uint64_t getCostInc(unsigned Multiply) const;

    operator bool() const { return DatapathCost; }

    // Debugging Support
    void print(raw_ostream &OS) const;
    void dump() const;
  };

  explicit DesignMetrics(TargetData *TD);
  ~DesignMetrics();

  // Visit the LLVM IR and collect the information to estimate the metrics of
  // the design.
  void visit(Instruction &Inst);
  void visit(Instruction *Inst) { visit(*Inst); }

  void visit(BasicBlock &BB);
  void visit(BasicBlock *BB) { visit(*BB); }

  void visit(Function &F);
  void visit(Function *F) { visit(*F); }

  template<typename Iterator>
  void visit(Iterator I, Iterator E) {
    while (I != E)
      visit(*I++);
  }

  void reset();

  // Visit all data-path expression and compute the cost.
  DesignCost getCost() const;
  unsigned getNumCalls() const;
};

//===----------------------------------------------------------------------===//
// Debugging Support

inline raw_ostream& operator<<(raw_ostream &OS,
                               const DesignMetrics::DesignCost &Cost) {
  Cost.print(OS);
  return OS;
}
}

#endif
