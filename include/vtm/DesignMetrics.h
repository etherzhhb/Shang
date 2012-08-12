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

class DesignMetrics {
  DesignMetricsImpl *Impl;
public:
  explicit DesignMetrics(TargetData *TD);
  ~DesignMetrics();

  // Visit the LLVM IR and collect the information to estimate the metrics of
  // the design.
  void visit(Instruction &Inst);
  void visit(BasicBlock &BB);
  void visit(Function &F);

  void reset();

  // Visit all data-path expression and compute the cost.
  uint64_t getDatapathFUCost() const;
};
}

#endif
