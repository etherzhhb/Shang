//===- DesignMetrics.cpp - Estimate the metrics of the design ---*- C++ -*-===//
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
// This file implement the DesignMetrics class, which estimate the resource
// usage and speed performance of the design at early stage.
//
//===----------------------------------------------------------------------===//

#include "IR2Datapath.h"
#include "vtm/Passes.h"

#include "llvm/Pass.h"
#include "llvm/Target/TargetData.h"
#include "llvm/Support/InstIterator.h"

using namespace llvm;

namespace llvm {
// Wrapper for the external values.
class VASTValueOperand : public VASTValue {
public:
  const Value *const V;

  VASTValueOperand(const Value *V, unsigned Size)
    : VASTValue(VASTNode::vastCustomNode, Size),
      V(V) {}

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const VASTValueOperand *A) { return true; }
  static inline bool classof(const VASTNode *A) {
    return A->getASTType() == vastCustomNode;
  }

  void printAsOperandImpl(raw_ostream &OS, unsigned UB, unsigned LB) const {
    OS << *V << '[' << UB << ',' << LB << ']';
  }
};
}

namespace {
// FIXME: Move the class definition to a header file.
class DesignMetrics : public EarlyDatapathBuilderContext {
  // Data-path container to hold the optimized data-path of the design.
  DatapathContainer DPContainer;
  EarlyDatapathBuilder Builder;

  // TODO: Model the control-path, in the control-path, we can focus on the MUX
  // in the control-path, note that the effect of FU allocation&binding
  // algorithm should also be considered when estimating resource usage.

  // TODO: To not perform cycle-accurate speed performance estimation at the IR
  // layer, instead we should only care about the number of memory accesses.
  
  VASTImmediate *getOrCreateImmediate(uint64_t Value, int8_t BitWidth) {
    return DPContainer.getOrCreateImmediate(Value, BitWidth);
  }

  VASTValPtr createExpr(VASTExpr::Opcode Opc, ArrayRef<VASTValPtr> Ops,
                        unsigned UB, unsigned LB) {
    return DPContainer.createExpr(Opc, Ops, UB, LB);;
  }

  VASTValPtr getAsOperand(Value *Op, bool GetAsInlineOperand);

public:
  DesignMetrics(TargetData *TD) : Builder(*this, TD) {}

  void addInstruction(Instruction &Inst) {
    if (VASTValPtr V = Builder.visit(Inst))
      Builder.indexVASTExpr(&Inst, V);
  }
};

struct DesignMetricsPass : public FunctionPass {
  static char ID;

  DesignMetricsPass() : FunctionPass(ID) {}

  void getAnalysisUsage(AnalysisUsage &AU) const {
    AU.addRequired<TargetData>();
    AU.setPreservesAll();
  }

  bool runOnFunction(Function &F);
};
}

VASTValPtr DesignMetrics::getAsOperand(Value *Op, bool GetAsInlineOperand) {
  if (ConstantInt *Int = dyn_cast<ConstantInt>(Op)) {
    unsigned ConstantSize = Int->getType()->getScalarSizeInBits();
    // Do not create the immediate if it is too wide.
    if (ConstantSize <= 64)
      return getOrCreateImmediate(Int->getZExtValue(), ConstantSize);
  }

  if (VASTValPtr V = Builder.lookupExpr(Op)) {
    // Try to inline the operand if user ask to.
    if (GetAsInlineOperand) V = V.getAsInlineOperand();
    return V;
  }

  // Else we need to create a leaf node for the expression tree.
  VASTValueOperand *ValueOp
    = DPContainer.getAllocator()->Allocate<VASTValueOperand>();
    
  new (ValueOp) VASTValueOperand(Op, Builder.getValueSizeInBits(Op));

  // Remember the newly create VASTValueOperand, so that it will not be created
  // again.
  Builder.indexVASTExpr(Op, ValueOp);
  return ValueOp;
}

char DesignMetricsPass::ID = 0;

bool DesignMetricsPass::runOnFunction(Function &F) {
  DesignMetrics Metrics(&getAnalysis<TargetData>());

  for (inst_iterator I = inst_begin(F), E = inst_end(F); I != E; ++I)
    Metrics.addInstruction(*I);
  
  return false;
}

FunctionPass *llvm::createDesignMetricsPass() {
  return new DesignMetricsPass();
}
