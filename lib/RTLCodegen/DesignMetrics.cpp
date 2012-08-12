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
#include "llvm/ADT/DenseSet.h"
#define DEBUG_TYPE "vtm-design-metrics"
#include "llvm/Support/Debug.h"

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

  // The data-path value which are used by control-path operations.
  typedef std::set<VASTValue*> ValSetTy;
  ValSetTy LiveOutedVal;

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

  // Visit the expression tree whose root is Root and return the cost of the
  // tree, insert all visited data-path nodes into Visited.
  uint64_t getExprTreeFUCost(VASTValPtr Root, ValSetTy &Visited) const;
  uint64_t getFUCost(VASTValue *V) const;
public:
  DesignMetrics(TargetData *TD) : Builder(*this, TD) {}

  void addInstruction(Instruction &Inst);

  // Visit all data-path expression and compute the cost.
  uint64_t getDatapathFUCost() const;
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

void DesignMetrics::addInstruction(Instruction &Inst) {
  if (VASTValPtr V = Builder.visit(Inst)) {
    Builder.indexVASTExpr(&Inst, V);
    return;
  }

  // Else Inst is a control-path instruction, all its operand are live-outed.
  // A live-outed data-path expression and its children should never be
  // eliminated.
  typedef Instruction::op_iterator op_iterator;
  for (op_iterator I = Inst.op_begin(), E = Inst.op_end(); I != E; ++I) {
    if (VASTValPtr V = Builder.lookupExpr(*I))
      LiveOutedVal.insert(V.get());
  }
}

uint64_t DesignMetrics::getFUCost(VASTValue *V) const {
  VASTExpr *Expr = dyn_cast<VASTExpr>(V);
  // We can only estimate the cost of VASTExpr.
  if (!Expr) return 0;

  unsigned ValueSize = std::min(V->getBitWidth(), 64u);

  switch (Expr->getOpcode()) {
  default: break;

  case VASTExpr::dpAdd: return VFUs::AddCost[ValueSize];
  case VASTExpr::dpMul: return VFUs::MulCost[ValueSize];
  case VASTExpr::dpShl:
  case VASTExpr::dpSRA:
  case VASTExpr::dpSRL: return VFUs::ShiftCost[ValueSize];
  }

  return 0;
}

uint64_t DesignMetrics::getExprTreeFUCost(VASTValPtr Root, ValSetTy &Visited)
                                          const {
  uint64_t Cost = 0;
  // FIXME: Provide a generic depth-first search iterator for VASTValue tree.
  typedef VASTValue::dp_dep_it ChildIt;
  std::vector<std::pair<VASTValue*, ChildIt> > WorkStack;

  WorkStack.push_back(std::make_pair(Root.get(),
                                     VASTValue::dp_dep_begin(Root.get())));

  Cost += getFUCost(Root.get());
  
  while (!WorkStack.empty()) {
    VASTValue *N;
    ChildIt It;
    tie(N, It) = WorkStack.back();

    // All children are visited.  
    if (It == VASTValue::dp_dep_end(N)) {
      WorkStack.pop_back();
      continue;
    }

    // Depth first traverse the child of current node.
    VASTValue *ChildNode = It->get().get();
    ++WorkStack.back().second;

    // Had we visited this node?
    if (!Visited.insert(ChildNode).second) continue;

    // Visit the node and compute its FU cost.
    Cost += getFUCost(ChildNode);

    WorkStack.push_back(std::make_pair(ChildNode,
                                       VASTValue::dp_dep_begin(ChildNode)));
  }

  return Cost;
}

uint64_t DesignMetrics::getDatapathFUCost() const {
  uint64_t Cost = 0;
  ValSetTy Visited;

  typedef ValSetTy::const_iterator iterator;
  for (iterator I = LiveOutedVal.begin(), E = LiveOutedVal.end(); I != E; ++I)
    Cost += getExprTreeFUCost(*I, Visited);

  return Cost;
}

char DesignMetricsPass::ID = 0;

bool DesignMetricsPass::runOnFunction(Function &F) {
  DesignMetrics Metrics(&getAnalysis<TargetData>());

  for (inst_iterator I = inst_begin(F), E = inst_end(F); I != E; ++I)
    Metrics.addInstruction(*I);

  DEBUG(dbgs() << "Data-path cost of function " << F.getName() << ':'
               << Metrics.getDatapathFUCost() << '\n');
  
  return false;
}

FunctionPass *llvm::createDesignMetricsPass() {
  return new DesignMetricsPass();
}
