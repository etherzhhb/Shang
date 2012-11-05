//===-------------- IR2Datapath.h - LLVM IR <-> VAST ------------*- C++ -*-===//
//
//                      The Shang HLS frameowrk                               //
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file define the classes which convert LLVM IR to VASTExprs.
//
//===----------------------------------------------------------------------===//

#ifndef VTM_LLVM_IR_TO_DATAPATH
#define VTM_LLVM_IR_TO_DATAPATH

#include "VASTExprBuilder.h"

#include "llvm/Support/InstVisitor.h"
#include "llvm/ADT/ValueMap.h"

namespace llvm {
class TargetData;

class EarlyDatapathBuilderContext : public VASTExprBuilderContext {
public:
  virtual VASTValPtr getAsOperand(Value *Op,
                                  bool GetAsInlineOperand = true) {
    return 0;
  }
};

class EarlyDatapathBuilder : public VASTExprBuilder,
                             public InstVisitor<EarlyDatapathBuilder,
                                                VASTValPtr> {
  EarlyDatapathBuilderContext &getContext() {
    return static_cast<EarlyDatapathBuilderContext&>(Context);
  }

public:
  typedef DenseMap<Value*, VASTValPtr> ValueMapTy;
private:
  ValueMapTy Value2Expr;
  TargetData *TD;

public:
  EarlyDatapathBuilder(EarlyDatapathBuilderContext &Context, TargetData *TD)
    : VASTExprBuilder(Context), TD(TD) {}

  TargetData *getTargetData() const { return TD; }

  unsigned getValueSizeInBits(const Value *V) const;
  unsigned getValueSizeInBits(const Value &V) const {
    return getValueSizeInBits(&V);
  }

  VASTValPtr getAsOperand(Value *Op, bool GetAsInlineOperand = true) {
    return getContext().getAsOperand(Op, GetAsInlineOperand);
  }

  // Value mapping.
  VASTValPtr createAndIndexExpr(Instruction *I, bool mayExisted = false);
  VASTValPtr lookupExpr(Value *Val) const;
  VASTValPtr indexVASTExpr(Value *Val, VASTValPtr V);

  // Converting CastInst
  VASTValPtr visitTruncInst(TruncInst &I);
  VASTValPtr visitZExtInst(ZExtInst &I);
  VASTValPtr visitSExtInst(SExtInst &I);
  VASTValPtr visitBitCastInst(BitCastInst &I);

  VASTValPtr visitSelectInst(SelectInst &I);

  VASTValPtr visitICmpInst(ICmpInst &I);

  VASTValPtr visitGetElementPtrInst(GetElementPtrInst &I);

  VASTValPtr visitBinaryOperator(BinaryOperator &I);

  VASTValPtr visitInstruction(Instruction &I) {
    // Unhandled instructions can be safely ignored.
    return VASTValPtr();
  }
};
}

#endif
