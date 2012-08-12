//===------------ IR2Datapath.cpp - LLVM IR <-> VAST ------------*- C++ -*-===//
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
// This file implement the classes which convert LLVM IR to VASTExprs.
//
//===----------------------------------------------------------------------===//

#include "IR2Datapath.h"

#include "llvm/Target/TargetData.h"

using namespace llvm;
// TD
unsigned EarlyDatapathBuilder::getValueSizeInBits(const Value *V) const {
  unsigned SizeInBits = TD->getTypeSizeInBits(V->getType());
  assert(SizeInBits && "Size of V is unknown!");
  return SizeInBits;
}

VASTValPtr EarlyDatapathBuilder::lookupExpr(Value *Val) const {
   ValueMapTy::const_iterator at = Value2Expr.find(Val);
   return at == Value2Expr.end() ? 0 : at->second;
 }

VASTValPtr EarlyDatapathBuilder::indexVASTExpr(Value *Val, VASTValPtr V) {
  bool inserted = Value2Expr.insert(std::make_pair(Val, V)).second;
  assert(inserted && "RegNum already indexed some value!");

  return V;
}

//VASTValPtr EarlyDatapathBuilder::createAndIndexExpr(Instruction *I,
//                                                    bool mayExisted){
//  VASTValPtr &Expr = Value2Expr[I];
//  assert((!Expr || mayExisted) && "Expression had already been created!");
//
//  if (Expr) return Expr;
//
//  return (Expr = visit(I));
//}

VASTValPtr EarlyDatapathBuilder::visitTruncInst(TruncInst &I) {
  VASTValPtr Operand = getAsOperand(I.getOperand(0));
  // Truncate the value by bitslice expression.
  return buildBitSliceExpr(Operand, getValueSizeInBits(I), 0);
}

VASTValPtr EarlyDatapathBuilder::visitZExtInst(ZExtInst &I) {
  VASTValPtr Operand = getAsOperand(I.getOperand(0));
  return padHigherBits(Operand, getValueSizeInBits(I), false);
}

VASTValPtr EarlyDatapathBuilder::visitSExtInst(SExtInst &I) {
  VASTValPtr Operand = getAsOperand(I.getOperand(0));
  VASTValPtr SignBit = getSignBit(Operand);
  unsigned ResultSize = getValueSizeInBits(I);
  unsigned NumExtendBits = ResultSize - Operand->getBitWidth();
  VASTValPtr ExtendBits = buildExpr(VASTExpr::dpBitRepeat, SignBit,
                                    getOrCreateImmediate(NumExtendBits, 8),
                                    NumExtendBits);
  VASTValPtr Ops[] = { ExtendBits, Operand };
  return buildBitCatExpr(Ops, ResultSize);
}

VASTValPtr EarlyDatapathBuilder::visitBitCastInst(BitCastInst &I) {
  VASTValPtr Operand = getAsOperand(I.getOperand(0));

  assert(getValueSizeInBits(I) == Operand->getBitWidth()
         && "Cast between types with different size found!");
  return Operand;
}

VASTValPtr EarlyDatapathBuilder::visitSelectInst(SelectInst &I) {
  return buildExpr(VASTExpr::dpSel,
                   getAsOperand(I.getOperand(0)),
                   getAsOperand(I.getOperand(1)),
                   getAsOperand(I.getOperand(2)),
                   getValueSizeInBits(I));
}

VASTValPtr EarlyDatapathBuilder::visitICmpInst(ICmpInst &I) {
  VASTValPtr LHS = getAsOperand(I.getOperand(0)),
             RHS = getAsOperand(I.getOperand(1));

  VASTExpr::Opcode Opcode = I.isSigned() ? VASTExpr::dpSCmp : VASTExpr::dpUCmp;

  unsigned ResultBit = 0;

  switch (I.getPredicate()) {
  case CmpInst::ICMP_NE:  ResultBit = 1; break;
  case CmpInst::ICMP_EQ:  ResultBit = 2; break;

  case CmpInst::ICMP_SLT:
  case CmpInst::ICMP_ULT:
    std::swap(LHS, RHS);
    // Fall though.
  case CmpInst::ICMP_SGE:
  case CmpInst::ICMP_UGE: ResultBit = 3; break;

  case CmpInst::ICMP_SLE:
  case CmpInst::ICMP_ULE:
    std::swap(LHS, RHS);
    // Fall though.
  case CmpInst::ICMP_SGT:
  case CmpInst::ICMP_UGT: ResultBit = 4; break;
  default: llvm_unreachable("Unexpected ICmp predicate!"); break;
  }

  VASTValPtr CmpExpr = buildExpr(Opcode, LHS, RHS, 8);
  // Get the result bit from the CmpExpr.
  return buildBitSliceExpr(CmpExpr, ResultBit + 1, ResultBit);
}

VASTValPtr EarlyDatapathBuilder::visitBinaryOperator(BinaryOperator &I) {
  VASTValPtr Ops[] = { getAsOperand(I.getOperand(0)),
                       getAsOperand(I.getOperand(1))};
  unsigned ResultSize = getValueSizeInBits(I);

  // FIXME: Do we need to care about NSW and NUW?
  switch (I.getOpcode()) {
  case Instruction::Add: return buildAddExpr(Ops, ResultSize);
  case Instruction::Sub: {
    // A - B is equivalent to A + ~(B) + 1
    VASTValPtr SubOps[] = { Ops[0],
                            buildNotExpr(Ops[1]),
                            getOrCreateImmediate(1, 1) };
    return buildAddExpr(SubOps, ResultSize);
  }
  case Instruction::Mul: return buildMulExpr(Ops, ResultSize);

  case Instruction::Shl: return buildExpr(VASTExpr::dpShl, Ops, ResultSize);
  case Instruction::AShr: return buildExpr(VASTExpr::dpSRA, Ops, ResultSize);
  case Instruction::LShr: return buildExpr(VASTExpr::dpSRL, Ops, ResultSize);

  // Div is implemented as submodule.
  case Instruction::SRem:
  case Instruction::URem:
  case Instruction::UDiv:
  case Instruction::SDiv: return VASTValPtr();

  case Instruction::And:  return buildAndExpr(Ops, ResultSize);
  case Instruction::Or:   return buildOrExpr(Ops, ResultSize);
  case Instruction::Xor:  return buildXorExpr(Ops, ResultSize);
  default: llvm_unreachable("Unexpected opcode!"); break;
  }

  return VASTValPtr();
}

VASTValPtr EarlyDatapathBuilder::visitGetElementPtrInst(GetElementPtrInst &I) {
  VASTValPtr Ptr = getAsOperand(I.getOperand(0));
  // FIXME: All the pointer arithmetic are perform under the precision of
  // PtrSize, do we need to perform the arithmetic at the max avilable integer
  // width and truncate the resunt?
  unsigned PtrSize = Ptr->getBitWidth();
  // Note that the pointer operand may be a vector of pointers. Take the scalar
  // element which holds a pointer.
  Type *Ty = I.getOperand(0)->getType()->getScalarType();

  for (GetElementPtrInst::const_op_iterator OI = I.op_begin()+1, E = I.op_end();
       OI != E; ++OI) {
    const Value *Idx = *OI;
    if (StructType *StTy = dyn_cast<StructType>(Ty)) {
      unsigned Field = cast<ConstantInt>(Idx)->getZExtValue();
      if (Field) {
        // N = N + Offset
        uint64_t Offset = TD->getStructLayout(StTy)->getElementOffset(Field);
        Ptr = buildExpr(VASTExpr::dpAdd,
                        Ptr, getOrCreateImmediate(Offset, PtrSize),
                        PtrSize);
      }

      Ty = StTy->getElementType(Field);
    } else {
      Ty = cast<SequentialType>(Ty)->getElementType();

      // If this is a constant subscript, handle it quickly.
      if (const ConstantInt *CI = dyn_cast<ConstantInt>(Idx)) {
        if (CI->isZero()) continue;
        uint64_t Offs =
            TD->getTypeAllocSize(Ty) * cast<ConstantInt>(CI)->getSExtValue();
        
        Ptr = buildExpr(VASTExpr::dpAdd,
                        Ptr, getOrCreateImmediate(Offs, PtrSize),
                        PtrSize);
        continue;
      }

      // N = N + Idx * ElementSize;
      APInt ElementSize = APInt(PtrSize, TD->getTypeAllocSize(Ty));
      VASTValPtr IdxN = getAsOperand(const_cast<Value*>(Idx));

      // If the index is smaller or larger than intptr_t, truncate or extend
      // it.
      IdxN = buildBitSliceExpr(IdxN, PtrSize, 0);

      // If this is a multiply by a power of two, turn it into a shl
      // immediately.  This is a very common case.
      if (ElementSize != 1) {
        if (ElementSize.isPowerOf2()) {
          unsigned Amt = ElementSize.logBase2();
          IdxN = buildShiftExpr(VASTExpr::dpShl, IdxN,
                                getOrCreateImmediate(Amt, PtrSize),
                                PtrSize);
        } else {
          VASTValPtr Scale = getOrCreateImmediate(ElementSize.getSExtValue(),
                                                  PtrSize);
          IdxN = buildExpr(VASTExpr::dpMul, IdxN, Scale, PtrSize);
        }
      }
      
      Ptr = buildExpr(VASTExpr::dpAdd, Ptr, IdxN, PtrSize);
    }
  }

  return Ptr;
}
