//===--- VASTExprBuilder.cpp - Building Verilog AST Expressions -*- C++ -*-===//
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
// This file implement the Verilog AST Expressions building and optimizating
// functions.
//
//===----------------------------------------------------------------------===//

#include "VASTExprBuilder.h"
#include "vtm/Utilities.h"

#define DEBUG_TYPE "vtm-vast-expr-builder"
#include "llvm/Support/Debug.h"

using namespace llvm;

// Inline all operands in the expression whose Opcode is the same as Opc
// recursively;
template<VASTExpr::Opcode Opcode, typename visitor>
void VASTExprBuilder::flattenExpr(VASTValPtr V, visitor F) {
  if (VASTExpr *Expr = dyn_cast<VASTExpr>(V)) {
    typedef const VASTUse *op_iterator;
    if (Expr->getOpcode() == Opcode && shouldExprBeFlatten(Expr)) {
      for (op_iterator I = Expr->op_begin(), E = Expr->op_end(); I != E; ++I)
        flattenExpr<Opcode>(I->getAsInlineOperand(), F);

      return;
    }
  }

  F++ = V;
}

template<VASTExpr::Opcode Opcode, typename iterator, typename visitor>
void VASTExprBuilder::flattenExpr(iterator begin, iterator end, visitor F) {
  while (begin != end)
    flattenExpr<Opcode>(*begin++, F);
}

void VASTExprBuilder::calculateBitCatBitMask(VASTExprPtr Expr,
                                             uint64_t &KnownZeros,
                                             uint64_t &KnownOnes) {
  unsigned CurUB = Expr->getBitWidth();
  // Clear the mask.
  KnownOnes = KnownZeros = UINT64_C(0);

  // Concatenate the bit mask together.
  for (unsigned i = 0; i < Expr->NumOps; ++i) {
    VASTValPtr CurBitSlice = Expr.getOperand(i);
    unsigned CurSize = CurBitSlice->getBitWidth();
    unsigned CurLB = CurUB - CurSize;
    uint64_t CurKnownZeros , CurKnownOnes;
    calculateBitMask(CurBitSlice, CurKnownZeros, CurKnownOnes);
    KnownZeros   |= getBitSlice64(CurKnownZeros, CurSize) << CurLB;
    CurKnownOnes |= getBitSlice64(CurKnownOnes, CurSize) << CurLB;

    CurUB = CurLB;
  }
}

void VASTExprBuilder::calculateBitMask(VASTValPtr V, uint64_t &KnownZeros,
                                       uint64_t &KnownOnes) {
  // Clear the mask.
  KnownOnes = KnownZeros = UINT64_C(0);

  // Most simple case: Immediate.
  if (VASTImmPtr Imm = dyn_cast<VASTImmediate>(V)) {
    KnownOnes = Imm.getUnsignedValue();
    KnownZeros = getBitSlice64(~KnownOnes, Imm->getBitWidth());
    return;
  }

  VASTExprPtr Expr = dyn_cast<VASTExpr>(V);
  if (!Expr) return;

  switch(Expr->getOpcode()) {
  default: return;
  case VASTExpr::dpBitCat:
    calculateBitCatBitMask(Expr, KnownZeros, KnownOnes);
    return;
  }
}

VASTValPtr VASTExprBuilder::trimZeros(VASTValPtr V, unsigned &Offset) {
  if (VASTImmediate *Imm = dyn_cast<VASTImmediate>(V)) {
    uint64_t Val = Imm->getSignedValue();
    if (isAllZeros64(Val, Imm->getBitWidth())) return V;
    
    unsigned TrailingZeros = CountTrailingZeros_64(Val);
    unsigned LeadingZeros = CountLeadingZeros_64(Val);
    Val = getBitSlice64(Val, 64 - LeadingZeros, TrailingZeros);
    if (LeadingZeros) LeadingZeros -= 64 - Imm->getBitWidth();    
    unsigned NewBitWidth = Imm->getBitWidth() - LeadingZeros - TrailingZeros;
    assert(NewBitWidth <= Imm->getBitWidth() && "Bad bitwidth!");
    Offset = TrailingZeros;
    return getOrCreateImmediate(Val, NewBitWidth);
  }
  
  VASTExpr *Expr = dyn_cast<VASTExpr>(V);
  if (!Expr || Expr->getOpcode() != VASTExpr::dpBitCat) return V;

  // Too complex to handle.
  if (Expr->NumOps != 2) return V;

  VASTValPtr Hi = Expr->getOperand(0), Lo = Expr->getOperand(1);

  if (isAllZeros(Hi)) {
      // The higher part are zeros, offset is zero.
      Offset = 0;
      return Lo;
  }

  if (isAllZeros(Lo)) {
    // The higher part are zeros, offset is zero.
    Offset = Lo->getBitWidth();
    return Hi;
  }

  return V;
}

VASTValPtr VASTExprBuilder::buildNotExpr(VASTValPtr U) {
  U = U.invert();

  if (VASTImmPtr ImmPtr = dyn_cast<VASTImmediate>(U))
    return Context.getOrCreateImmediate(ImmPtr.getUnsignedValue(),
                                        ImmPtr->getBitWidth());

  if (U.isInverted()) {
    if (VASTExpr *Expr = dyn_cast<VASTExpr>(U.get())) {
      if (Expr->getOpcode() == VASTExpr::dpBitCat) {
        typedef VASTExpr::op_iterator it;
        SmallVector<VASTValPtr, 4> Ops;
        for (it I = Expr->op_begin(), E = Expr->op_end(); I != E; ++I)
          Ops.push_back(buildNotExpr(*I));

        return buildBitCatExpr(Ops, Expr->getBitWidth());
      }
    }
  }

  return U;
}

VASTValPtr VASTExprBuilder::foldBitSliceExpr(VASTValPtr U, uint8_t UB,
                                             uint8_t LB) {
  unsigned OperandSize = U->getBitWidth();
  // Not a sub bitslice.
  if (UB == OperandSize && LB == 0) return U;

  if (VASTImmPtr Imm = dyn_cast<VASTImmediate>(U)) {
    uint64_t imm = getBitSlice64(Imm.getUnsignedValue(), UB, LB);
    return Context.getOrCreateImmediate(imm, UB - LB);
  }

  VASTExprPtr Expr = dyn_cast<VASTExpr>(U);

  if (Expr == 0) return VASTValPtr(0);

  if (Expr->getOpcode() == VASTExpr::dpAssign){
    unsigned Offset = Expr->LB;
    UB += Offset;
    LB += Offset;
    return buildBitSliceExpr(Expr.getOperand(0), UB, LB);
  }

  if (Expr->getOpcode() == VASTExpr::dpBitCat) {
    // Collect the bitslices which fall into (UB, LB]
    SmallVector<VASTValPtr, 8> Ops;
    unsigned CurUB = Expr->getBitWidth(), CurLB = 0;
    unsigned LeadingBitsToLeft = 0, TailingBitsToTrim = 0;
    for (unsigned i = 0; i < Expr->NumOps; ++i) {
      VASTValPtr CurBitSlice = Expr.getOperand(i);
      CurLB = CurUB - CurBitSlice->getBitWidth();
      // Not fall into (UB, LB] yet.
      if (CurLB >= UB) {
        CurUB = CurLB;
        continue;
      }
      // The entire range is visited.
      if (CurUB <= LB) break;
      // Now we have CurLB < UB and CurUB > LB.
      // Compute LeadingBitsToLeft if UB fall into [CurUB, CurLB), which imply
      // CurUB >= UB >= CurLB.
      if (CurUB >= UB) LeadingBitsToLeft = UB - CurLB;
      // Compute TailingBitsToTrim if LB fall into (CurUB, CurLB], which imply
      // CurUB >= LB >= CurLB.
      if (LB >= CurLB) TailingBitsToTrim = LB - CurLB;

      Ops.push_back(CurBitSlice);
      CurUB = CurLB;
    }

    // Trival case: Only 1 bitslice in range.
    if (Ops.size() == 1)
      return buildBitSliceExpr(Ops.back(), LeadingBitsToLeft, TailingBitsToTrim);

    Ops.front() = buildBitSliceExpr(Ops.front(), LeadingBitsToLeft, 0);
    Ops.back() = buildBitSliceExpr(Ops.back(), Ops.back()->getBitWidth(),
                                   TailingBitsToTrim);

    return buildBitCatExpr(Ops, UB - LB);
  }

  return VASTValPtr(0);
}

VASTValPtr VASTExprBuilder::buildBitCatExpr(ArrayRef<VASTValPtr> Ops,
                                            unsigned BitWidth) {
  SmallVector<VASTValPtr, 8> NewOps;
  flattenExpr<VASTExpr::dpBitCat>(Ops.begin(), Ops.end(),
                                  std::back_inserter(NewOps));

  VASTImmPtr LastImm = dyn_cast<VASTImmediate>(NewOps[0]);
  unsigned ActualOpPos = 1;

  // Merge the constant sequence.
  for (unsigned i = 1, e = NewOps.size(); i < e; ++i) {
    VASTImmPtr CurImm = dyn_cast<VASTImmediate>(NewOps[i]);

    if (!CurImm) {
      LastImm = 0;
      NewOps[ActualOpPos++] = NewOps[i]; //push_back.
      continue;
    }

    if (LastImm) {
      // Merge the constants.
      uint64_t HiVal = LastImm.getUnsignedValue(),
               LoVal = CurImm.getUnsignedValue();
      unsigned HiSizeInBits = LastImm->getBitWidth(),
               LoSizeInBits = CurImm->getBitWidth();
      unsigned SizeInBits = LoSizeInBits + HiSizeInBits;
      assert(SizeInBits <= 64 && "Constant too large!");
      uint64_t Val = (LoVal) | (HiVal << LoSizeInBits);
      LastImm = Context.getOrCreateImmediate(Val, SizeInBits);
      NewOps[ActualOpPos - 1] = LastImm; // Modify back.
    } else {
      LastImm = CurImm;
      NewOps[ActualOpPos++] = NewOps[i]; //push_back.
    }
  }

  NewOps.resize(ActualOpPos);
  if (NewOps.size() == 1) return NewOps.back();

#ifndef NDEBUG
  unsigned TotalBits = 0;
  for (unsigned i = 0, e = NewOps.size(); i < e; ++i)
    TotalBits += NewOps[i]->getBitWidth();
  if (TotalBits != BitWidth) {
    dbgs() << "Bad bitcat operands: \n";
    for (unsigned i = 0, e = NewOps.size(); i < e; ++i)
      NewOps[i]->dump();
    llvm_unreachable("Bitwidth not match!");
  }
#endif

  return Context.createExpr(VASTExpr::dpBitCat, NewOps, BitWidth, 0);
}

VASTValPtr VASTExprBuilder::buildBitSliceExpr(VASTValPtr U, uint8_t UB,
                                              uint8_t LB) {
  // Try to fold the expression.
  if (VASTValPtr P = foldBitSliceExpr(U, UB, LB)) return P;

  assert(UB <= U->getBitWidth() && UB > LB && "Bad bit range!");

  // Name the expression if necessary.
  U = Context.nameExpr(U);

  VASTValPtr Ops[] = { U };
  return Context.createExpr(VASTExpr::dpAssign, Ops, UB, LB);
}

VASTValPtr VASTExprBuilder::buildReduction(VASTExpr::Opcode Opc,VASTValPtr Op) {
  if (VASTImmPtr Imm = dyn_cast<VASTImmediate>(Op)) {
    uint64_t Val = Imm.getUnsignedValue();
    switch (Opc) {
    case VASTExpr::dpRAnd:
      // Only reduce to 1 if all bits are 1.
      if (isAllOnes64(Val, Imm->getBitWidth()))
        return getBoolImmediate(true);
      else
        return getBoolImmediate(false);
    case VASTExpr::dpRXor:
      // Only reduce to 1 if there are odd 1s.
      if (CountPopulation_64(Val) & 0x1)
        return getBoolImmediate(true);
      else
        return getBoolImmediate(false);
      break; // FIXME: Who knows how to evaluate this?
    default:  llvm_unreachable("Unexpected Reduction Node!");
    }
  }

  // Try to fold the expression according to the bit mask.
  uint64_t KnownZeros, KnownOnes;
  calculateBitMask(Op, KnownZeros, KnownOnes);

  if (KnownZeros && Opc == VASTExpr::dpRAnd) return getBoolImmediate(false);

  // Promote the reduction to the operands.
  if (VASTExpr *Expr = dyn_cast<VASTExpr>(Op)) {
    switch (Expr->getOpcode()) {
    default: break;
    case VASTExpr::dpBitCat: {
      SmallVector<VASTValPtr, 8> Ops;
      typedef VASTExpr::op_iterator it;
      for (it I = Expr->op_begin(), E = Expr->op_end(); I != E; ++I)
        Ops.push_back(buildReduction(Opc, *I));

      switch (Opc) {
      case VASTExpr::dpRAnd: return buildAndExpr(Ops, 1);
      case VASTExpr::dpRXor: return buildXorExpr(Ops, 1);
      default:  llvm_unreachable("Unexpected Reduction Node!");
      }
    }
    }
  }

  return Context.createExpr(Opc, Op, 1, 0);
}

static bool VASTValPtr_less(const VASTValPtr LHS, const VASTValPtr RHS) {
  if (LHS->getASTType() < RHS->getASTType()) return true;
  else if (LHS->getASTType() > RHS->getASTType()) return false;

  return LHS < RHS;
}

VASTValPtr
VASTExprBuilder::getOrCreateCommutativeExpr(VASTExpr::Opcode Opc,
                                            SmallVectorImpl<VASTValPtr> &Ops,
                                             unsigned BitWidth) {
  std::sort(Ops.begin(), Ops.end(), VASTValPtr_less);
  return Context.createExpr(Opc, Ops, BitWidth, 0);
}

namespace llvm {
template<>
struct VASTExprOpInfo<VASTExpr::dpAnd> {
  unsigned OperandWidth;
  uint64_t KnownZeros, KnownOnes;

  VASTExprOpInfo(unsigned OperandWidth)
    : OperandWidth(OperandWidth), KnownZeros(UINT64_C(0)),
      KnownOnes(~UINT64_C(0)) /*Assume all bits are ones*/{}

  VASTValPtr analyzeOperand(VASTValPtr V) {
    assert(OperandWidth == V->getBitWidth() && "Bitwidth not match!");

    if (VASTImmPtr Imm = dyn_cast<VASTImmediate>(V)) {
      // The bit is known one only if the bit of all operand are one.
      KnownOnes &= Imm.getSignedValue();
      // The bit is known zero if the bit of any operand are zero.
      KnownZeros |= ~Imm.getSignedValue();
      return 0;
    }

    uint64_t OpKnownZeros, OpKnownOnes;
    VASTExprBuilder::calculateBitMask(V, OpKnownZeros, OpKnownOnes);
    KnownOnes &= OpKnownOnes;
    KnownZeros |=OpKnownZeros;

    // Do nothing by default.
    return V;
  }

  bool isAllZeros() const { return isAllOnes64(KnownZeros, OperandWidth); }
  bool hasAnyZero() const  { return getBitSlice64(KnownZeros, OperandWidth); }
  // For the and expression, only zero is known.
  uint64_t getImmVal() const { return ~KnownZeros; }

  bool getZeroMaskSplitPoints(unsigned &HiPt, unsigned &LoPt) const {
    HiPt = OperandWidth;
    LoPt = 0;

    if (!KnownZeros) return false;

    if (isShiftedMask_64(KnownZeros) || isMask_64(KnownZeros)) {
      unsigned NumZeros = CountPopulation_64(KnownZeros);
      if (NumZeros < OperandWidth / 2) return false;

      LoPt = CountTrailingZeros_64(KnownZeros);
      HiPt = std::min(OperandWidth, 64 - CountLeadingZeros_64(KnownZeros));
      return true;
    }

    unsigned NotKnownZeros = ~KnownZeros;
    if (isShiftedMask_64(NotKnownZeros) || isMask_64(NotKnownZeros)) {
      unsigned NumZeros = CountPopulation_64(KnownZeros);
      if (NumZeros < OperandWidth / 2) return false;

      LoPt = CountTrailingZeros_64(NotKnownZeros);
      HiPt = std::min(OperandWidth, 64 - CountLeadingZeros_64(NotKnownZeros));
      return true;
    }

    return false;
  }
};
}

VASTValPtr VASTExprBuilder::buildAndExpr(ArrayRef<VASTValPtr> Ops,
                                         unsigned BitWidth) {
  SmallVector<VASTValPtr, 8> NewOps;
  typedef const VASTUse *op_iterator;
  VASTExprOpInfo<VASTExpr::dpAnd> OpInfo(BitWidth);
  flattenExpr<VASTExpr::dpAnd>(Ops.begin(), Ops.end(),
                               op_filler<VASTExpr::dpAnd>(NewOps, OpInfo));

  // Check the immediate mask.
  if (OpInfo.isAllZeros())
    return getOrCreateImmediate(UINT64_C(0), BitWidth);

  if (OpInfo.hasAnyZero()) {
    NewOps.push_back(Context.getOrCreateImmediate(OpInfo.getImmVal(), BitWidth));

    // Split the word according to known zeros.
    unsigned HiPt, LoPt;
    if (OpInfo.getZeroMaskSplitPoints(HiPt, LoPt)) {
      assert(BitWidth >= HiPt && HiPt > LoPt && "Bad split point!");
      SmallVector<VASTValPtr, 4> Ops;

      if (HiPt != BitWidth)
        Ops.push_back(buildExprByOpBitSlice(VASTExpr::dpAnd, NewOps, BitWidth,
                                            HiPt));

      Ops.push_back(buildExprByOpBitSlice(VASTExpr::dpAnd, NewOps, HiPt, LoPt));

      if (LoPt != 0)
        Ops.push_back(buildExprByOpBitSlice(VASTExpr::dpAnd, NewOps, LoPt, 0));

      return buildBitCatExpr(Ops, BitWidth);
    }
  }

  if (NewOps.empty())
    return Context.getOrCreateImmediate(getBitSlice64(~0ull, BitWidth),
                                        BitWidth);

  std::sort(NewOps.begin(), NewOps.end(), VASTValPtr_less);
  typedef SmallVectorImpl<VASTValPtr>::iterator it;
  VASTValPtr LastVal;
  unsigned ActualPos = 0;
  for (unsigned i = 0, e = NewOps.size(); i != e; ++i) {
    VASTValPtr CurVal = NewOps[i];
    if (CurVal == LastVal) {
      // A & A = A
      continue;
    } else if (CurVal.invert() == LastVal)
      // A & ~A => 0
      return getBoolImmediate(false);

    NewOps[ActualPos++] = CurVal;
    LastVal = CurVal;
  }
  // If there is only 1 operand left, simply return the operand.
  if (ActualPos == 1) return LastVal;

  // Resize the operand vector so it only contains valid operands.
  NewOps.resize(ActualPos);

  return Context.createExpr(VASTExpr::dpAnd, NewOps, BitWidth, 0);
}

VASTValPtr VASTExprBuilder::buildExpr(VASTExpr::Opcode Opc, VASTValPtr LHS,
                                      VASTValPtr RHS, unsigned BitWidth) {
  VASTValPtr Ops[] = { LHS, RHS };
  return buildExpr(Opc, Ops, BitWidth);
}

VASTValPtr VASTExprBuilder::buildExpr(VASTExpr::Opcode Opc, VASTValPtr Op0,
                                       VASTValPtr Op1, VASTValPtr Op2,
                                       unsigned BitWidth) {
  VASTValPtr Ops[] = { Op0, Op1, Op2 };
  return buildExpr(Opc, Ops, BitWidth);
}

VASTValPtr VASTExprBuilder::buildExpr(VASTExpr::Opcode Opc,
                                      ArrayRef<VASTValPtr> Ops,
                                      unsigned BitWidth) {
  switch (Opc) {
  default: break;
  case VASTExpr::dpAdd:  return buildAddExpr(Ops, BitWidth);
  case VASTExpr::dpMul:  return buildMulExpr(Ops, BitWidth);
  case VASTExpr::dpAnd:  return buildAndExpr(Ops, BitWidth);
  case VASTExpr::dpBitCat: return buildBitCatExpr(Ops, BitWidth);

  case VASTExpr::dpRAnd:
  case VASTExpr::dpRXor:
    assert(Ops.size() == 1 && "Unexpected more than 1 operands for reduction!");
    assert(BitWidth == 1 && "Bitwidth of reduction should be 1!");
    return buildReduction(Opc, Ops[0]);
  }

  return Context.createExpr(Opc, Ops, BitWidth, 0);
}

VASTValPtr VASTExprBuilder::buildExpr(VASTExpr::Opcode Opc, VASTValPtr Op,
                                      unsigned BitWidth) {
  switch (Opc) {
  default: break;
  case VASTExpr::dpRAnd:
  case VASTExpr::dpRXor:
    assert(BitWidth == 1 && "Bitwidth of reduction should be 1!");
    return buildReduction(Opc, Op);
  }

  VASTValPtr Ops[] = { Op };
  return Context.createExpr(Opc, Ops, BitWidth, 0);
}

VASTValPtr VASTExprBuilder::buildMulExpr(ArrayRef<VASTValPtr> Ops,
                                         unsigned BitWidth) {
  SmallVector<VASTValPtr, 8> NewOps;
  VASTExprOpInfo<VASTExpr::dpMul> OpInfo;

  flattenExpr<VASTExpr::dpMul>(Ops.begin(), Ops.end(),
                               op_filler<VASTExpr::dpMul>(NewOps, OpInfo));

  return getOrCreateCommutativeExpr(VASTExpr::dpMul, NewOps, BitWidth);
}

VASTValPtr VASTExprBuilder::buildAddExpr(ArrayRef<VASTValPtr> Ops,
                                         unsigned BitWidth) {
  SmallVector<VASTValPtr, 8> NewOps;
  uint64_t ImmVal = 0;
  unsigned MaxImmWidth = 0;
  VASTValPtr Carry = 0;
  VASTExprOpInfo<VASTExpr::dpAdd> OpInfo;

  for (unsigned i = 0; i < Ops.size(); ++i) {
    VASTValPtr V = Ops[i];
    // Discard the leading zeros of the operand of addition;
    V = trimLeadingZeros(V);

    // Fold the immediate.
    if (VASTImmPtr Imm = dyn_cast<VASTImmediate>(V)) {
      ImmVal += Imm->getUnsignedValue();
      MaxImmWidth = std::max(MaxImmWidth, Imm->getBitWidth());
      continue;
    }

    if (V->getBitWidth() == 1) {
      if (Carry) NewOps.push_back(Carry);

      Carry = V;
      continue;
    }

    flattenExpr<VASTExpr::dpAdd>(V, op_filler<VASTExpr::dpAdd>(NewOps, OpInfo));
  }

  // Add the immediate value back to the operand list.
  if (ImmVal)
    NewOps.push_back(Context.getOrCreateImmediate(ImmVal, MaxImmWidth));

  // If the addition contains only 2 operand, check if we can inline a operand
  // of this addition to make use of the carry bit.
  if (NewOps.size() == 2 && !Carry) {
    unsigned ExprIdx = 0;
    VASTExpr *Expr = Context.getAddExprToFlatten(NewOps[ExprIdx]);
    if (Expr == 0) Expr = Context.getAddExprToFlatten(NewOps[++ExprIdx]);

    // If we can find such expression, flatten the expression tree.
    if (Expr) {
      // Replace the expression by the no-carry operand
      NewOps[ExprIdx] = Expr->getOperand(0);
      // And add the carry from the expression.
      Carry = Expr->getOperand(1);
      assert(Carry->getBitWidth() == 1 && "Carry is not 1 bit!");
    }
  }

  // Sort the operands excluding carry bit, we want to place the carry bit at
  // last.
  std::sort(NewOps.begin(), NewOps.end(), VASTValPtr_less);

  // Add the carry bit back to the operand list.
  if (Carry) NewOps.push_back(Carry);

  // All operands are zero?
  if (NewOps.empty()) return Context.getOrCreateImmediate(UINT64_C(0),BitWidth);

  if (NewOps.size() == 1) {
    VASTValPtr V = NewOps.back();
    assert(BitWidth >= V->getBitWidth() && "Bad bitwidth!");
    unsigned ZeroBits = BitWidth - V->getBitWidth();

    if (ZeroBits == 0) return V;

    // Pad the MSB by zeros.
    VASTValPtr Ops[] = {Context.getOrCreateImmediate(UINT64_C(0), ZeroBits), V};
    return buildBitCatExpr(Ops, BitWidth);
  }

  return Context.createExpr(VASTExpr::dpAdd, NewOps, BitWidth, 0);
}

VASTValPtr VASTExprBuilder::buildOrExpr(ArrayRef<VASTValPtr> Ops,
                                        unsigned BitWidth) {
  if (Ops.size() == 1) return Ops[0];

  assert (Ops.size() > 1 && "There should be more than one operand!!");

  SmallVector<VASTValPtr, 4> NotExprs;
  // Build the operands of Or operation into not Expr.
  for (unsigned i = 0; i < Ops.size(); ++i) {
    VASTValPtr V = buildNotExpr(Ops[i]);
    NotExprs.push_back(V);
  }

  // Build Or operation with the And Inverter Graph (AIG).
  return buildNotExpr(buildAndExpr(NotExprs, BitWidth));
}

VASTValPtr VASTExprBuilder::buildXorExpr(ArrayRef<VASTValPtr> Ops,
                                         unsigned BitWidth) {
  assert (Ops.size() == 2 && "There should be more than one operand!!");

  // Build the Xor Expr with the And Inverter Graph (AIG).
  return buildExpr(VASTExpr::dpAnd, buildOrExpr(Ops, BitWidth),
                   buildNotExpr(buildAndExpr(Ops, BitWidth)),
                   BitWidth);
}

VASTWire *VASTModule::buildAssignCnd(VASTSlot *Slot,
                                     SmallVectorImpl<VASTValPtr> &Cnds,
                                     VASTExprBuilder &Builder,
                                     bool AddSlotActive) {
  // We only assign the Src to Dst when the given slot is active.
  if (AddSlotActive) Cnds.push_back(Slot->getActive()->getAsInlineOperand(false));
  VASTValPtr AssignAtSlot = Builder.buildExpr(VASTExpr::dpAnd, Cnds, 1);
  VASTWire *Wire = Allocator.Allocate<VASTWire>();
  new (Wire) VASTWire(0, AssignAtSlot->getBitWidth(), "");
  assign(Wire, AssignAtSlot, VASTWire::AssignCond)->setSlot(Slot->SlotNum);
  // Recover the condition vector.
  if (AddSlotActive) Cnds.pop_back();

  return Wire;
}

void VASTModule::addAssignment(VASTRegister *Dst, VASTValPtr Src, VASTSlot *Slot,
                               SmallVectorImpl<VASTValPtr> &Cnds,
                               VASTExprBuilder &Builder, bool AddSlotActive) {
  if (Src) {
    VASTWire *Cnd = buildAssignCnd(Slot, Cnds, Builder, AddSlotActive);
    Dst->addAssignment(new (Allocator.Allocate<VASTUse>()) VASTUse(Src, 0), Cnd);
  }
}
