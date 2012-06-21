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
  if (VASTImmPtr Imm = dyn_cast<VASTImmPtr>(V)) {
    KnownOnes = Imm.getUnsignedValue();
    KnownZeros = getBitSlice64(~KnownOnes, Imm->getBitWidth());
    return;
  }

  VASTExprPtr Expr = dyn_cast<VASTExprPtr>(V);
  if (!Expr) {
    VASTValPtr NameStripped = Context.stripName(V);
    if (NameStripped != V)
      // Compute the bitmask of underlying expression if name is actually
      // stripped.
      calculateBitMask(NameStripped, KnownZeros, KnownOnes);
    
    return;
  }

  switch(Expr->getOpcode()) {
  default: return;
  case VASTExpr::dpBitCat:
    calculateBitCatBitMask(Expr, KnownZeros, KnownOnes);
    return;
  case VASTExpr::dpAssign:
    calculateBitMask(Expr.getOperand(0), KnownZeros, KnownOnes);
    // Adjust the bitmask by LB.
    KnownOnes = getBitSlice64(KnownOnes >> Expr->LB, Expr->getBitWidth());
    KnownZeros = getBitSlice64(KnownZeros >> Expr->LB, Expr->getBitWidth());
    return;
  case VASTExpr::dpSel: {
    VASTValPtr OpTrue = Expr.getOperand(1), OpFalse = Expr.getOperand(2);;
    calculateBitMask(OpTrue, KnownZeros, KnownOnes);
    uint64_t OpFalseKnownOnes, OpFalseKnownZeros;
    calculateBitMask(OpFalse, OpFalseKnownZeros, OpFalseKnownOnes);
    KnownOnes &= OpFalseKnownOnes;
    KnownZeros &= OpFalseKnownZeros;
    return;
  }
  }
}

VASTValPtr VASTExprBuilder::buildNotExpr(VASTValPtr U) {
  U = U.invert();


  // Try to promote the invert flag
  if (U.isInverted()) {
    if (VASTImmPtr ImmPtr = dyn_cast<VASTImmPtr>(U))
        return getOrCreateImmediate(ImmPtr.getUnsignedValue(),
                                    ImmPtr->getBitWidth());

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

  if (VASTImmPtr Imm = dyn_cast<VASTImmPtr>(U)) {
    uint64_t imm = getBitSlice64(Imm.getUnsignedValue(), UB, LB);
    return Context.getOrCreateImmediate(imm, UB - LB);
  }

  VASTExprPtr Expr = dyn_cast<VASTExprPtr>(U);

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

  if (Expr->getOpcode() == VASTExpr::dpBitRepeat) {
    VASTValPtr Pattern = Expr.getOperand(0);
    // Simply repeat the pattern by the correct number.
    if (Pattern->getBitWidth() == 1) {
      VASTValPtr Ops[] = { Pattern, getOrCreateImmediate(UB - LB, 64) };
      return buildExpr(VASTExpr::dpBitRepeat, Ops, UB - LB);
    }
    // TODO: Build the correct pattern.
  }

  return VASTValPtr(0);
}

static VASTExprPtr getAsBitSliceExpr(VASTValPtr V) {
  VASTExprPtr Expr = dyn_cast<VASTExprPtr>(V);
  if (!Expr || !Expr->isSubBitSlice()) return 0;

  return Expr;
}

VASTValPtr VASTExprBuilder::buildBitCatExpr(ArrayRef<VASTValPtr> Ops,
                                            unsigned BitWidth) {
  SmallVector<VASTValPtr, 8> NewOps;
  flattenExpr<VASTExpr::dpBitCat>(Ops.begin(), Ops.end(),
                                  std::back_inserter(NewOps));

  VASTImmPtr LastImm = dyn_cast<VASTImmPtr>(NewOps[0]);
  VASTExprPtr LastBitSlice = getAsBitSliceExpr(NewOps[0]);

  unsigned ActualOpPos = 1;

  // Merge the constant sequence.
  for (unsigned i = 1, e = NewOps.size(); i < e; ++i) {
    VASTValPtr V = NewOps[i];
    if (VASTImmPtr CurImm = dyn_cast<VASTImmPtr>(V)) {
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
        continue;
      } else {
        LastImm = CurImm;
        NewOps[ActualOpPos++] = V; //push_back.
        continue;
      }
    } else // Reset LastImm, since the current value is not immediate.
      LastImm = 0;

    if (VASTExprPtr CurBitSlice = getAsBitSliceExpr(V)) {
      VASTValPtr CurBitSliceParent = CurBitSlice.getOperand(0);
      if (LastBitSlice && CurBitSliceParent == LastBitSlice.getOperand(0)
          && LastBitSlice->LB == CurBitSlice->UB) {
        VASTValPtr MergedBitSlice
          = buildBitSliceExpr(CurBitSliceParent, LastBitSlice->UB,
                              CurBitSlice->LB);
        NewOps[ActualOpPos - 1] = MergedBitSlice; // Modify back.
        LastBitSlice = getAsBitSliceExpr(MergedBitSlice);
        continue;
      } else {
        LastBitSlice = CurBitSlice;
        NewOps[ActualOpPos++] = V; //push_back.
        continue;
      }
    } else
      LastBitSlice = 0;

    NewOps[ActualOpPos++] = V; //push_back.
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
  assert(UB <= U->getBitWidth() && UB > LB && "Bad bit range!");
  // Try to fold the expression.
  if (VASTValPtr P = foldBitSliceExpr(U, UB, LB)) return P;

  // Name the expression if necessary.
  U = Context.nameExpr(U);

  VASTValPtr Ops[] = { U };
  return Context.createExpr(VASTExpr::dpAssign, Ops, UB, LB);
}

VASTValPtr VASTExprBuilder::buildReduction(VASTExpr::Opcode Opc,VASTValPtr Op) {
  if (VASTImmPtr Imm = dyn_cast<VASTImmPtr>(Op)) {
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

VASTValPtr VASTExprBuilder::buildSelExpr(VASTValPtr Cnd, VASTValPtr TrueV,
                                         VASTValPtr FalseV, unsigned BitWidth) {
  assert(Cnd->getBitWidth() == 1 && "Bad condition width!");
  assert(TrueV->getBitWidth() == FalseV->getBitWidth()
         && TrueV->getBitWidth() == BitWidth && "Bad bitwidth!");

  if (VASTImmPtr Imm = dyn_cast<VASTImmPtr>(Cnd))
    return Imm.getUnsignedValue() ? TrueV : FalseV;

  VASTValPtr Ops[] = { Cnd, TrueV, FalseV };
  return Context.createExpr(VASTExpr::dpSel, Ops, BitWidth, 0);
}

namespace llvm {
template<>
struct VASTExprOpInfo<VASTExpr::dpAnd> {
  VASTExprBuilder &Builder;
  unsigned OperandWidth;
  uint64_t KnownZeros, KnownOnes;

  VASTExprOpInfo(VASTExprBuilder &Builder, unsigned OperandWidth)
    : Builder(Builder), OperandWidth(OperandWidth), KnownZeros(UINT64_C(0)),
      KnownOnes(~UINT64_C(0)) /*Assume all bits are ones*/{}

  VASTValPtr analyzeOperand(VASTValPtr V) {
    assert(OperandWidth == V->getBitWidth() && "Bitwidth not match!");

    if (VASTImmPtr Imm = dyn_cast<VASTImmPtr>(V)) {
      // The bit is known one only if the bit of all operand are one.
      KnownOnes &= Imm.getSignedValue();
      // The bit is known zero if the bit of any operand are zero.
      KnownZeros |= ~Imm.getSignedValue();
      return 0;
    }

    uint64_t OpKnownZeros, OpKnownOnes;
    Builder.calculateBitMask(V, OpKnownZeros, OpKnownOnes);
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
  VASTExprOpInfo<VASTExpr::dpAnd> OpInfo(*this, BitWidth);
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
  if (Opc == VASTExpr::dpSel) return buildSelExpr(Op0, Op1, Op2, BitWidth);

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
  case VASTExpr::dpSel:  return buildSelExpr(Ops[0], Ops[1], Ops[2], BitWidth);

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

namespace llvm {
struct AddMultOpInfoBase {
  VASTExprBuilder &Builder;
  const unsigned ResultSize;
  uint64_t ImmVal;
  unsigned ImmSize;
  unsigned MaxTailingZeros;
  VASTValPtr OpWithTailingZeros;

  AddMultOpInfoBase(VASTExprBuilder &Builder, unsigned ResultSize)
    : Builder(Builder), ResultSize(ResultSize), ImmVal(0), ImmSize(0),
      MaxTailingZeros(0), OpWithTailingZeros(0) {}

  VASTValPtr analyzeBitMask(VASTValPtr V,  unsigned &CurTailingZeros) {
    uint64_t KnownZeros, KnownOnes;
    unsigned OperandSize = V->getBitWidth();
    // Trim the unused bits according to the result's size
    if (OperandSize > ResultSize)
      V = Builder.buildBitSliceExpr(V, ResultSize, 0);

    CurTailingZeros = 0;

    Builder.calculateBitMask(V, KnownZeros, KnownOnes);
    // Any known zeros?
    if (KnownZeros) {
      // Try to trim the leading zeros.
      KnownZeros = SignExtend64(KnownZeros, OperandSize);

      // Ignore the zero operand for the addition.
      if (KnownZeros == ~UINT64_C(0)) return 0;

      // Any known leading zeros?
      if (KnownZeros >> 63) {
        unsigned NoZerosUB = 64 - CountLeadingOnes_64(KnownZeros);

        V = Builder.buildBitSliceExpr(V, NoZerosUB, 0);
      }

      CurTailingZeros = CountTrailingOnes_64(KnownZeros);
    }

    return V;
  }

  void updateTailingZeros(VASTValPtr V, unsigned CurTailingZeros) {
    // Remember the operand with tailing zeros.
    if (MaxTailingZeros < CurTailingZeros) {
      MaxTailingZeros = CurTailingZeros;
      OpWithTailingZeros = V;
    }
  }
};

template<>
struct VASTExprOpInfo<VASTExpr::dpAdd> : public AddMultOpInfoBase {
  VASTValPtr Carry;

  VASTExprOpInfo(VASTExprBuilder &Builder, unsigned ResultSize)
    : AddMultOpInfoBase(Builder, ResultSize), Carry(0) {}

  VASTValPtr analyzeOperand(VASTValPtr V) {
    unsigned CurTailingZeros;

    V = analyzeBitMask(V, CurTailingZeros);
    if (!V) return 0;

    // Fold the immediate.
    if (VASTImmPtr Imm = dyn_cast<VASTImmPtr>(V)) {
      ImmSize = std::max(ImmSize, Imm->getBitWidth());
      // Each addition will produce 1 extra bit (carry bit).
      if (ImmVal) ImmSize = std::min(ImmSize + 1, ResultSize);

      ImmVal += Imm->getUnsignedValue();
      return 0;
    }

    // Cache the carry and make sure we place the carry at the last of the
    // operand list.
    if (V->getBitWidth() == 1 && !Carry) {
      Carry = V;
      return 0;
    }

    updateTailingZeros(V, CurTailingZeros);

    return V;
  }

  VASTValPtr flushImmOperand() {
    if (ImmVal) {
      VASTImmPtr Imm = Builder.getOrCreateImmediate(ImmVal, ImmSize);
      uint64_t KnownZeros = ~Imm.getUnsignedValue();
      unsigned CurTailingZeros = CountTrailingOnes_64(KnownZeros);

      updateTailingZeros(Imm, CurTailingZeros);

      // Reset the Immediate context.
      ImmSize = 0;
      ImmVal = 0;

      // Try to treat 1 bit immediate as carry bit.
      if (Imm->getBitWidth() == 1 && !Carry) {
        Carry = Imm;
        return 0;
      }

      return Imm;
    }

    return 0;
  }
};

template<>
struct VASTExprOpInfo<VASTExpr::dpMul> : public AddMultOpInfoBase {
  bool ZeroDetected;

  VASTExprOpInfo(VASTExprBuilder &Builder, unsigned ResultSize)
    : AddMultOpInfoBase(Builder, ResultSize), ZeroDetected(false) {}


  VASTValPtr analyzeOperand(VASTValPtr V) {
    // Zero detected, no need to analyze.
    if (ZeroDetected) return 0;

    unsigned CurTailingZeros;
    V = analyzeBitMask(V, CurTailingZeros);

    if (!V) {
      ZeroDetected = true;
      return 0;
    }

    if (VASTImmPtr Imm = dyn_cast<VASTImmPtr>(V)) {
      // Ignore multiply by 1.
      if (Imm.getUnsignedValue() == 1) return 0;
    }

    updateTailingZeros(V, CurTailingZeros);
    return V;
  }
};
}

VASTValPtr VASTExprBuilder::padHeadOrTail(VASTValPtr V, unsigned BitWidth,
                                          bool ByOnes, bool PadTail) {
  assert(BitWidth >= V->getBitWidth() && "Bad bitwidth!");
  unsigned ZeroBits = BitWidth - V->getBitWidth();

  if (ZeroBits == 0) return V;

  VASTValPtr Pader =
    Context.getOrCreateImmediate(ByOnes ? ~UINT64_C(0) : UINT64_C(0), ZeroBits);

  VASTValPtr Hi = PadTail ? V : Pader, Lo = PadTail ? Pader : V;

  VASTValPtr Ops[] = { Hi, Lo};
  return buildBitCatExpr(Ops, BitWidth);
}

VASTValPtr VASTExprBuilder::buildMulExpr(ArrayRef<VASTValPtr> Ops,
                                         unsigned BitWidth) {
  SmallVector<VASTValPtr, 8> NewOps;
  VASTExprOpInfo<VASTExpr::dpMul> OpInfo(*this, BitWidth);

  flattenExpr<VASTExpr::dpMul>(Ops.begin(), Ops.end(),
                                op_filler<VASTExpr::dpMul>(NewOps, OpInfo));

  if (OpInfo.ZeroDetected) return getOrCreateImmediate(UINT64_C(0), BitWidth);

  // Reduce the size of multiply according to tailing zero.
  if (NewOps.size() == 2 && OpInfo.OpWithTailingZeros) {
    VASTValPtr NotEndWithZeros = NewOps[0],
               EndWithZeros = OpInfo.OpWithTailingZeros;
    if (NotEndWithZeros == EndWithZeros)
      NotEndWithZeros = NewOps[1];

    unsigned TailingZeros = OpInfo.MaxTailingZeros;
    VASTValPtr TrimedOp =
      buildBitSliceExpr(EndWithZeros, EndWithZeros->getBitWidth(), TailingZeros);

    // Build the new multiply without known zeros.
    unsigned NewMultSize = BitWidth - TailingZeros;
    VASTValPtr NewMultOps[] = { NotEndWithZeros, TrimedOp };
    VASTValPtr NewMult = Context.nameExpr(buildMulExpr(NewMultOps, NewMultSize));
    return padLowerBits(NewMult, BitWidth, false);
  }

  return getOrCreateCommutativeExpr(VASTExpr::dpMul, NewOps, BitWidth);
}

VASTValPtr VASTExprBuilder::buildAddExpr(ArrayRef<VASTValPtr> Ops,
                                         unsigned BitWidth) {
  SmallVector<VASTValPtr, 8> NewOps;
  VASTExprOpInfo<VASTExpr::dpAdd> OpInfo(*this, BitWidth);
  flattenExpr<VASTExpr::dpAdd>(Ops.begin(), Ops.end(),
                               op_filler<VASTExpr::dpAdd>(NewOps, OpInfo));

  // Add the immediate value back to the operand list.
  if (VASTValPtr V = OpInfo.flushImmOperand())
    NewOps.push_back(V);

  // If the addition contains only 2 operand, check if we can inline a operand
  // of this addition to make use of the carry bit.
  if (NewOps.size() == (OpInfo.Carry ? 1 : 2)) {
    bool MustHasCarry = (NewOps.size() != 1);
    unsigned ExprIdx = 0;
    VASTExpr *Expr = Context.getAddExprToFlatten(NewOps[ExprIdx],MustHasCarry);
    if (Expr == 0 && NewOps.size() > 1)
      Expr = Context.getAddExprToFlatten(NewOps[++ExprIdx], MustHasCarry);

    // If we can find such expression, flatten the expression tree.
    if (Expr) {
      // Try to keep the operand bitwidth unchanged.
      unsigned OpBitwidth = NewOps[ExprIdx]->getBitWidth();
      NewOps.erase(NewOps.begin() + ExprIdx);

      // Replace the expression by the no-carry operand
      VASTValPtr ExprLHS = Expr->getOperand(0);
      if (ExprLHS->getBitWidth() > OpBitwidth)
        ExprLHS = buildBitSliceExpr(ExprLHS, OpBitwidth, 0);

      NewOps.push_back(ExprLHS);

      VASTValPtr ExprRHS = Expr->getOperand(1);
      if (ExprRHS->getBitWidth() > OpBitwidth)
        ExprRHS = buildBitSliceExpr(ExprRHS, OpBitwidth, 0);

      NewOps.push_back(ExprRHS);;

      if (OpInfo.Carry) NewOps.push_back(OpInfo.Carry);

      assert(NewOps.size() < 4 && "Bad add folding!");
      return buildAddExpr(NewOps, BitWidth);
    }
  }

  // Sort the operands excluding carry bit, we want to place the carry bit at
  // last.
  std::sort(NewOps.begin(), NewOps.end(), VASTValPtr_less);

  // Add the carry bit back to the operand list.
  if (OpInfo.Carry) NewOps.push_back(OpInfo.Carry);

  // All operands are zero?
  if (NewOps.empty()) return Context.getOrCreateImmediate(UINT64_C(0),BitWidth);

  if (NewOps.size() == 1)
    // Pad the higer bits by zeros.
    return padHigherBits(NewOps.back(), BitWidth, false);

  // If one of the operand has tailing zeros, we can directly forward the value
  // of the corresponding bitslice of another operand.
  if (NewOps.size() == 2 && OpInfo.OpWithTailingZeros) {
    VASTValPtr NotEndWithZeros = NewOps[0],
               EndWithZeros = OpInfo.OpWithTailingZeros;
    if (NotEndWithZeros == EndWithZeros)
      NotEndWithZeros = NewOps[1];

    unsigned TailingZeros = OpInfo.MaxTailingZeros;

    VASTValPtr Hi =
      buildBitSliceExpr(EndWithZeros, EndWithZeros->getBitWidth(),TailingZeros);
    // NotEndWithZeros cannot entirely fit into the zero bits, addition is
    // need for the higher part.
    if (NotEndWithZeros->getBitWidth() > TailingZeros) {
      VASTValPtr HiAddOps[] = {
        buildBitSliceExpr(NotEndWithZeros, NotEndWithZeros->getBitWidth(),
                          TailingZeros),
        Hi
      };
      Hi = buildAddExpr(HiAddOps, BitWidth - TailingZeros);
      Hi = Context.nameExpr(Hi);
    } else
      // In this case, no addition is needed, we can simply concatenate the
      // operands together, still, we may pad the higher bit for additions.
      Hi = padHigherBits(Hi, BitWidth - TailingZeros, false);

    // We can directly forward the lower part.
    VASTValPtr Lo = buildBitSliceExpr(NotEndWithZeros, TailingZeros, 0);
    // Concatenate them together.
    VASTValPtr BitCatOps[] = { Hi, Lo };
    return buildBitCatExpr(BitCatOps, BitWidth);
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
