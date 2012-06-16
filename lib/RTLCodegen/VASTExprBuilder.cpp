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

VASTValPtr VASTExprBuilder::trimZeros(VASTValPtr V, unsigned &Offset) {
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

  if (U.isInverted()) {
    // Try to fold the not expression.
    if (VASTImmediate *Imm = dyn_cast<VASTImmediate>(U.get()))
      return Context.getOrCreateImmediate(~Imm->getUnsignedValue(),
                                          Imm->getBitWidth());
  }

  return U;
}

VASTValPtr VASTExprBuilder::foldBitSliceExpr(VASTValPtr U, uint8_t UB,
                                             uint8_t LB) {
  unsigned OperandSize = U->getBitWidth();
  // Not a sub bitslice.
  if (UB == OperandSize && LB == 0) return U;

  // Try to fold the bitslice.
  VASTValue *V = U.get();
  bool isInverted = U.isInverted();

  if (VASTImmediate *Imm = dyn_cast<VASTImmediate>(V)) {
    uint64_t imm = getBitSlice64(Imm->getUnsignedValue(), UB, LB);
    if (isInverted) imm = ~imm;
    return Context.getOrCreateImmediate(imm, UB - LB);
  }

  VASTExpr *Expr = dyn_cast<VASTExpr>(V);

  if (Expr == 0) return VASTValPtr(0);

  if (Expr->getOpcode() == VASTExpr::dpAssign){
    unsigned Offset = Expr->LB;
    UB += Offset;
    LB += Offset;
    return buildBitSliceExpr(Expr->getOperand(0), UB, LB).invert(isInverted);
  }

  if (Expr->getOpcode() == VASTExpr::dpBitCat) {
    VASTValPtr Hi = Expr->getOperand(0),
               Lo = Expr->getOperand(1);
    unsigned SplitBit = Lo->getBitWidth();
    if (UB <= SplitBit)
      return buildBitSliceExpr(Lo, UB, LB).invert(isInverted);
    if (LB >= SplitBit)
      return buildBitSliceExpr(Hi, UB - SplitBit, LB - SplitBit).invert(isInverted);

    VASTValPtr Ops[] = { buildBitSliceExpr(Hi, UB - SplitBit, 0),
                         buildBitSliceExpr(Lo, SplitBit, LB) };
    return buildBitCatExpr(Ops, UB - LB).invert(isInverted);
  }

  return VASTValPtr(0);
}

VASTValPtr VASTExprBuilder::buildBitCatExpr(ArrayRef<VASTValPtr> Ops,
                                            unsigned BitWidth) {
  SmallVector<VASTValPtr, 8> NewOps(Ops.begin(), Ops.end());

  VASTImmediate *LastImm = dyn_cast<VASTImmediate>(NewOps[0]);
  // Skip the first operand since we have already processed it.
  unsigned ActualOpPos = 1;

  // Merge the constant sequence.
  for (unsigned i = 1; i < NewOps.size(); ++i) {
    VASTImmediate *CurImm = dyn_cast<VASTImmediate>(NewOps[i]);

    if (!CurImm) {
      LastImm = 0;
      NewOps[ActualOpPos++] = NewOps[i];
      continue;
    }

    // Now CurImm holds a immediate.
    if (LastImm) {
      // Merge the constants.
      uint64_t HiVal = LastImm->getUnsignedValue(),
               LoVal = CurImm->getUnsignedValue();
      unsigned HiSizeInBits = LastImm->getBitWidth(),
               LoSizeInBits = CurImm->getBitWidth();
      unsigned SizeInBits = LoSizeInBits + HiSizeInBits;
      assert(SizeInBits <= 64 && "Constant too large!");
      uint64_t Val = (LoVal) | (HiVal << LoSizeInBits);
      LastImm = Context.getOrCreateImmediate(Val, SizeInBits);
      NewOps[ActualOpPos] = LastImm;
    } else {
      // Add the immediate to operand list.
      LastImm = CurImm;
      NewOps[ActualOpPos++] = NewOps[i];
    }
  }

  if (ActualOpPos == 1) return NewOps.back();
  NewOps.resize(ActualOpPos);

  // FIXME: Flatten bitcat.
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

VASTValPtr VASTExprBuilder::buildReduction(VASTExpr::Opcode Opc,VASTValPtr Op,
                                           unsigned BitWidth) {
  assert(BitWidth == 1 && "Bitwidth of reduction should be 1!");

  if (VASTImmediate *Imm = dyn_cast<VASTImmediate>(Op)) {
    uint64_t Val = Imm->getUnsignedValue();
    switch (Opc) {
    case VASTExpr::dpROr:
      // Only reduce to 0 if all bits are 0.
      if (isAllZeros64(Val, Imm->getBitWidth()))
        return getBoolImmediate(false);
      else
        return getBoolImmediate(true);
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

  if (VASTExpr *Expr = dyn_cast<VASTExpr>(Op)) {
    switch (Expr->getOpcode()) {
    default: break;
    case VASTExpr::dpBitCat: {
      SmallVector<VASTValPtr, 8> Ops;
      typedef VASTExpr::op_iterator it;
      for (it I = Expr->op_begin(), E = Expr->op_end(); I != E; ++I)
        Ops.push_back(buildReduction(Opc, *I, BitWidth));

      switch (Opc) {
      case VASTExpr::dpROr:   return buildOrExpr(Ops, BitWidth);
      case VASTExpr::dpRAnd: return buildAndExpr(Ops, BitWidth);
      case VASTExpr::dpRXor: return buildXorExpr(Ops, BitWidth);
      default:  llvm_unreachable("Unexpected Reduction Node!");
      }
    }
    }
  } else if (Op.isInverted()) {
    switch (Opc) {
    case VASTExpr::dpROr: // ~(A & B) = (~A | ~B)
      return buildNotExpr(buildReduction(VASTExpr::dpRAnd,Op.invert(),BitWidth));
    case VASTExpr::dpRAnd:
      return buildNotExpr(buildReduction(VASTExpr::dpROr,Op.invert(),BitWidth));
    default: break;
    }
  }

  return Context.createExpr(Opc, Op, BitWidth, 0);
}

VASTValPtr VASTExprBuilder::buildExpr(VASTExpr::Opcode Opc, VASTValPtr Op,
                                      unsigned BitWidth) {
  switch (Opc) {
  default: break;
  case VASTExpr::dpROr:
  case VASTExpr::dpRAnd:
  case VASTExpr::dpRXor:
    return buildReduction(Opc, Op, BitWidth);
  }

  VASTValPtr Ops[] = { Op };
  return Context.createExpr(Opc, Ops, BitWidth, 0);
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

VASTValPtr VASTExprBuilder::buildAndExpr(ArrayRef<VASTValPtr> Ops,
                                         unsigned BitWidth) {
  SmallVector<VASTValPtr, 8> NewOps;
  typedef const VASTUse *op_iterator;

  for (unsigned i = 0; i < Ops.size(); ++i) {
    // The expression is actually commutative only if all its operands have the
    // same bitwidth.
    assert(BitWidth == Ops[i]->getBitWidth() && "Bitwidth not match!");

    if (VASTImmediate *Imm = dyn_cast<VASTImmediate>(Ops[i])) {
      // X & 1 = X;
      if (Imm->isAllOnes()) continue;

      // X & 0 = 0;
      if (Imm->isAllZeros()) return Imm;
    }

    // Try to flatten the expression tree.
    flattenExpr<VASTExpr::dpAnd>(Ops[i], std::back_inserter(NewOps));
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
  }

  return Context.createExpr(Opc, Ops, BitWidth, 0);
}

VASTValPtr VASTExprBuilder::buildMulExpr(ArrayRef<VASTValPtr> Ops,
                                         unsigned BitWidth) {
  SmallVector<VASTValPtr, 8> NewOps;
  flattenExpr<VASTExpr::dpMul>(Ops.begin(), Ops.end(),
                               std::back_inserter(NewOps));

  return getOrCreateCommutativeExpr(VASTExpr::dpMul, NewOps, BitWidth);
}

VASTValPtr VASTExprBuilder::buildAddExpr(ArrayRef<VASTValPtr> Ops,
                                         unsigned BitWidth) {
  SmallVector<VASTValPtr, 8> NewOps;
  uint64_t ImmVal = 0;
  unsigned MaxImmWidth = 0;
  VASTValPtr Carry = 0;
  for (unsigned i = 0; i < Ops.size(); ++i) {
    VASTValPtr V = Ops[i];
    // X + 0 = 0;
    if (VASTImmediate *Imm = dyn_cast<VASTImmediate>(V)) {
      ImmVal += Imm->getUnsignedValue();
      MaxImmWidth = std::max(MaxImmWidth, Imm->getBitWidth());
      continue;
    }

    // Discard the leading zeros of the operand of addition;
    V = trimLeadingZeros(V);

    if (V->getBitWidth() == 1) {
      assert(!Carry && "unexpected multiple carry bit!");
      Carry = V;
      continue;
    }

    NewOps.push_back(V);
  }

  // Add the carry bit back to the operand list.
  if (Carry) NewOps.push_back(Carry);

  // Add the immediate value back to the operand list.
  if (ImmVal)
    NewOps.push_back(Context.getOrCreateImmediate(ImmVal, MaxImmWidth));

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
