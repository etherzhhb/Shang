//===------------ VerilogASTExprs.cpp - Verilog AST Expressions -*- C++ -*-===//
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

#include "vtm/VerilogAST.h"
#include "vtm/Utilities.h"

using namespace llvm;

// Inline all operands in the expression whose Opcode is the same as Opc
// recursively;
static void flattenExpr(VASTValPtr V, VASTExpr::Opcode Opc,
                        SmallVectorImpl<VASTValPtr> &NewOps) {
  if (VASTExpr *Expr = dyn_cast<VASTExpr>(V)) {
    typedef const VASTUse *op_iterator;
    if (Expr->getOpcode() == Opc) {
      for (op_iterator I = Expr->op_begin(), E = Expr->op_end(); I != E; ++I)
        flattenExpr(I->getAsInlineOperand(), Opc, NewOps);

      return;
    }
  }

  NewOps.push_back(V);
}


static void flattenExprTree(VASTExpr::Opcode Opc, ArrayRef<VASTValPtr> Ops,
                            SmallVectorImpl<VASTValPtr> &NewOps) {
  for (unsigned i = 0, e = Ops.size(); i < e; ++i)
    // Try to flatten the expression tree.
    flattenExpr(Ops[i], Opc, NewOps);
}

VASTValPtr VASTModule::buildNotExpr(VASTValPtr U) {
  U = U.invert();

  if (U.isInverted()) {
    // Try to fold the not expression.
    if (VASTImmediate *Imm = dyn_cast<VASTImmediate>(U.get()))
      return getOrCreateImmediate(~Imm->getValue(), Imm->getBitWidth());
  }

  return U;
}

VASTValPtr VASTModule::foldBitSliceExpr(VASTValPtr U, uint8_t UB, uint8_t LB) {
  unsigned OperandSize = U->getBitWidth();
  // Not a sub bitslice.
  if (UB == OperandSize && LB == 0) return U;

  // Try to fold the bitslice.
  VASTValue *V = U.get();
  bool isInverted = U.isInverted();
  VASTExpr *AssignExpr = 0;
  if (VASTExpr *E = dyn_cast<VASTExpr>(V))
    AssignExpr = E;
  else if (VASTWire *W = dyn_cast<VASTWire>(V)) {
    VASTExprPtr P = W->getExpr().invert(isInverted);
    // DirtyHack: the Invert information will be lost by this way. Allocate a
    // new VASTExprPtr?
    AssignExpr = P.get();
    isInverted = P.isInverted();
  }

  if (AssignExpr) {
    switch(AssignExpr->getOpcode()) {
    default: break;
    case VASTExpr::dpAssign: {
      unsigned Offset = AssignExpr->LB;
      UB += Offset;
      LB += Offset;
      return buildBitSliceExpr(AssignExpr->getOperand(0), UB, LB).invert(isInverted);
    }
    case VASTExpr::dpBitCat: {
      VASTValPtr Hi = AssignExpr->getOperand(0),
                 Lo = AssignExpr->getOperand(1);
      if (Lo->getBitWidth() == LB)
        return buildBitSliceExpr(Hi, UB - LB, 0).invert(isInverted);
      break;
    }
    }
  }

  if (VASTImmediate *Imm = dyn_cast<VASTImmediate>(V)) {
    uint64_t imm = getBitSlice64(Imm->getValue(), UB, LB);
    if (isInverted) imm = ~imm;
    return getOrCreateImmediate(imm, UB - LB);
  }

  return VASTValPtr(0);
}

VASTValPtr VASTModule::buildBitCatExpr(ArrayRef<VASTValPtr> Ops,
                                       unsigned BitWidth) {
  VASTImmediate *LastImm = dyn_cast<VASTImmediate>(Ops[0]);
  SmallVector<VASTValPtr, 8> NewOps;
  NewOps.push_back(Ops[0]);

  // Merge the constant sequence.
  for (unsigned i = 1; i < Ops.size(); ++i) {
    VASTImmediate *CurImm = dyn_cast<VASTImmediate>(Ops[i]);

    if (!CurImm) {
      LastImm = 0;
      NewOps.push_back(Ops[i]);
      continue;
    }

    if (LastImm) {
      // Merge the constants.
      uint64_t HiVal = LastImm->getValue(), LoVal = CurImm->getValue();
      unsigned HiSizeInBits = LastImm->getBitWidth(),
               LoSizeInBits = CurImm->getBitWidth();
      unsigned SizeInBits = LoSizeInBits + HiSizeInBits;
      assert(SizeInBits <= 64 && "Constant too large!");
      uint64_t Val = (LoVal) | (HiVal << LoSizeInBits);
      LastImm = getOrCreateImmediate(Val, SizeInBits);
      NewOps.back() = LastImm;
    } else {
      LastImm = CurImm;
      NewOps.push_back(Ops[i]);
    }
  }

  if (NewOps.size() == 1) return NewOps.back();

  // FIXME: Flatten bitcat.
  return createExpr(VASTExpr::dpBitCat, NewOps, BitWidth, 0);
}

VASTValPtr VASTModule::buildBitSliceExpr(VASTValPtr U, uint8_t UB, uint8_t LB) {
  // Try to fold the expression.
  if (VASTValPtr P = foldBitSliceExpr(U, UB, LB)) return P;

  assert(UB <= U->getBitWidth() && UB > LB && "Bad bit range!");
  // FIXME: We can name the expression when necessary.
  assert(isa<VASTNamedValue>(U)
         && cast<VASTNamedValue>(U)->getName()
         && *cast<VASTNamedValue>(U)->getName() != '\0'
         && "Cannot get bitslice of value without name!");

  VASTValPtr Ops[] = { U };
  return createExpr(VASTExpr::dpAssign, Ops, UB, LB);
}

VASTValPtr VASTModule::buildReduction(VASTExpr::Opcode Opc,VASTValPtr Op,
                                      unsigned BitWidth) {
  assert(BitWidth == 1 && "Bitwidth of reduction should be 1!");

  if (VASTImmediate *Imm = dyn_cast<VASTImmediate>(Op)) {
    uint64_t Val = Imm->getValue();
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

  return createExpr(Opc, Op, BitWidth, 0);
}

VASTValPtr VASTModule::buildExpr(VASTExpr::Opcode Opc, VASTValPtr Op,
                                 unsigned BitWidth) {
  switch (Opc) {
  default: break;
  case VASTExpr::dpROr:
  case VASTExpr::dpRAnd:
  case VASTExpr::dpRXor:
    return buildReduction(Opc, Op, BitWidth);
  }

  VASTValPtr Ops[] = { Op };
  return createExpr(Opc, Ops, BitWidth, 0);
}

static bool VASTValPtr_less(const VASTValPtr LHS, const VASTValPtr RHS) {
  if (LHS->getASTType() < RHS->getASTType()) return true;
  else if (LHS->getASTType() > RHS->getASTType()) return false;

  return LHS < RHS;
}

VASTValPtr
VASTModule::getOrCreateCommutativeExpr(VASTExpr::Opcode Opc,
                                       SmallVectorImpl<VASTValPtr> &Ops,
                                       unsigned BitWidth) {
  std::sort(Ops.begin(), Ops.end(), VASTValPtr_less);
  return createExpr(Opc, Ops, BitWidth, 0);
}

VASTValPtr VASTModule::buildAndExpr(ArrayRef<VASTValPtr> Ops,
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
    flattenExpr(Ops[i], VASTExpr::dpAnd, NewOps);
  }

  if (NewOps.empty())
    return getOrCreateImmediate(getBitSlice64(~0ull, BitWidth), BitWidth);

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

  return createExpr(VASTExpr::dpAnd, NewOps, BitWidth, 0);
}

VASTValPtr VASTModule::buildExpr(VASTExpr::Opcode Opc, VASTValPtr LHS,
                                 VASTValPtr RHS, unsigned BitWidth) {
  VASTValPtr Ops[] = { LHS, RHS };
  return buildExpr(Opc, Ops, BitWidth);
}

VASTValPtr VASTModule::buildExpr(VASTExpr::Opcode Opc, VASTValPtr Op0,
                                 VASTValPtr Op1, VASTValPtr Op2,
                                 unsigned BitWidth) {
  VASTValPtr Ops[] = { Op0, Op1, Op2 };
  return buildExpr(Opc, Ops, BitWidth);
}

VASTValPtr VASTModule::buildExpr(VASTExpr::Opcode Opc, ArrayRef<VASTValPtr> Ops,
                                 unsigned BitWidth) {
  switch (Opc) {
  default: break;
  case VASTExpr::dpAdd:  return buildAddExpr(Ops, BitWidth);
  case VASTExpr::dpMul:  return buildMulExpr(Ops, BitWidth);
  case VASTExpr::dpAnd:  return buildAndExpr(Ops, BitWidth);
  case VASTExpr::dpBitCat: return buildBitCatExpr(Ops, BitWidth);
  }

  return createExpr(Opc, Ops, BitWidth, 0);
}

VASTValPtr VASTModule::buildMulExpr(ArrayRef<VASTValPtr> Ops,
                                    unsigned BitWidth) {
  SmallVector<VASTValPtr, 8> NewOps;
  flattenExprTree(VASTExpr::dpMul, Ops, NewOps);

  return getOrCreateCommutativeExpr(VASTExpr::dpMul, NewOps, BitWidth);
}

VASTValPtr VASTModule::buildAddExpr(ArrayRef<VASTValPtr> Ops,
                                    unsigned BitWidth) {
  SmallVector<VASTValPtr, 8> NewOps;
  flattenExprTree(VASTExpr::dpAdd, Ops, NewOps);
  return createExpr(VASTExpr::dpAdd, NewOps, BitWidth, 0);
}

VASTValPtr VASTModule::buildOrExpr(ArrayRef<VASTValPtr> Ops,
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

VASTValPtr VASTModule::buildXorExpr(ArrayRef<VASTValPtr> Ops,
                                    unsigned BitWidth) {
  assert (Ops.size() == 2 && "There should be more than one operand!!");

  // Build the Xor Expr with the And Inverter Graph (AIG).
  return buildExpr(VASTExpr::dpAnd, buildOrExpr(Ops, BitWidth),
                   buildNotExpr(buildAndExpr(Ops, BitWidth)),
                   BitWidth);
}

VASTValPtr VASTModule::createExpr(VASTExpr::Opcode Opc,
                                  ArrayRef<VASTValPtr> Ops,
                                  unsigned UB, unsigned LB) {
  assert(!Ops.empty() && "Unexpected empty expression");
  if (Ops.size() == 1) {
    switch (Opc) {
    default: break;
    case VASTExpr::dpAnd: case VASTExpr::dpAdd: case VASTExpr::dpMul:
      return Ops[0];
    }
  }

  FoldingSetNodeID ID;

  // Profile the elements of VASTExpr.
  ID.AddInteger(Opc);
  ID.AddInteger(UB);
  ID.AddInteger(LB);
  for (unsigned i = 0; i < Ops.size(); ++i) {
    ID.AddPointer(Ops[i].get());
    ID.AddBoolean(Ops[i].isInverted());
  }
  void *IP = 0;
  if (VASTExpr *E = UniqueExprs.FindNodeOrInsertPos(ID, IP))
    return E;

  // If the Expression do not exist, allocate a new one.
  // Place the VASTUse array right after the VASTExpr.
  void *P = Allocator.Allocate(sizeof(VASTExpr) + Ops.size() * sizeof(VASTUse),
                               alignOf<VASTExpr>());
  VASTExpr *E = new (P) VASTExpr(Opc, Ops.size(), UB, LB,
                                 ID.Intern(Allocator));
  UniqueExprs.InsertNode(E, IP);

  // Initialize the use list and compute the actual size of the expression.
  unsigned ExprSize = 0;

  for (unsigned i = 0; i < Ops.size(); ++i) {
    if (VASTExpr *E = Ops[i].getAsLValue<VASTExpr>()) ExprSize += E->ExprSize;
    else                                              ++ExprSize;
    
    (void) new (E->ops() + i) VASTUse(Ops[i], E);
  }

  E->ExprSize = ExprSize;
  return E;
}
