//===- MachineFunction2Datapath.cpp - MachineFunction <-> VAST --*- C++ -*-===//
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
// This file implement the classes which convert MachineFunction to VASTExprs and
// vice versa.
//
//===----------------------------------------------------------------------===//

#include "MachineFunction2Datapath.h"
#include "llvm/Target/TargetRegisterInfo.h"

using namespace llvm;

VASTValPtr DatapathBuilder::createCnd(MachineOperand Op) {
  // Is there an always true predicate?
  if (VInstrInfo::isAlwaysTruePred(Op)) return getBoolImmediate(true);

  bool isInverted = VInstrInfo::isPredicateInverted(Op);
  // Fix the bitwidth, the bitwidth of condition is always 1.
  VInstrInfo::setBitWidth(Op, 1);

  // Otherwise it must be some signal.
  VASTValPtr C = getAsOperand(Op);

  if (isInverted) C = buildNotExpr(C);

  return C;
}

VASTValPtr DatapathBuilder::buildDatapathExpr(MachineInstr *MI) {
  switch (MI->getOpcode()) {
  case VTM::VOpBitSlice:  return buildBitSlice(MI);
  case VTM::VOpBitCat:
    return buildBinaryOp(MI, VASTExprBuilder::buildExpr<VASTExpr::dpBitCat>);
  case VTM::VOpBitRepeat:
    return buildBinaryOp(MI, VASTExprBuilder::buildExpr<VASTExpr::dpBitRepeat>);

  case VTM::VOpAdd_c:     return buildAdd(MI);
  case VTM::VOpICmp_c:    return buildICmp(MI);

  case VTM::VOpSHL_c:
    return buildBinaryOp(MI, VASTExprBuilder::buildExpr<VASTExpr::dpShl>);
  case VTM::VOpSRA_c:
    return buildBinaryOp(MI, VASTExprBuilder::buildExpr<VASTExpr::dpSRA>);
  case VTM::VOpSRL_c:
    return buildBinaryOp(MI, VASTExprBuilder::buildExpr<VASTExpr::dpSRL>);

  case VTM::VOpMultLoHi_c:
  case VTM::VOpMult_c:
    return buildBinaryOp(MI, VASTExprBuilder::buildExpr<VASTExpr::dpMul>);

  case VTM::VOpSel:       return buildSel(MI);

  case VTM::VOpLUT:       return buildLut(MI);

  case VTM::VOpXor:       return buildBinaryOp(MI, VASTExprBuilder::buildXor);
  case VTM::VOpAnd:
    return buildBinaryOp(MI, VASTExprBuilder::buildExpr<VASTExpr::dpAnd>);
  case VTM::VOpOr:        return buildBinaryOp(MI, VASTExprBuilder::buildOr);
  case VTM::VOpNot:       return buildInvert(MI);
  case VTM::VOpROr:       return buildReduceOr(MI);
  case VTM::VOpRAnd:      return buildUnaryOp(MI, VASTExpr::dpRAnd);
  case VTM::VOpRXor:      return buildUnaryOp(MI, VASTExpr::dpRXor);
  case VTM::VOpPipelineStage: return buildUnaryOp(MI, VASTExpr::dpAssign);
  default:  assert(0 && "Unexpected opcode!");    break;
  }

  return VASTValPtr(0);
}

VASTValPtr DatapathBuilder::buildAdd(MachineInstr *MI) {
  return buildExpr(VASTExpr::dpAdd, getAsOperand(MI->getOperand(1)),
                                    getAsOperand(MI->getOperand(2)),
                                    getAsOperand(MI->getOperand(3)),
                   VInstrInfo::getBitWidth(MI->getOperand(0)));
}

VASTValPtr DatapathBuilder::buildICmp(MachineInstr *MI) {
  unsigned CndCode = MI->getOperand(3).getImm();
  if (CndCode == VFUs::CmpSigned)
    return buildBinaryOp(MI, VASTExprBuilder::buildExpr<VASTExpr::dpSCmp>);

  // else
  return buildBinaryOp(MI, VASTExprBuilder::buildExpr<VASTExpr::dpUCmp>);
}

VASTValPtr DatapathBuilder::buildInvert(MachineInstr *MI) {
  return buildNotExpr(getAsOperand(MI->getOperand(1)));
}

VASTValPtr DatapathBuilder::buildReduceOr(MachineInstr *MI) {
  // A | B .. | Z = ~(~A & ~B ... & ~Z).
  VASTValPtr V = buildNotExpr(getAsOperand(MI->getOperand(1)));
  V = buildNotExpr(buildExpr(VASTExpr::dpRAnd, V, 1));
  return V;
}

VASTValPtr DatapathBuilder::buildUnaryOp(MachineInstr *MI,
                                          VASTExpr::Opcode Opc) {
  return buildExpr(Opc, getAsOperand(MI->getOperand(1)),
                            VInstrInfo::getBitWidth(MI->getOperand(0)));
}

VASTValPtr DatapathBuilder::buildSel(MachineInstr *MI) {
  VASTValPtr Ops[] = { createCnd(MI->getOperand(1)),
                       getAsOperand(MI->getOperand(2)),
                       getAsOperand(MI->getOperand(3)) };

  return buildExpr(VASTExpr::dpSel, Ops,
                   VInstrInfo::getBitWidth(MI->getOperand(0)));
}

VASTValPtr DatapathBuilder::buildLut(MachineInstr *MI) {
  unsigned SizeInBits = VInstrInfo::getBitWidth(MI->getOperand(0));

  SmallVector<VASTValPtr, 8> Operands;
  for (unsigned i = 4, e = MI->getNumOperands(); i < e; ++i)
    Operands.push_back(getAsOperand(MI->getOperand(i)));
  unsigned NumInputs = Operands.size();

  // Interpret the sum of product table.
  const char *p = MI->getOperand(1).getSymbolName();
  SmallVector<VASTValPtr, 8> ProductOps, SumOps;
  bool isComplement = false;

  while (*p) {
    // Interpret the product.
    ProductOps.clear();
    for (unsigned i = 0; i < NumInputs; ++i) {
      char c = *p++;
      switch (c) {
      default: llvm_unreachable("Unexpected SOP char!");
      case '-': /*Dont care*/ break;
      case '1': ProductOps.push_back(Operands[i]); break;
      case '0':
        ProductOps.push_back(buildNotExpr(Operands[i]));
        break;
      }
    }

    // Inputs and outputs are seperated by blank space.
    assert(*p == ' ' && "Expect the blank space!");
    ++p;

    // Create the product.
    // Add the product to the operand list of the sum.
    SumOps.push_back(buildAndExpr(ProductOps, SizeInBits));

    // Is the output inverted?
    char c = *p++;
    assert((c == '0' || c == '1') && "Unexpected SOP char!");
    isComplement = (c == '0');

    // Products are separated by new line.
    assert(*p == '\n' && "Expect the new line!");
    ++p;
  }

  // Or the products together to build the SOP (Sum of Product).
  VASTValPtr SOP = buildOrExpr(SumOps, SizeInBits);

  if (isComplement) SOP = buildNotExpr(SOP);

  // Build the sum;
  return SOP;
}

VASTValPtr DatapathBuilder::buildBitSlice(MachineInstr *MI) {
  // Get the range of the bit slice, Note that the
  // bit at upper bound is excluded in VOpBitSlice
  unsigned UB = MI->getOperand(2).getImm(),
           LB = MI->getOperand(3).getImm();

  // RHS should be a register.
  MachineOperand &MO = MI->getOperand(1);
  return buildBitSliceExpr(getAsOperand(MO), UB, LB);
}

VASTValPtr DatapathBuilder::createAndIndexExpr(MachineInstr *MI,
                                               bool mayExisted) {
  unsigned RegNo = MI->getOperand(0).getReg();
  // Ignore the dead data-path.
  if (RegNo == 0) return 0;
  
  assert(TargetRegisterInfo::isVirtualRegister(RegNo)
         && "Expected virtual register!");
  VASTValPtr &Expr = Idx2Expr[RegNo];
  assert((!Expr || mayExisted) && "Expression had already been created!");

  if (Expr) return Expr;
  
  return (Expr = buildDatapathExpr(MI));
}

VASTValPtr DatapathBuilder::lookupExpr(unsigned RegNo) const {
   assert(TargetRegisterInfo::isVirtualRegister(RegNo)
          && "Expect virtual register!");
   RegIdxMapTy::const_iterator at = Idx2Expr.find(RegNo);
   return at == Idx2Expr.end() ? 0 : at->second;
 }

VASTValPtr DatapathBuilder::indexVASTExpr(unsigned RegNo, VASTValPtr V) {
  assert(TargetRegisterInfo::isVirtualRegister(RegNo)
    && "Expect physical register!");
  bool inserted = Idx2Expr.insert(std::make_pair(RegNo, V)).second;
  assert(inserted && "RegNum already indexed some value!");

  return V;
}
