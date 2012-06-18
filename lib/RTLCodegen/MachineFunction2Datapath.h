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
// This file define the classes which convert MachineFunction to VASTExprs and
// vice versa.
//
//===----------------------------------------------------------------------===//
#ifndef VTM_MACHINE_FUNCTION_TO_DATAPATH
#define VTM_MACHINE_FUNCTION_TO_DATAPATH

#include "VASTExprBuilder.h"

#include "vtm/VInstrInfo.h"

#include "llvm/CodeGen/MachineFunction.h"

namespace llvm {
class MachineRegisterInfo;

class DatapathBuilderContext : public VASTExprBuilderContext {
public:
  virtual VASTValPtr getAsOperand(MachineOperand &Op,
                                  bool GetAsInlineOperand = true) {
    return 0;
  }
};

class DatapathBuilder : public VASTExprBuilder {
public:
  typedef std::map<unsigned, VASTValPtr> RegIdxMapTy;
  MachineRegisterInfo &MRI;
  RegIdxMapTy Idx2Expr;

  DatapathBuilderContext &getContext() {
    return reinterpret_cast<DatapathBuilderContext&>(Context);
  }

public:
  explicit DatapathBuilder(VASTExprBuilderContext &Context,
                           MachineRegisterInfo &MRI)
    : VASTExprBuilder(Context), MRI(MRI) {}

  VASTValPtr getAsOperand(MachineOperand &Op, bool GetAsInlineOperand = true) {
    return getContext().getAsOperand(Op, GetAsInlineOperand);
  }

  // Virtual register mapping.
  VASTValPtr getOrCreateExpr(unsigned RegNo, MachineInstr *MI = 0);
  VASTValPtr getOrCreateExpr(MachineInstr *MI) {
    return getOrCreateExpr(MI->getOperand(0).getReg(), MI);
  }

  VASTValPtr lookupExpr(unsigned RegNo) const;
  VASTValPtr indexVASTExpr(unsigned RegNo, VASTValPtr V);

  // Build VASTExpr from MachineInstr.
  VASTValPtr buildDatapathExpr(MachineInstr *MI);
  VASTValPtr buildUnaryOp(MachineInstr *MI, VASTExpr::Opcode Opc);
  VASTValPtr buildInvert(MachineInstr *MI);
  VASTValPtr buildReduceOr(MachineInstr *MI);
  template<typename FnTy>
  VASTValPtr buildBinaryOp(MachineInstr *MI, FnTy F) {
    return  F(getAsOperand(MI->getOperand(1)), getAsOperand(MI->getOperand(2)),
              VInstrInfo::getBitWidth(MI->getOperand(0)), this);
  }

  VASTValPtr buildLut(MachineInstr *MI);
  VASTValPtr buildSel(MachineInstr *MI);
  VASTValPtr buildAdd(MachineInstr *MI);
  VASTValPtr buildICmp(MachineInstr *MI);
  VASTValPtr buildBitSlice(MachineInstr *MI);

  // Create a condition from a predicate operand.
  VASTValPtr createCnd(MachineOperand Op);
  VASTValPtr createCnd(MachineInstr *MI) {
    return createCnd(*VInstrInfo::getPredOperand(MI));
  }
};
}

#endif
