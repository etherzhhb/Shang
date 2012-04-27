//===---- MicroState.cpp - Represent MicroState in a FSM state  --*- C++ -*-===//
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
//
//
//===----------------------------------------------------------------------===//

#include "vtm/MicroState.h"
#include "vtm/VerilogAST.h"
#include "vtm/VInstrInfo.h"
#include "vtm/VRegisterInfo.h"
#include "vtm/VerilogBackendMCTargetDesc.h"
#include "vtm/Utilities.h"

#include "llvm/Function.h"
#include "llvm/Metadata.h"
#include "llvm/Type.h"
#include "llvm/Constants.h"
#include "llvm/Target/TargetRegisterInfo.h"
#include "llvm/ADT/PointerIntPair.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

ucOperand ucOperand::CreatePredicate(unsigned Reg) {
  // Read reg0 means always execute.
  ucOperand MO = MachineOperand::CreateReg(Reg, false);
  MO.setBitWidth(1);
  return MO;
}

MachineOperand ucOperand::CreateTrace(MachineBasicBlock *MBB) {
  MachineOperand MO = MachineOperand::CreateImm(MBB->getNumber());
  MO.setTargetFlags(4);
  return MO;
}

ucOperand ucOperand::CreateReg(unsigned RegNum, unsigned BitWidth,
                               bool IsDef /* = false */) {
  ucOperand MO = MachineOperand::CreateReg(RegNum, IsDef);
  MO.setBitWidth(BitWidth);
  return MO;
}

ucOperand ucOperand::CreateImm(int64_t Val, unsigned BitWidth) {
  ucOperand MO = MachineOperand::CreateImm(Val);
  MO.setBitWidth(BitWidth);
  return MO;
}
