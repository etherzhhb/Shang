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
#include "vtm/VerilgoBackendMCTargetDesc.h"
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

static uint64_t getMachineOperandHashValue(const MachineOperand &MO) {
  uint64_t Key = (uint64_t)MO.getType() << 32;
  switch (MO.getType()) {
  default: break;
  case MachineOperand::MO_Register:
    if (MO.isDef() && TargetRegisterInfo::isVirtualRegister(MO.getReg()))
      return 0;  // Skip virtual register defs.
    Key |= MO.getReg();
    break;
  case MachineOperand::MO_Immediate:
    Key |= MO.getImm();
    break;
  case MachineOperand::MO_FrameIndex:
  case MachineOperand::MO_ConstantPoolIndex:
  case MachineOperand::MO_JumpTableIndex:
    Key |= MO.getIndex();
    break;
  case MachineOperand::MO_MachineBasicBlock:
    Key |= DenseMapInfo<void*>::getHashValue(MO.getMBB());
    break;
  case MachineOperand::MO_GlobalAddress:
    Key |= DenseMapInfo<void*>::getHashValue(MO.getGlobal());
    break;
  case MachineOperand::MO_BlockAddress:
    Key |= DenseMapInfo<void*>::getHashValue(MO.getBlockAddress());
    break;
  case MachineOperand::MO_MCSymbol:
    Key |= DenseMapInfo<void*>::getHashValue(MO.getMCSymbol());
    break;
  }
  Key += ~(Key << 32);
  Key ^= (Key >> 22);
  Key += ~(Key << 13);
  Key ^= (Key >> 8);
  Key += (Key << 3);
  Key ^= (Key >> 15);
  Key += ~(Key << 27);
  Key ^= (Key >> 31);

  return Key;
}

unsigned ucOperandValueTrait::getHashValue(ucOperand Op) {
  return getMachineOperandHashValue(Op);
}

bool ucOperand::isPredicateInverted() const {
  return getTargetFlags() & VInstrInfo::PredInvertFlag;
}

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

void ucOperand::print(raw_ostream &OS,
                      unsigned UB /* = 64 */, unsigned LB /* = 0 */,
                      bool isPredicate /* = false */) {
  switch (getType()) {
  case MachineOperand::MO_ExternalSymbol:
    UB = std::min(getBitWidth(), UB);
    OS << getSymbolName();
    OS << verilogBitRange(UB, LB, getBitWidth() != 1);
    return;
  case MachineOperand::MO_GlobalAddress:
    OS << "(`gv" << VBEMangle(getGlobal()->getName());
    if (int64_t Offset = getOffset())
      OS  << " + " << verilogConstToStr(Offset, getBitWidth(), false);
    OS << ')';
    return;
  default: break;
  }

  MachineOperand::print(OS);
}
