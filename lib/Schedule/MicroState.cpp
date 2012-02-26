//===---- MicroState.cpp - Represent MicroState in a FSM state  --*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
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

ucOperand ucOperand::CreateWire(unsigned WireNum, unsigned BitWidth,
                                bool IsDef /* = false */) {
  ucOperand MO = MachineOperand::CreateReg(WireNum, IsDef);
  MO.setBitWidth(BitWidth);
  MO.setIsWire();
  return MO;
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

bool ucOperand::isWire() const {
  return isReg() && (IsWireFlag & getTargetFlags());
}

ucOperand::bit_range ucOperand::getBitRange() const {
  unsigned Offset = 0;

  //unsigned Reg = getReg();
  //if (TargetRegisterInfo::isPhysicalRegister(Reg))
  //  Offset = (Reg & 0x7) * 8;

  return std::make_pair(getBitWidth() + Offset, 0 + Offset);
}

void ucOperand::print(raw_ostream &OS,
                      unsigned UB /* = 64 */, unsigned LB /* = 0 */,
                      bool isPredicate /* = false */) {
  switch (getType()) {
  case MachineOperand::MO_Register: {
    UB = std::min(getBitWidthOrZero(), UB);
    if (isImplicit() || getReg() == 0) break;

    unsigned Reg = getReg();
    std::string BitRange = "";
    std::string Prefix = "reg";

    OS << "/*";
    if (isDef())
      OS << "def_";
    else {
      OS << "use_";
      if (isKill()) OS << "kill_";
    }

    if (TargetRegisterInfo::isVirtualRegister(Reg)) {
      //DEBUG(
        if (MachineInstr *MI = getParent()) {
          MachineRegisterInfo &MRI = MI->getParent()->getParent()->getRegInfo();
          const TargetRegisterClass *RC = MRI.getRegClass(Reg);
          OS << RC->getName();
        }
      //);
      Reg = TargetRegisterInfo::virtReg2Index(Reg);
      if (!isPredicate) BitRange = verilogBitRange(UB, LB, getBitWidth() != 1);

      if (isWire()) {
        OS << "*/ wire" << Reg << BitRange;
        return;
      }
    } else { // Compute the offset of physics register.
      Prefix = "phy_reg";
      if (!isPredicate) BitRange = verilogBitRange(UB, LB, getBitWidth() != 1);
    }

    OS << "_reg" << Reg <<"*/ " << Prefix << Reg << BitRange;
    return;
  }
  case MachineOperand::MO_Immediate:
    UB = std::min(getBitWidthOrZero(), UB);
    assert(UB == getBitWidth() && LB == 0 && "Get BitSlice of constant!");
    OS << verilogConstToStr(getImm(), getBitWidth(), false);
    return;
  case MachineOperand::MO_ExternalSymbol:
    UB = std::min(getBitWidth(), UB);
    OS << getSymbolName();
    OS << verilogBitRange(UB, LB, getBitWidth() != 1);
    return;
  case MachineOperand::MO_GlobalAddress:
    OS << "(`gv" << getGlobal()->getName() << " + "
       << verilogConstToStr(getOffset(), getBitWidth(), false) << ')';
    return;
  default: break;
  }

  MachineOperand::print(OS);
}
