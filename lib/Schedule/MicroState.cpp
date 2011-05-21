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
#include "vtm/VTM.h"

#include "llvm/Function.h"
#include "llvm/Metadata.h"
#include "llvm/Type.h"
#include "llvm/Constants.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/ADT/PointerIntPair.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/Debug.h"

using namespace llvm;

void ucOp::print(raw_ostream &OS) const {
  const TargetMachine &TM =
    OpCode.getParent()->getParent()->getParent()->getTarget();

  OS << TM.getInstrInfo()->get(OpCode.getOpcode()).getName()
     << "{" << OpCode.getFUId() << "}"
     << "@" << OpCode.getPredSlot();
  OS << ' ';
  // Print the operands;
  for (op_iterator I = op_begin(), E = op_end(); I != E; ++I) {
    ucOperand &MOP = *I;
    MOP.print(OS);
    OS << ", ";
  }
}

void ucOp::dump() const {
  print(dbgs());
  dbgs() << '\n';
}

ucOp::op_iterator ucOpIterator::getNextIt() const {
  ucOp::op_iterator NextIt = CurIt;

  assert(ucOperand(*NextIt).isOpcode() && "Bad leading token");

  while (++NextIt != EndIt) {
    ucOperand &TokenOp = *NextIt;
    if (TokenOp.isOpcode()) break;
  }

  return NextIt;
}

// Out of line virtual function to provide home for the class.
void ucOp::anchor() {}

// Out of line virtual function to provide home for the class.
void ucOpIterator::anchor() {}

void ucState::print(raw_ostream &OS) const {
  switch (Instr.getOpcode()) {
  case VTM::Control:  OS << "Control ";   break;
  case VTM::Datapath: OS << "Datapath ";  break;
  default:  assert(0 && "Bad opcode!");   break;
  }

  OS << '@' << getSlot() << '\n';
  for (ucState::iterator I = begin(), E = end(); I != E; ++I) {
    (*I).print(OS.indent(2));
    OS << '\n';
  }
}

void ucState::dump() const {
  print(dbgs());
}

// Out of line virtual function to provide home for the class.
void ucState::anchor() {}

ucOperand ucOperand::CreateOpcode(unsigned Opcode, unsigned PredSlot,
                                  FuncUnitId FUId /*= VFUs::Trivial*/) {
  uint64_t Context = 0x0;
  Context |= (uint64_t(Opcode & OpcodeMask) << OpcodeShiftAmount);
  Context |= (uint64_t(PredSlot & PredSlotMask) << PredSlotShiftAmount);
  Context |= (uint64_t(FUId.getData() & FUIDMask) << FUIDShiftAmount);
  ucOperand MO = cast<ucOperand>(MachineOperand::CreateImm(Context));
  MO.setTargetFlags(IsOpcode);
  assert((MO.getOpcode() == Opcode && MO.getPredSlot() == PredSlot
          && MO.getFUId() == FUId) && "Data overflow!");
  return MO;
}

ucOperand ucOperand::CreateWireDefine(MachineRegisterInfo &MRI,
                                      unsigned BitWidth) {
  unsigned WireNum = MRI.createVirtualRegister(VTM::WireRegisterClass);
  ucOperand MO = cast<ucOperand>(MachineOperand::CreateReg(WireNum, true));
  MO.setBitWidth(BitWidth, true);
  MO.setIsEarlyClobber();
  return MO;
}

ucOperand ucOperand::CreateWireRead(unsigned WireNum, unsigned BitWidth) {
  ucOperand MO = cast<ucOperand>(MachineOperand::CreateReg(WireNum, false));
  MO.setBitWidth(BitWidth, true);
  return MO;
}

void ucOperand::print(raw_ostream &OS,
                      unsigned UB /* = 64 */, unsigned LB /* = 0 */) {
  switch (getType()) {
  case MachineOperand::MO_Register: {
    unsigned Reg = getReg();
    UB = std::min(getBitWidth(), UB);
    unsigned Offset = 0;
    if (isWire()) {
      OS << "wire" << TargetRegisterInfo::virtReg2Index(Reg);
    } else {
      //assert(TargetRegisterInfo::isPhysicalRegister(Reg)
      //       && "Unexpected virtual register!");
      // Get the one of the 64 bit registers.
      OS << "/*reg" << Reg <<"*/ reg" << (Reg & ~0x7);
      // Select the sub register
      Offset = (Reg & 0x7) * 8;
    }
    // If the operand is a wire and has only 1 bit, do not print the bit
    // range.
    OS << verilogBitRange(UB + Offset, LB + Offset, !isWire());
    return;
  }
  case MachineOperand::MO_Immediate:
    OS << verilogConstToStr(getImm(), getBitWidth(), false);
    return;
  case MachineOperand::MO_ExternalSymbol:
    OS << getSymbolName();
    return;
  default:
    MachineOperand::print(OS);
  }
}

raw_ostream &llvm::printVMBB(raw_ostream &OS, const MachineBasicBlock &MBB) {
  OS << "Scheduled MBB: " << MBB.getName() << '\n';
  for (MachineBasicBlock::const_iterator I = MBB.begin(), E = MBB.end();
       I != E; ++I) {
    const MachineInstr *Instr = I;
    switch (Instr->getOpcode()) {
    case VTM::Control:
    case VTM::Datapath:
      ucState(*Instr).print(OS);
      break;
    default:
      OS << "MI: ";
      Instr->print(OS);
      break;
    }
  }
  OS << '\n';
  return OS;
}

raw_ostream &llvm::printVMF(raw_ostream &OS, const MachineFunction &MF) {
  OS << "Scheduled MF: " << MF.getFunction()->getName() << '\n';

  for (MachineFunction::const_iterator I = MF.begin(), E = MF.end();
       I != E; ++I)
    printVMBB(OS, *I);

  return OS;
}
