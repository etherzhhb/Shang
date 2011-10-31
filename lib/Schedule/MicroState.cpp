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

bool ucOp::isControl() const {
  return OpCode.getParent()->getOpcode() == VTM::Control;
}
void ucOp::printOpcode(raw_ostream &OS) const {
  OS << OpCode.getDesc().getName();

  OS << "{" << OpCode.getFUId() << "} ";
}

void ucOp::print(raw_ostream &OS) const {
  OS << "@" << OpCode.getPredSlot() << " ";
  if (isControl()) {
    if (getPredicate().isPredicateInverted())
      OS << "~";
    getPredicate().print(OS, 1);
    OS << "? ";
  }

  if (OpCode.getOpcode() == VTM::IMPLICIT_DEF) {
    OS << "Implicit_def " << *op_begin();
    return;
  }

  bool isFirstUse = true;
  // Print the operands;
  for (op_iterator I = op_begin(), E = op_end(); I != E; ++I) {
    ucOperand &MOP = *I;
    if ((!MOP.isReg() || !MOP.isDef()) && isFirstUse) {
      if (I != op_begin()) OS << " = ";
      printOpcode(OS);
      isFirstUse = false;
    }

    MOP.print(OS);
    OS << ", ";
  }

  if (isFirstUse) printOpcode(OS);
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


const TargetInstrDesc &ucOperand::getDesc() const {
  assert(isOpcode() && "getDesc on a wrong operand type!");
  const TargetMachine &TM = getParent()->getParent()->getParent()->getTarget();
  return TM.getInstrInfo()->get(getOpcode());
}

ucOp ucOperand::getucParent() {
  MachineInstr *MI = getParent();

  bool OpLocated = false;
  MachineInstr::mop_iterator IStart = MI->operands_begin(),
                             IEnd = MI->operands_end();
  for (MachineInstr::mop_iterator I = MI->operands_begin(),
       E = MI->operands_end(); I != E; ++I) {
    ucOperand &Op = cast<ucOperand>(*I);
    if (Op.isOpcode()) {
      if (!OpLocated) IStart = I;
      else {
        IEnd = I;
        break;
      }
    } else
      OpLocated = (&Op == this);
  }

  assert(OpLocated && "Broken MI found!");
  return ucOp(ucOp::op_iterator(IStart, ucOperand::Mapper()),
              ucOp::op_iterator(IEnd, ucOperand::Mapper()));
}

void ucOperand::changeOpcode(unsigned Opcode, unsigned PredSlot,
                             FuncUnitId FUId /* = VFUs::Trivial */) {
  uint64_t Context = 0x0;
  Context |= (uint64_t(Opcode & OpcodeMask) << OpcodeShiftAmount);
  Context |= (uint64_t(PredSlot & PredSlotMask) << PredSlotShiftAmount);
  Context |= (uint64_t(FUId.getData() & FUIDMask) << FUIDShiftAmount);
  setImm(Context);
  setTargetFlags(IsOpcode);
}

bool ucOperand::isPredicateInverted() const {
  return getTargetFlags() & VInstrInfo::PredInvertFlag;
}

ucOperand ucOperand::CreateOpcode(unsigned Opcode, unsigned PredSlot,
                                  FuncUnitId FUId /*= VFUs::Trivial*/) {
  ucOperand MO = MachineOperand::CreateImm(0);
  MO.changeOpcode(Opcode, PredSlot, FUId);
  assert((MO.getOpcode() == Opcode && MO.getPredSlot() == PredSlot
          && MO.getFUId() == FUId) && "Data overflow!");
  return MO;
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
    OS << "(`gv" << getGlobal()->getNameStr() << " + "
       << verilogConstToStr(getOffset(), getBitWidth(), false) << ')';
    return;
  default: break;
  }

  MachineOperand::print(OS);
}

raw_ostream &llvm::printVMBB(raw_ostream &OS, const MachineBasicBlock &MBB) {
  OS << "Scheduled BB#" << MBB.getNumber() << ": " << MBB.getName() << '\n';

  if (!MBB.pred_empty()) {
    OS << "    Predecessors according to CFG:";
    for (MachineBasicBlock::const_pred_iterator PI = MBB.pred_begin(),
         E = MBB.pred_end(); PI != E; ++PI)
      OS << " BB#" << (*PI)->getNumber();
    OS << '\n';
  }

  for (MachineBasicBlock::const_iterator I = MBB.begin(), E = MBB.end();
       I != E; ++I) {
    const MachineInstr *Instr = I;
    switch (Instr->getOpcode()) {
    case VTM::Control:
    case VTM::Datapath:
      ucState(*Instr).print(OS);
      break;
    default:
      OS << "OI: ";
      Instr->print(OS);
      break;
    }
  }

  // Print the successors of this block according to the CFG.
  if (!MBB.succ_empty()) {
    OS << "    Successors according to CFG:";
    for (MachineBasicBlock::const_succ_iterator SI = MBB.succ_begin(),
         E = MBB.succ_end(); SI != E; ++SI)
      OS << " BB#" << (*SI)->getNumber();
    OS << '\n';
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
