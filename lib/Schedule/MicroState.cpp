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

uint64_t MetaToken::getUInt64Field(unsigned Elt) const {
  if (TokenNode == 0)
    return 0;

  if (Elt < TokenNode->getNumOperands())
    if (ConstantInt *CI = dyn_cast<ConstantInt>(TokenNode->getOperand(Elt)))
      return CI->getZExtValue();

  return 0;
}

StringRef MetaToken::getStringField(unsigned Elt) const {
  if (TokenNode  == 0)
    return StringRef();

  if (Elt < TokenNode ->getNumOperands())
    if (MDString *MDS = dyn_cast_or_null<MDString>(TokenNode->getOperand(Elt)))
      return MDS->getString();

  return StringRef();
}

bool MetaToken::isInstr() const {
  if (!TokenNode) return false;

  return getTag() == MetaToken::tokenInstr;
}

void MetaToken::print(raw_ostream &OS) const {
  switch (getTag()) {
  case MetaToken::tokenInstr:
    OS << getInstrName() << "{" << getFUId() << "}"
       << "@" << getPredSlot();
    break;
  }
}

void MetaToken::dump() const {
  print(dbgs());
  dbgs() << '\n';
}

void ucOp::print(raw_ostream &OS) const {
  // Print the leading token.
  Token.print(OS);
  OS << ' ';
  // Print the operands;
  for (op_iterator I = op_begin(), E = op_end(); I != E; ++I) {
    MachineOperand &MOP = *I;
    if (MOP.isMetadata())
      MetaToken(MOP.getMetadata()).print(OS);
    else
      MOP.print(OS);
    OS << ", ";
  }
}

void ucOp::dump() const {
  print(dbgs());
  dbgs() << '\n';
}

MachineInstr::mop_iterator ucOpIterator::getNextIt() const {
  MachineInstr::mop_iterator NextIt = CurIt;

  assert(MetaToken((*NextIt).getMetadata()).isInstr()
         && "Bad leading token");

  while (++NextIt != EndIt) {
    MachineOperand &TokenOp = *NextIt;
    if (!TokenOp.isMetadata())
      continue;

    MetaToken Token(TokenOp.getMetadata());

    // We found the begin of next range.
    if (Token.isInstr()) break;
  }

  return NextIt;
}

static Constant *createPredSlot(LLVMContext &Context, unsigned PredSlot) {
  return ConstantInt::get(Type::getInt32Ty(Context), PredSlot);
}

static Constant *createTagConstant(unsigned TAG, LLVMContext &Context) {
  return ConstantInt::get(Type::getInt8Ty(Context), TAG);
}

MDNode *MetaToken::createInstr(unsigned PredSlot, const TargetInstrDesc &TID,
                               LLVMContext &Context, FuncUnitId FUId) {
  Value *Elts[] = {
    createTagConstant(MetaToken::tokenInstr, Context),
    createPredSlot(Context, PredSlot),
    ConstantInt::get(Type::getInt32Ty(Context), FUId.getData()),
    ConstantInt::get(Type::getInt32Ty(Context), TID.getOpcode()),
    MDString::get(Context, TID.getName())
  };

  return MDNode::get(Context, Elts, array_lengthof(Elts));
}

// Out of line virtual function to provide home for the class.
void MetaToken::anchor() {}


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
      assert(TargetRegisterInfo::isPhysicalRegister(Reg)
             && "Unexpected virtual register!");
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
    assert(0 && "Unknown Operand type!");
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
