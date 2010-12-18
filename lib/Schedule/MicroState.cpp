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
#include "vtm/VInstrInfo.h"
#include "vtm/VTM.h"

#include "llvm/Metadata.h"
#include "llvm/Type.h"
#include "llvm/Constants.h"
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

bool MetaToken::isDefWire() const {
  if (!TokenNode) return false;

  return getTag() == MetaToken::tokenDefWire;
}

bool MetaToken::isReadWire() const {
  if (!TokenNode) return false;

  return getTag() == MetaToken::tokenReadWire;
}

bool MetaToken::isInstr() const {
  if (!TokenNode) return false;

  return getTag() == MetaToken::tokenInstr;
}

void MetaToken::print(raw_ostream &OS) const {
  switch (getTag()) {
  case MetaToken::tokenDefWire:
    OS << "wire" << getWireNum() << "[" << getBitWidth() <<"]";
    break;
  case MetaToken::tokenReadWire:
    OS << "wire" << getWireNum();
    break;
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
  return ConstantInt::get(Type::getInt8Ty(Context), PredSlot);
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

MDNode * llvm::MetaToken::createReadWire(uint64_t WireNum,
                                         LLVMContext &Context) {
  assert(WireNum != 0 && "Bad wire number!");
  Value *Elts[] = {
    createTagConstant(MetaToken::tokenReadWire, Context),
    ConstantInt::get(Type::getInt32Ty(Context), WireNum)
  };

  return MDNode::get(Context, Elts, array_lengthof(Elts));
}

MDNode *MetaToken::createDefWire(uint64_t WireNum, unsigned BitWidth,
                                        LLVMContext &Context) {
  Value *Elts[] = {
    createTagConstant(MetaToken::tokenDefWire, Context),
    ConstantInt::get(Type::getInt32Ty(Context), WireNum),
    ConstantInt::get(Type::getInt8Ty(Context), BitWidth)
  };

  return MDNode::get(Context, Elts, array_lengthof(Elts));
}

void ucState::print(raw_ostream &OS) const {
  switch (Instr.getOpcode()) {
  case VTM::Control:
    OS << "Control ";
    break;
  case VTM::Datapath:
    OS << "Datapath ";
    break;
  case VTM::Terminator:
    OS << "Terminator ";
    break;
  default:
    assert(0 && "Bad opcode!");
    return;
  }

  OS << '@' << getSlot() << '\n';
  for (ucState::iterator I = begin(), E = end(); I != E; ++I) {
    (*I).print(OS);
    OS << '\n';
  }
}

void ucState::dump() const {
  print(dbgs());
}
