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

bool MetaToken::isDefWire() const {
  if (!TokenNode) return false;

  return getTag() == MetaToken::tokenDefWire;
}

bool MetaToken::isReadWire() const {
  if (!TokenNode) return false;

  return getTag() == MetaToken::tokenReadWire;
}

bool MetaToken::isDefReg() const {
  if (!TokenNode) return false;

  return getTag() == MetaToken::tokenWriteReg;
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
  case MetaToken::tokenWriteReg:
    OS << "wire" << getWireNum() << " ->";
    break;
  case MetaToken::tokenInstr:
    OS << "Instr" << getOpcode() << "{" << getFUType() << "}";
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

  assert((MetaToken((*NextIt).getMetadata()).isDefReg()
          || MetaToken((*NextIt).getMetadata()).isInstr())
         && "Bad leading token");

  while (++NextIt != EndIt) {
    MachineOperand &TokenOp = *NextIt;
    if (!TokenOp.isMetadata())
      continue;

    MetaToken Token(TokenOp.getMetadata());

    // We found the begin of next range.
    if (Token.isDefReg() || Token.isInstr()) break;
  }

  return NextIt;
}


static Constant *getOpId(LLVMContext &Context, unsigned OpId) {
  return ConstantInt::get(Type::getInt8Ty(Context), OpId);
}

static Constant *getTagConstant(unsigned TAG, LLVMContext &Context) {
  return ConstantInt::get(Type::getInt8Ty(Context), TAG);
}

MDNode *MetaToken::createDefReg(unsigned OpId, uint64_t WireNum, 
                                LLVMContext &Context) {
  Value *Elts[] = {
    getTagConstant(MetaToken::tokenWriteReg, Context), getOpId(Context, OpId),
    ConstantInt::get(Type::getInt32Ty(Context), WireNum)
  };

  return MDNode::get(Context, Elts, array_lengthof(Elts));
}

MDNode *MetaToken::createInstr(unsigned OpId, const MachineInstr &Instr,
                               unsigned FUId, LLVMContext &Context) {
  VTFInfo VTID(Instr);

  Value *Elts[] = {
    getTagConstant(MetaToken::tokenInstr, Context), getOpId(Context, OpId),
    ConstantInt::get(Type::getInt32Ty(Context), VTID.getFUType()),
    ConstantInt::get(Type::getInt32Ty(Context), VTID->getOpcode()),
    ConstantInt::get(Type::getInt32Ty(Context), FUId)
  };

  return MDNode::get(Context, Elts, array_lengthof(Elts));
}

MDNode * llvm::MetaToken::createReadWire(uint64_t WireNum,
                                         LLVMContext &Context) {
  assert(WireNum != 0 && "Bad wire number!");
  Value *Elts[] = {
    getTagConstant(MetaToken::tokenReadWire, Context),
    ConstantInt::get(Type::getInt32Ty(Context), WireNum)
  };

  return MDNode::get(Context, Elts, array_lengthof(Elts));
}

MDNode * llvm::MetaToken::createDefWire(uint64_t WireNum, unsigned BitWidth,
                                        LLVMContext &Context) {
  Value *Elts[] = {
    getTagConstant(MetaToken::tokenDefWire, Context),
    ConstantInt::get(Type::getInt32Ty(Context), WireNum),
    ConstantInt::get(Type::getInt8Ty(Context), BitWidth)
  };

  return MDNode::get(Context, Elts, array_lengthof(Elts));
}
