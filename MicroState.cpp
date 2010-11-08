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

#include "MicroState.h"
#include "VTM.h"

#include "llvm/Metadata.h"
#include "llvm/Constants.h"
#include "llvm/ADT/PointerIntPair.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/Debug.h"

using namespace llvm;

uint64_t BundleToken::getUInt64Field(unsigned Elt) const {
  if (TokenNode == 0)
    return 0;

  if (Elt < TokenNode->getNumOperands())
    if (ConstantInt *CI = dyn_cast<ConstantInt>(TokenNode->getOperand(Elt)))
      return CI->getZExtValue();

  return 0;
}

bool BundleToken::isDefWire() const {
  if (!TokenNode) return false;

  return getTag() == BundleToken::tokenDefWire;
}

bool BundleToken::isReadWire() const {
  if (!TokenNode) return false;

  return getTag() == BundleToken::tokenReadWire;
}

bool BundleToken::isDefReg() const {
  if (!TokenNode) return false;

  return getTag() == BundleToken::tokenWriteReg;
}

bool BundleToken::isInstr() const {
  if (!TokenNode) return false;

  return getTag() == BundleToken::tokenInstr;
}

void BundleToken::print(raw_ostream &OS) const {
  switch (getTag()) {
  case BundleToken::tokenDefWire:
    OS << "wire" << getWireNum() << "[" << getBitWidth() <<"]";
    break;
  case BundleToken::tokenReadWire:
    OS << "wire" << getWireNum();
    break;
  case BundleToken::tokenWriteReg:
    OS << "wire" << getWireNum() << " ->";
    break;
  case BundleToken::tokenInstr:
    OS << "Instr" << getOpcode() << "{" << getResType() << "}";
    break;
  }
}

void BundleToken::dump() const {
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
      BundleToken(MOP.getMetadata()).print(OS);
    else
      MOP.print(OS);
    OS << ", ";
  }
}

void ucOp::dump() const {
  print(dbgs());
  dbgs() << '\n';
}

bool ucOp::haveDataPath() const {
  if (Token.isDefReg()) return false;
  
  switch (Token.getOpcode()) {
  case VTM::VOpArgi8: case VTM::VOpArgi16: case VTM::VOpArgi32:
  case VTM::VOpArgi64:
  case VTM::VOpRetVali8: case VTM::VOpRetVali16: case VTM::VOpRetVali32:
  case VTM::VOpRetVali64:
  case VTM::VOpRet:
    return false;
  }

  return true;
}

MachineInstr::mop_iterator ucOpIterator::getNextIt() const {
  MachineInstr::mop_iterator NextIt = CurIt;

  assert((BundleToken((*NextIt).getMetadata()).isDefReg()
          || BundleToken((*NextIt).getMetadata()).isInstr())
         && "Bad leading token");

  while (++NextIt != EndIt) {
    MachineOperand &TokenOp = *NextIt;
    if (!TokenOp.isMetadata())
      continue;

    BundleToken Token(TokenOp.getMetadata());

    // We found the begin of next range.
    if (Token.isDefReg() || Token.isInstr()) break;
  }

  return NextIt;
}
