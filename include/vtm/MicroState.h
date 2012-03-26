//===----- BundleTokens.h - Tokens for operation in a FSM state  -*- C++ -*-===//
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


#ifndef BUNDLE_TOKENS_H
#define BUNDLE_TOKENS_H

#include "vtm/VerilgoBackendMCTargetDesc.h"
#include "vtm/FUInfo.h"
#include "vtm/VInstrInfo.h"

#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/SmallPtrSet.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/ADT/STLExtras.h"

namespace llvm {
class MCInstrDesc;
class ucOpIterator;
class ucState;
class ucOp;
struct ucOpExpressionTrait;

//uc Operand
class ucOperand : public MachineOperand {
  static const unsigned BitwidthMask = 0x7f;
  static const unsigned IsWireFlag = 0x80;

  static const unsigned IsOpcode = 0x80;
  static const unsigned FUIDMask = 0xffff;
  static const unsigned FUIDShiftAmount = 0x0;
  static const unsigned PredSlotMask = 0xffff;
  static const unsigned PredSlotShiftAmount = 0x10;
  static const unsigned OpcodeMask = 0xffff;
  static const unsigned OpcodeShiftAmount = 0x20;

public:
  /*implicit*/ ucOperand(const MachineOperand &O) : MachineOperand(O) {}
  ucOperand() : MachineOperand(MachineOperand::CreateReg(0, false)) {}

  static bool classof(const MachineOperand *) { return true; }

  bool isWire() const;

  unsigned getBitWidthOrZero() const {
    assert((isImm() || isReg() || isSymbol() || isGlobal())
      && "Unsupported operand type!");
    return getTargetFlags() & BitwidthMask;
  }

  unsigned getBitWidth() const {
    unsigned BitWidth = getBitWidthOrZero();
    assert(BitWidth && "Bit width information not available!");
    return BitWidth;
  }

  typedef std::pair<unsigned, unsigned> bit_range;
  bit_range getBitRange() const;

  void setIsWire(bool isWire = true) {
    unsigned char TF = getTargetFlags();
    TF = isWire ? (TF | IsWireFlag) : (TF & ~IsWireFlag);
    setTargetFlags(TF);
  }

  void setBitWidth(unsigned BitWidth) {
    unsigned TF = getTargetFlags();
    TF &= ~BitwidthMask;
    TF |= BitWidth & BitwidthMask;
    setTargetFlags(TF);
    assert(getBitWidthOrZero() == BitWidth && "Bit width overflow!");
  }

  bool isPredicateInverted() const;

  static ucOperand CreateWire(unsigned WireNum, unsigned BitWidth,
                              bool IsDef = false);
  static ucOperand CreateReg(unsigned RegNum, unsigned BitWidth,
                             bool IsDef = false);
  static ucOperand CreateImm(int64_t Val, unsigned BitWidth);

  static ucOperand CreatePredicate(unsigned Reg = 0);

  static MachineOperand CreateTrace(MachineBasicBlock *MBB);

  /*FIXME: Get the value from the max word length*/
  void print(raw_ostream &OS, unsigned UB = 64, unsigned LB = 0,
             bool isPredicate = false);

  struct Mapper {
    typedef ucOperand &result_type;

    ucOperand &operator()(MachineOperand &Op) const {
      return cast<ucOperand>(Op);
    }
  };
};

//ucOperandExpressionTrait - Special DenseMapInfo traits to compare
//ucOperand* by *value* of the instruction rather than by pointer value.
//The hashing and equality testing functions ignore definitions so this is
//useful for CSE, etc.
struct ucOperandValueTrait : DenseMapInfo<ucOperand> {
  static inline ucOperand getEmptyKey() {
    return ucOperand::CreateReg(0, 0);
  }

  static inline ucOperand getTombstoneKey() {
    return ucOperand::CreateReg(0, 1);
  }

  static unsigned getHashValue(ucOperand Op);

  static bool isEqual(ucOperand LHS, ucOperand RHS) {
    if (RHS.isIdenticalTo(getEmptyKey())
      || RHS.isIdenticalTo(getTombstoneKey())
      || LHS.isIdenticalTo(getEmptyKey())
      || LHS.isIdenticalTo(getTombstoneKey()))
      return LHS.isIdenticalTo(RHS);

    return LHS.isIdenticalTo(RHS);
  }
};

static MachineInstr *getBundleHead(MachineInstr *MI) {
  assert(MI->isBundled() && "Not a bundle!");
  MachineInstr *Head = MI->getBundleStart();
  assert((Head->getOpcode() == VTM::CtrlStart ||
          Head->getOpcode() == VTM::Datapath) && "Broken bundle found!");
  return Head;
}

static inline unsigned getBundleSlot(MachineInstr *MI) {
  return getBundleHead(MI)->getOperand(0).getImm();
}

static inline unsigned getInstrSlotNum(MachineInstr *MI) {
  assert(MI->isInsideBundle() && "Cannot get InstrSlot!");
  return VInstrInfo::getPredOperand(MI)[1].getImm();
}

static inline bool isCtrlBundle(MachineInstr *MI) {
  return getBundleHead(MI)->getOpcode() == VTM::CtrlStart;
}

static inline bool isDatapathBundle(MachineInstr *MI) {
  return getBundleHead(MI)->getOpcode() == VTM::Datapath;
}

static inline
MachineBasicBlock::instr_iterator getCtrlBundleEnd(MachineInstr *MI) {
  assert(MI->getOpcode() == VTM::CtrlStart && "Bad MI!");
  MachineBasicBlock::instr_iterator I = MI;
  do {
    ++I;
    assert(I != I->getParent()->instr_end() && "Broken bundle found!");
  } while (I->getOpcode() != VTM::CtrlEnd);

  return I;
}
}

#endif
