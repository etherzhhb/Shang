//===----- BundleTokens.h - Tokens for operation in a FSM state  -*- C++ -*-===//
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


#ifndef BUNDLE_TOKENS_H
#define BUNDLE_TOKENS_H

#include "vtm/VTM.h"
#include "vtm/FUInfo.h"

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

  static bool classof(const MachineOperand *) { return true; }

  bool isOpcode() const { return isImm() && (IsOpcode & getTargetFlags()); }
  unsigned getPredSlot() const {
    assert(isOpcode() && "Bad Operand type!");
    uint64_t Context = getImm();
    return (Context >> PredSlotShiftAmount) & PredSlotMask;
  }

  unsigned getOpcode() const {
    assert(isOpcode() && "Bad Operand type!");
    uint64_t Context = getImm();
    return (Context >> OpcodeShiftAmount) & OpcodeMask;
  }

  bool isOpcode(unsigned OpCode) const {
    assert(isOpcode() && "Bad Operand type!");
    return getOpcode() == OpCode;
  }

  void changeOpcode(unsigned Opcode, unsigned PredSlot,
                    FuncUnitId FUId = VFUs::Trivial);

  void clearPredSlot();

  const MCInstrDesc &getDesc() const;

  FuncUnitId getFUId() const {
    assert(isOpcode() && "Bad Operand type!");
    uint64_t Context = getImm();
    return FuncUnitId((Context >> FUIDShiftAmount) & FUIDMask);
  }

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

  ucOp getucParent();

  static ucOperand CreateOpcode(unsigned Opcode, unsigned PredSlot,
                                FuncUnitId FUId = VFUs::Trivial);

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
    return ucOperand::CreateReg(0, false);
  }

  static inline ucOperand getTombstoneKey() {
    return ucOperand::CreateOpcode(VTM::PHI, 0);
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

class ucOp {
public:
  typedef mapped_iterator<MachineInstr::mop_iterator, ucOperand::Mapper>
          op_iterator;
private:
  // FIXME: We should use reference of ucOperand instead, but the operator= will
  // not work, which is need by DenseMap.
  ucOperand OpCode;
  // iterator op begin and op end.
  op_iterator rangeBegin, rangeEnd;

  // op begin and op end
  ucOp(op_iterator range_begin, op_iterator range_end)
    : OpCode(cast<ucOperand>(*range_begin)),
      rangeBegin(range_begin + 1), rangeEnd(range_end) {
      assert(OpCode.isOpcode() && "Bad leading token!");
    // Skip the predicate operand.
    if (isControl()) ++rangeBegin;
  }

  ucOp(uint64_t i) : OpCode(MachineOperand::CreateImm(i)),
    rangeBegin(MachineInstr::mop_iterator(), ucOperand::Mapper()),
    rangeEnd(MachineInstr::mop_iterator(), ucOperand::Mapper()) {}

  friend class ucOpIterator;
  friend class ucOperand;
  friend struct ucOpExpressionTrait;

  void printOpcode(raw_ostream &OS) const;
public:

  const ucOp& operator=(const ucOp &RHS) {
    OpCode = RHS.OpCode;
    rangeBegin = RHS.rangeBegin;
    rangeEnd = RHS.rangeEnd;
    return *this;
  }

  void changeOpcode(unsigned Opcode, unsigned PredSlot,
                    FuncUnitId FUId = VFUs::Trivial) {
    OpCode.changeOpcode(Opcode, PredSlot, FUId);
  }

  bool isControl() const;

  op_iterator op_begin() const { return rangeBegin; }

  op_iterator op_end() const { return rangeEnd; }

  size_t getNumOperands() const { return rangeEnd - rangeBegin; }

  ucOperand &getOperand(unsigned i) const {
    op_iterator I = op_begin() + i;
    assert(I < rangeEnd && "index out of range!");
    return *I;
  }

  unsigned getOpIdx(const MachineOperand &MO) const {
    assert(MO.getParent() == OpCode.getParent()
           && "Call getOpIdx with MO from other MI!");
    int Idx = const_cast<MachineOperand*>(&MO)
              - reinterpret_cast<MachineOperand*>(&*rangeBegin.getCurrent());
    // Operands place right after the opcode.
    Idx += (isControl() ? 2 : 1);
    assert(Idx > 0 && "Call getOpIdx with MO from other ucOp!");
    return Idx;
  }

  ucOperand &getPredicate() const {
    assert(isControl() && "Data path do not have predicate operand!");
    return *(op_begin() - 1);
  }

  const ucOperand *operator->() const { return &OpCode; }

  template<typename def_use_it>
  static ucOp getParent(def_use_it DI) {
    MachineInstr &DefInst = *DI;
    MachineInstr::mop_iterator OI = DefInst.operands_begin() + DI.getOperandNo();
    assert(!cast<ucOperand>(*OI).isOpcode() && "Not an operand!");

    MachineInstr::mop_iterator OpcodeI = OI;
    while (!cast<ucOperand>(*(--OpcodeI)).isOpcode())
      assert(OI != DefInst.operands_begin() && "Broken ucState!");

    MachineInstr::mop_iterator NextOpcodeI = OI, OE = DefInst.operands_end();
    while (NextOpcodeI != OE && !cast<ucOperand>(*NextOpcodeI).isOpcode())
      ++NextOpcodeI;

    return ucOp(ucOp::op_iterator(OpcodeI, ucOperand::Mapper()),
                ucOp::op_iterator(NextOpcodeI, ucOperand::Mapper()));
  }

  void print(raw_ostream &OS) const;
  void dump() const;

  /// isIdenticalTo - Return true if this instruction is identical to (same
  /// opcode and same operands as) the specified instruction.
  bool isIdenticalTo(const ucOp &Other, MachineInstr::MICheckType Check
                     = MachineInstr::IgnoreVRegDefs) const;
};

 //ucOpExpressionTrait - Special DenseMapInfo traits to compare
 //ucOp* by *value* of the instruction rather than by pointer value.
 //The hashing and equality testing functions ignore definitions so this is
 //useful for CSE, etc.
struct ucOpExpressionTrait : DenseMapInfo<ucOp> {
  static inline ucOp getEmptyKey() {
    return ucOp(0);
  }

  static inline ucOp getTombstoneKey() {
    return ucOp(-1);
  }

  static unsigned getHashValue(ucOp Op);

  static bool isEqual(ucOp LHS, ucOp RHS) {
    if (RHS.OpCode.isIdenticalTo(getEmptyKey().OpCode)
        || RHS.OpCode.isIdenticalTo(getTombstoneKey().OpCode)
        || LHS.OpCode.isIdenticalTo(getEmptyKey().OpCode)
        || LHS.OpCode.isIdenticalTo(getTombstoneKey().OpCode))
        return LHS.OpCode.isIdenticalTo(RHS.OpCode);

    return LHS.isIdenticalTo(RHS, MachineInstr::IgnoreVRegDefs);
  }
};

static inline raw_ostream &operator<<(raw_ostream &O, const ucOp &Op) {
  Op.print(O);
  return O;
}

class ucOpIterator : public std::iterator<std::forward_iterator_tag,
                                          ucOp, ptrdiff_t> {
  ucOp::op_iterator CurIt, EndIt;

  ucOp::op_iterator getNextIt() const;

  /// Create the begin iterator from a machine instruction.
  inline ucOpIterator(MachineInstr &MI)
    : CurIt(MI.operands_begin() + 2, ucOperand::Mapper()),
      EndIt(MI.operands_end(), ucOperand::Mapper()){
    assert(MI.getOperand(0).isImm() && "Bad bundle!");
  }

  /// Create the begin iterator from a machine instruction.
  inline ucOpIterator(MachineInstr &MI, bool)
    : CurIt(MI.operands_end(), ucOperand::Mapper()),
      EndIt(MI.operands_end(), ucOperand::Mapper()) {
    assert(MI.getOperand(0).isImm() && "Bad bundle!");
  }

  friend class ucState;
  friend class ucOperand;
public:
  inline bool operator==(const ucOpIterator& x) const {
    return CurIt == x.CurIt;
  }

  inline bool operator!=(const ucOpIterator& x) const {
    return !operator==(x);
  }

  inline ucOp operator*() const {
    assert(CurIt != EndIt && "Iterator out of range!");
    return ucOp(CurIt, getNextIt());
  }

  //inline ucOp *operator->() const {
  //  return &operator *();
  //}

  inline ucOpIterator& operator++() {
    assert(CurIt != EndIt && "Can not increase!");
    CurIt = getNextIt();
    return *this;
  }

  inline ucOpIterator operator++(int) {
    ucOpIterator tmp = *this;
    ++*this;
    return tmp;
  }

  inline const ucOpIterator &operator=(const ucOpIterator &I) {
    CurIt = I.CurIt;
    EndIt = I.EndIt;
    return *this;
  }
};

// uc State in a FSM state.
class ucState {
  const MachineInstr &Instr;
public:

  /*implicit*/ ucState(const MachineInstr &MI) : Instr(MI) {
    assert((MI.getOpcode() == VTM::Control
            || MI.getOpcode() == VTM::Datapath)
           && "Bad Instr!");
  }
  /*implicit*/ ucState(MachineBasicBlock::iterator I) : Instr(*I) {
    assert((I->getOpcode() == VTM::Control
      || I->getOpcode() == VTM::Datapath)
      && "Bad Instr!");
  }

  unsigned getSlot() const {
    return Instr.getOperand(0).getImm();
  }

  operator MachineInstr *() const {
    return const_cast<MachineInstr*>(&Instr);
  }

  MachineInstr *operator ->() const {
    return const_cast<MachineInstr*>(&Instr);
  }

  /// Iterator to iterate over the uc operation in a uc state.
  typedef ucOpIterator iterator;
  iterator begin() const {
    return ucOpIterator(const_cast<MachineInstr&>(Instr));
  }
  iterator end() const {
    return ucOpIterator(const_cast<MachineInstr&>(Instr), false);
  }

  bool empty() const { return Instr.getNumOperands() == 2; }

  void print(raw_ostream &OS) const;
  void dump() const;
};

// Print the scheduled machine code of verilog target machine, which only
// contains VTM::Control and VTM::Datapath.
raw_ostream &printVMBB(raw_ostream &OS, const MachineBasicBlock &MBB);
raw_ostream &printVMF(raw_ostream &OS, const MachineFunction &MF);

}

#endif
