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
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/SmallPtrSet.h"
#include "llvm/ADT/StringExtras.h"

namespace llvm {
class ucOpIterator;
class ucState;

class MetaToken {
protected:
  const MDNode *TokenNode;

  StringRef getStringField(unsigned Elt) const;
  unsigned getUnsignedField(unsigned Elt) const {
    return (unsigned)getUInt64Field(Elt);
  }
  uint64_t getUInt64Field(unsigned Elt) const;

public:
  enum TokenType {
    // Define a wire, defwire wire_num, source_value_token
    tokenDefWire,
    // Read a wire, readwire wire_num
    tokenReadWire,
    // Excute an instruction, instr ResourceType, ResourceID, operands ...
    tokenInstr
  };

  explicit MetaToken() : TokenNode(0) {}
  explicit MetaToken(const MDNode *N) : TokenNode(N) {}

  bool Verify() const { return TokenNode != 0; }

  operator MDNode *() const { return const_cast<MDNode*>(TokenNode); }
  MDNode *operator ->() const { return const_cast<MDNode*>(TokenNode); }

  unsigned getTag() const {  return getUnsignedField(0); }

  bool isDefWire() const;
  bool isReadWire() const;
  bool isInstr() const;

  uint64_t getWireNum() const {
    assert((isDefWire() || isReadWire()) && "Bad token type!");
    return getUInt64Field(1);
  }

  std::string getWireName(const std::string &Prefix) const {
    return Prefix + "_wire" + utostr(getWireNum());
  }

  uint64_t getBitWidth() const {
    assert((isDefWire() || isReadWire()) && "Bad token type!");
    return getUInt64Field(2);
  }

  unsigned getOpcode() const {
    assert(isInstr() && "Bad token type!");
    return getUInt64Field(3);
  }
  unsigned getPredSlot() const { 
    assert(isInstr() && "Bad token type!");
    return getUnsignedField(1);
  }

  FuncUnitId getFUId() const {
    assert(isInstr() && "Bad token type!");
    return FuncUnitId(getUInt64Field(2));
  }

  StringRef getInstrName() const {
    return getStringField(4);
  }

  void print(raw_ostream &OS) const;
  void dump() const;

  // Out of line virtual function to provide home for the class.
  virtual void anchor();

  // Helper functions to build meta operand and meta opcode.
  static MDNode *createDefWire(uint64_t WireNum, unsigned BitWidth,
                               LLVMContext &Context);

  static MDNode *createReadWire(uint64_t WireNum, unsigned BitWidth,
                                LLVMContext &Context);

  static MDNode *createInstr(unsigned PredSlot, const MachineInstr &Instr,
                             FuncUnitId FUId, LLVMContext &Context) {
    return createInstr(PredSlot, Instr.getDesc(), Context, FUId);
  }

  static const unsigned GeneralSlot = 0;
  static MDNode *createInstr(unsigned PredSlot, const TargetInstrDesc &TID,
                             LLVMContext &Context,
                             FuncUnitId FUId = VFUs::Trivial);
};

class ucOp {
public:
  typedef MachineInstr::mop_iterator op_iterator;
private:
  MetaToken Token;
  // iterator op begin and op end.
  op_iterator rangeBegin, rangeEnd;

  // op begin and op end
  ucOp(op_iterator range_begin, op_iterator range_end)
    : Token((*range_begin).getMetadata()), 
    rangeBegin(range_begin + 1), rangeEnd(range_end) {
      assert((Token.isInstr()) && "Bad leading token!");
  }
  
  friend class ucOpIterator;
public:
  op_iterator op_begin() const { return rangeBegin; }
  op_iterator op_end() const { return rangeEnd; }

  MachineOperand &getOperand(unsigned i) const {
    op_iterator I = op_begin() + i;
    assert(I < rangeEnd && "index out of range!");
    return *I;
  }

  inline unsigned getOpCode() const {
    return Token.getOpcode();
  }

  const MetaToken *operator->() const { return &Token; }

  FuncUnitId getFUId() const {
    assert(Token.isInstr() && "Bad token type!");
    return Token.getFUId();
  }

  void print(raw_ostream &OS) const;
  void dump() const;

  // Out of line virtual function to provide home for the class.
  virtual void anchor();
};


static inline raw_ostream &operator<<(raw_ostream &O, const ucOp &Op) {
  Op.print(O);
  return O;
}

class ucOpIterator : public std::iterator<std::forward_iterator_tag,
                                             ucOp, ptrdiff_t> {
  MachineInstr::mop_iterator CurIt, EndIt;

  MachineInstr::mop_iterator getNextIt() const;

  /// Create the begin iterator from a machine instruction.
  inline ucOpIterator(MachineInstr &MI)
    : CurIt(MI.operands_begin() + 1), EndIt(MI.operands_end()){
    assert(MI.getOperand(0).isImm() && "Bad bundle!");
  }

  /// Create the begin iterator from a machine instruction.
  inline ucOpIterator(MachineInstr &MI, bool) : CurIt(MI.operands_end()),
    EndIt(MI.operands_end()) {
    assert(MI.getOperand(0).isImm() && "Bad bundle!");
  }

  friend class ucState;
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

  // Out of line virtual function to provide home for the class.
  virtual void anchor();
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

  void print(raw_ostream &OS) const;
  void dump() const;

  // Out of line virtual function to provide home for the class.
  virtual void anchor();
};

// Print the scheduled machine code of verilog target machine, which only
// contains VTM::Control and VTM::Datapath.
raw_ostream &printVMBB(raw_ostream &OS, const MachineBasicBlock &MBB);
raw_ostream &printVMF(raw_ostream &OS, const MachineFunction &MF);

}

#endif
