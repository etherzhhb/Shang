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
    tokenInstr,
    // Write register, LatchVal register, source value
    tokenLatchVal = VTM::COPY
  };

  explicit MetaToken() : TokenNode(0) {}
  explicit MetaToken(const MDNode *N) : TokenNode(N) {}

  bool Verify() const { return TokenNode != 0; }

  operator MDNode *() const { return const_cast<MDNode*>(TokenNode); }
  MDNode *operator ->() const { return const_cast<MDNode*>(TokenNode); }

  unsigned getTag() const {  return getUnsignedField(0); }
  unsigned getId() const { 
    assert((isInstr() || isDefReg()) && "Bad token type!");
    return getUnsignedField(1);
  }

  bool isDefWire() const;
  bool isReadWire() const;
  bool isInstr() const;
  bool isDefReg() const;

  uint64_t getWireNum() const {
    assert((isDefWire() || isReadWire() || isDefReg()) && "Bad token type!");
    
    if (isDefReg()) return getUInt64Field(2);

    return getUInt64Field(1);
  }

  std::string getWireName(const std::string &Prefix) const {
    assert((isDefWire() || isReadWire() || isDefReg()) && "Bad token type!");
    return Prefix + "_wire" + utostr(getWireNum());
  }

  uint64_t getBitWidth() const {
    assert(isDefWire() && "Bad token type!");
    return getUInt64Field(2);
  }

  unsigned getFUType() const {
    assert(isInstr() && "Bad token type!");
    return getUInt64Field(2);
  }

  unsigned getOpcode() const {
    assert(isInstr() && "Bad token type!");
    return getUInt64Field(3);
  }

  unsigned getFUId() const {
    assert(isInstr() && "Bad token type!");
    return getUInt64Field(4);
  }

  void print(raw_ostream &OS) const;
  void dump() const;

  // Helper functions to build meta operand and meta opcode.
  static MDNode *createDefWire(uint64_t WireNum, unsigned BitWidth,
                               LLVMContext &Context);

  static MDNode *createReadWire(uint64_t WireNum, LLVMContext &Context);

  static MDNode *createInstr(unsigned OpId, const MachineInstr &Instr,
                             unsigned FUId, LLVMContext &Context);

  static MDNode *createDefReg(unsigned OpId, uint64_t WireNum,
                              LLVMContext &Context);
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
      assert((Token.isInstr() || Token.isDefReg()) && "Bad leading token!");
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
    if (Token.isInstr())
      return Token.getOpcode();
    // Since the Token must be an instr token or DefReg token, if we reach
    // here, it must be a DefReg token.
    return VTM::COPY;
  }

  std::string getSrcWireName(const std::string &Prefix) const {
    assert(Token.isDefReg() && "Bad token type");
    return Token.getWireName(Prefix);
  }

  unsigned getFUNum() const {
    assert(Token.isInstr() && "Bad token type!");
    return Token.getFUId();
  }

  void print(raw_ostream &OS) const;
  void dump() const;
};

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
};

// uc State in a FSM state.
class ucState {
  const MachineInstr &Instr;
public:
  /*implicit*/ ucState(const MachineInstr &MI) : Instr(MI) {
    assert((MI.getOpcode() == VTM::Control
            || MI.getOpcode() == VTM::Datapath
            || MI.getOpcode() == VTM::Terminator)
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
};
}

#endif
