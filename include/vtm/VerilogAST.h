//===------------- VLang.h - Verilog HDL writing engine ---------*- C++ -*-===//
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
// The VLang provide funtions to complete common Verilog HDL writing task.
//
//===----------------------------------------------------------------------===//
#ifndef VBE_VLANG_H
#define VBE_VLANG_H

#include "vtm/Utilities.h"
#include "vtm/VerilgoBackendMCTargetDesc.h"
#include "vtm/FUInfo.h"
#include "vtm/LangSteam.h"

#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/StringMap.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/FoldingSet.h"
#include "llvm/Support/Allocator.h"

#include <map>

namespace llvm {
class MachineBasicBlock;
class ucOperand;
class VASTModule;
class VASTValue;
class VASTSlot;
class VASTWire;
class VASTExpr;
class VASTRegister;
class VASTUse;

class VASTNode {
public:
  // Leaf node type of Verilog AST.
  enum VASTTypes {
    vastImmediate,
    vastFirstValueType = vastImmediate,
    vastExpr,
    vastSymbol,
    vastFirstNamedType = vastSymbol,
    vastRegister,
    vastWire,
    vastLastNamedType = vastWire,
    vastLastValueType = vastWire,
    vastPort,
    vastSlot,

    vastModule
  };
private:
  uint8_t data[16];
protected:
  enum { SubClassDataBase = 1 };

  template<typename T, unsigned Offset>
  void d(T D) {
    *reinterpret_cast<T*>(data + Offset) = D;
  }

  template<typename T, unsigned Offset>
  T d() const {
    return Offset < array_lengthof(data) ?
           *reinterpret_cast<const T*>(data + Offset) : 0;
  }

  template<typename T>
  void ptr(T *p) { return d<T*, 8>(p); }
  template<typename T>
  T *ptr() const { return d<T*, 8>(); }

  void i64(int64_t i) { return d<int64_t, 8>(i); }
  int64_t i64() const { return d<int64_t, 8>(); }

  template<int Offset>
  void i8(int8_t i) { return d<int8_t, Offset>(i); }
  template<int Offset>
  int8_t i8() const { return d<int8_t, Offset>(); }

  template<int Offset>
  void i16(int16_t i) { return d<int16_t, (Offset & ~0x1)>(i); }
  template<int Offset>
  int16_t i16() const { return d<int16_t, (Offset & ~0x1)>(); }


  VASTNode(VASTTypes NodeT) {
    i8<0>(NodeT);
  }
  virtual void print(raw_ostream &OS) const = 0;
public:
  virtual ~VASTNode() {}

  unsigned getASTType() const { return i8<0>(); }

  void dump() const;
};

class VASTUse : public ilist_node<VASTUse> {
  VASTValue *V;
  VASTValue *User;
  friend class VASTExpr;

  friend struct ilist_sentinel_traits<VASTUse>;
  VASTUse(VASTValue *v = 0, VASTValue *user = 0);

  void operator=(const VASTUse &RHS); // DO NOT IMPLEMENT
  VASTUse(const VASTUse &RHS); // DO NOT IMPLEMENT

  friend class VASTModule;
  friend class VASTSlot;
  friend class VASTWire;
public:
  bool isInvalid() const { return V == 0; }

  void set(VASTValue *RHS) {
    assert(V == 0 && "Already using some value!");
    V = RHS;
  }
  // Set the user of this use and insert this use to use list.
  void setUser(VASTValue *User);
  // Get the user of this use.
  VASTValue *getUser() const { return User; }
  // Remove this use from use list.
  void removeFromList();

  //operator bool() const { return V != 0; }
  bool operator==(const VASTValue *RHS) const;

  bool operator!=(const VASTValue *RHS) const {
    return !operator==(RHS);
  }

  bool operator<(const VASTUse &RHS) const {
    return V < RHS.V;
  }

  // Return the underlying VASTValue.
  VASTValue *operator*() const {
    assert(!isInvalid() && "Not a valid Use!");
    return V;
  }

  operator VASTValue *() const { return operator*(); }

  VASTValue *operator->() const { return operator*(); }

  VASTValue *unwrap() const { return V; }

  // Prevent the user from being removed.
  void PinUser() const;

  unsigned getBitWidth() const ;
  void print(raw_ostream &OS) const;
};

template<>
struct ilist_traits<VASTUse> : public ilist_default_traits<VASTUse> {
  static void deleteNode(VASTUse *U) {}

  static bool inAnyList(const VASTUse *U) {
    return U->getPrev() != 0 || U->getNext() != 0;
  }
};

template<class IteratorType, class NodeType>
class VASTUseIterator : public std::iterator<std::forward_iterator_tag,
                                             NodeType*, ptrdiff_t> {
    IteratorType I;   // std::vector<MSchedGraphEdge>::iterator or const_iterator
    typedef VASTUseIterator<IteratorType, NodeType> Self;
public:
  VASTUseIterator(IteratorType i) : I(i) {}

  bool operator==(const Self RHS) const { return I == RHS.I; }
  bool operator!=(const Self RHS) const { return I != RHS.I; }

  const Self &operator=(const Self &RHS) {
    I = RHS.I;
    return *this;
  }

  NodeType* operator*() const {
    return I->getUser();
  }

  NodeType* operator->() const { return operator*(); }

  Self& operator++() {                // Preincrement
    ++I;
    return *this;
  }

  VASTUseIterator operator++(int) { // Postincrement
    VASTUseIterator tmp = *this;
    ++*this;
    return tmp;
  }

  VASTUse *get() { return I; }
};

class VASTValue : public VASTNode {
  typedef iplist<VASTUse> UseListTy;
  UseListTy UseList;
  int8_t bitwidth() const { return i8<VASTNode::SubClassDataBase>(); }
protected:
  enum { SubClassDataBase = VASTNode::SubClassDataBase + 1 };

  VASTValue(VASTTypes T, unsigned BitWidth) : VASTNode(T) {
    assert(T >= vastFirstValueType && T <= vastLastValueType
           && "Bad DeclType!");
    i8<VASTNode::SubClassDataBase>(BitWidth);
  }

  void addUseToList(VASTUse *U) { UseList.push_back(U); }
  void removeUseFromList(VASTUse *U) { UseList.remove(U); }
  friend class VASTUse;

  virtual void print(raw_ostream &OS) const;
public:
  unsigned getBitWidth() const { return bitwidth(); }

  typedef VASTUseIterator<UseListTy::iterator, VASTValue> use_iterator;
  use_iterator use_begin() { return use_iterator(UseList.begin()); }
  use_iterator use_end() { return use_iterator(UseList.end()); }

  bool use_empty() const { return UseList.empty(); }

  virtual void printAsOperand(raw_ostream &OS, unsigned UB, unsigned LB) const;

  void printAsOperand(raw_ostream &OS) const {
    printAsOperand(OS, getBitWidth(), 0);
  }

  bool replaceAllUseWith(VASTValue *To);

  typedef const VASTUse *dp_dep_it;
  static dp_dep_it dp_dep_begin(VASTValue *V);
  static dp_dep_it dp_dep_end(VASTValue *V);

  static bool is_dp_leaf(VASTValue *V) {
    return dp_dep_begin(V) == dp_dep_end(V);
  }
};

// simplify_type - Allow clients to treat VASTRValue just like VASTValues when
// using casting operators.
template<> struct simplify_type<const VASTUse> {
  typedef VASTValue *SimpleType;
  static SimpleType getSimplifiedValue(const VASTUse &Val) {
    return Val.unwrap();
  }
};

template<> struct simplify_type<VASTUse> {
  typedef VASTValue *SimpleType;
  static SimpleType getSimplifiedValue(const VASTUse &Val) {
    return Val.unwrap();
  }
};

class VASTImmediate : public VASTValue {
  uint64_t imm() const { return i64(); }

  VASTImmediate(uint64_t Imm, unsigned BitWidth)
    : VASTValue(vastImmediate, BitWidth) {
    i64(Imm);
  }

  friend class VASTModule;
public:
  uint64_t getValue() const { return imm(); }
  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const VASTImmediate *A) { return true; }
  static inline bool classof(const VASTNode *A) {
    return A->getASTType() == vastImmediate;
  }

  void printAsOperand(raw_ostream &OS, unsigned UB, unsigned LB) const;

  void printAsOperand(raw_ostream &OS) const {
    printAsOperand(OS, getBitWidth(), 0);
  }
};

class VASTNamedValue : public VASTValue {
protected:
  const char *name() const { return ptr<const char>(); }
  enum { SubClassDataBase = VASTValue::SubClassDataBase };

  VASTNamedValue(VASTTypes T, const char *Name, unsigned BitWidth)
    : VASTValue(T, BitWidth) {
    assert(T >= vastFirstNamedType && T <= vastLastNamedType
      && "Bad DeclType!");
    ptr<const char>(Name);
  }

public:
  const char *getName() const { return name(); }
  virtual void printAsOperand(raw_ostream &OS, unsigned UB, unsigned LB) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const VASTNamedValue *A) { return true; }
  static inline bool classof(const VASTNode *A) {
    return A->getASTType() >= vastFirstNamedType &&
           A->getASTType() <= vastLastNamedType;
  }

  void printAsOperand(raw_ostream &OS) const {
    printAsOperand(OS, getBitWidth(), 0);
  }
};

class VASTSymbol : public VASTNamedValue {
  VASTSymbol(const char *Name, unsigned BitWidth)
    : VASTNamedValue(VASTNode::vastSymbol, Name, BitWidth) {}

  friend class VASTModule;
public:
  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const VASTSymbol *A) { return true; }
  static inline bool classof(const VASTNode *A) {
    return A->getASTType() == vastSymbol;
  }
};

class VASTSignal : public VASTNamedValue {
  // TODO: Annotate the signal so we know that some of them are port signals
  // and no need to declare again in the declaration list.
  const char *AttrStr;
protected:
  enum { SubClassDataBase = VASTValue::SubClassDataBase + 2 };

  VASTSignal(VASTTypes DeclType, const char *Name, unsigned BitWidth,
             const char *Attr = "")
    : VASTNamedValue(DeclType, Name, BitWidth), AttrStr(Attr) {
    Pin(false);
    setTimingUndef(false);
  }
public:

  // Pin the wire, prevent it from being remove.
  void Pin(bool isPinned = true) {
    i8<VASTValue::SubClassDataBase + 1>(isPinned);
  }
  bool isPinned() const {
    return i8<VASTValue::SubClassDataBase>();
  }

  // The timing of a node may not captured by schedule information.
  void setTimingUndef(bool UnDef = true) {
    return i8<VASTValue::SubClassDataBase + 1>(UnDef);
  }
  bool isTimingUndef() const {
    return i8<VASTValue::SubClassDataBase + 1>();
  }

  void printDecl(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const VASTSignal *A) { return true; }
  static inline bool classof(const VASTExpr *A) { return true; }
  static inline bool classof(const VASTRegister *A) { return true; }
  static inline bool classof(const VASTNode *A) {
    return A->getASTType() == vastWire || A->getASTType() == vastRegister;
  }
};

class VASTPort : public VASTNode {
  void s(VASTSignal *S) { ptr<VASTSignal>(S); }
  VASTSignal *s() const { return ptr<VASTSignal>(); }
  void is_input(bool isInput) { i8<VASTNode::SubClassDataBase>(isInput); }
  int8_t is_input() const { return i8<VASTNode::SubClassDataBase>(); }
public:
  VASTPort(VASTSignal *S, bool isInput) : VASTNode(vastPort) {
    assert(!(isInput && isa<VASTRegister>(S)) && "Bad port decl!");
    s(S);
    is_input(isInput);
  }

  const char *getName() const { return s()->getName(); }
  bool isRegister() const { return isa<VASTRegister>(s()); }
  unsigned getBitWidth() const { return s()->getBitWidth(); }
  VASTSignal *get() const { return s(); }
  operator VASTSignal *() const { return s(); }

  bool isInput() const { return is_input(); }

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const VASTPort *A) { return true; }
  static inline bool classof(const VASTNode *A) {
    return A->getASTType() == vastPort;
  }

  void print(raw_ostream &OS) const;
  void printExternalDriver(raw_ostream &OS, uint64_t InitVal = 0) const;
  std::string getExternalDriverStr(unsigned InitVal = 0) const;

  // Out of line virtual function to provide home for the class.
  virtual void anchor();
};

// simplify_type - Allow clients to treat VASTRValue just like VASTValues when
// using casting operators.
template<> struct simplify_type<const VASTPort> {
  typedef VASTSignal *SimpleType;
  static SimpleType getSimplifiedValue(const VASTPort &Val) {
    return Val.get();
  }
};

template<> struct simplify_type<VASTPort> {
  typedef VASTSignal *SimpleType;
  static SimpleType getSimplifiedValue(const VASTPort &Val) {
    return Val.get();
  }
};

template<> struct FoldingSetTrait<VASTExpr>;
class VASTExpr : public VASTValue, public FoldingSetNode {
public:
  enum Opcode {
    // FU datapath
    dpAdd,
    dpMul,
    dpShl,
    dpSRA,
    dpSRL,
    dpSCmp,
    dpUCmp,
    // bitwise logic datapath
    dpAnd,
    dpOr,
    dpXor,
    dpNot,
    dpRAnd,
    dpROr,
    dpRXor,
    // bit level assignment.
    dpBitCat,
    dpBitRepeat,
    // Simple wire assignment.
    dpAssign,
    // VAST specific nodes.
    Dead,
    // Mux in datapath.
    dpMux,
    // Blackbox,
    dpBlackBox
  };
private:
  void ops(VASTUse *Ops) { ptr<VASTUse>(Ops); }
  VASTUse *ops() const { return ptr<VASTUse>(); }
  void num_ops(uint8_t N) { i8<VASTValue::SubClassDataBase>(N); }
  uint8_t num_ops() const { return i8<VASTValue::SubClassDataBase>(); }
  uint8_t opc() const { return i8<VASTValue::SubClassDataBase + 1>(); }
  void opc(uint8_t C) { return i8<VASTValue::SubClassDataBase + 1>(C); }
  uint8_t ub() const { return i8<VASTValue::SubClassDataBase + 2>(); }
  void ub(uint8_t u) { return i8<VASTValue::SubClassDataBase + 2>(u); }
  uint8_t lb() const { return i8<VASTValue::SubClassDataBase + 3>(); }
  void lb(uint8_t l) { return i8<VASTValue::SubClassDataBase + 3>(l); }

  enum { SubClassDataBase = VASTValue::SubClassDataBase + 4 };

  friend struct FoldingSetTrait<VASTExpr>;
  /// FastID - A reference to an Interned FoldingSetNodeID for this node.
  /// The ScalarEvolution's BumpPtrAllocator holds the data.
  const FoldingSetNodeIDRef FastID;

  VASTExpr(const VASTExpr&);             // Do not implement

  explicit VASTExpr(Opcode opc, VASTUse *ops, uint8_t numOps, unsigned BitWidth,
                    const FoldingSetNodeIDRef ID);

  explicit VASTExpr(VASTUse *U, unsigned ub, unsigned lb,
                    const FoldingSetNodeIDRef ID);

  friend class VASTModule;

  void printAsOperandInteral(raw_ostream &OS) const;

  void dropOperandsFromUseList() {
    for (VASTUse *I = ops(), *E = ops() + num_ops(); I != E; ++I)
      I->removeFromList();
  }

public:
  Opcode getOpcode() const { return VASTExpr::Opcode(opc()); }
  bool isDead() const { return getOpcode() == Dead; }

  unsigned num_operands() const { return num_ops(); }

  VASTUse &getOperand(unsigned Idx) const {
    assert(Idx < num_operands() && "Index out of range!");
    return ops()[Idx];
  }

  typedef const VASTUse *op_iterator;
  op_iterator op_begin() const { return ops(); }
  op_iterator op_end() const { return ops() + num_ops(); }

  //typedef VASTUse *op_iterator;
  //op_iterator op_begin() const { return ops(); }
  //op_iterator op_end() const { return ops() + num_ops(); }

  ArrayRef<VASTUse> getOperands() const {
    return ArrayRef<VASTUse>(ops(), num_ops());
  }

  uint8_t getUB() const { return ub(); }
  uint8_t getLB() const { return lb(); }

  void printAsOperand(raw_ostream &OS, unsigned UB, unsigned LB) const {
    assert(UB == getUB() && LB == getLB() && "Cannot print bitslice of Expr!");
    printAsOperandInteral(OS);
  }

  void printAsOperand(raw_ostream &OS) const {
    printAsOperand(OS, getUB(), getLB());
  }

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const VASTExpr *A) { return true; }
  static inline bool classof(const VASTNode *A) {
    return A->getASTType() == vastExpr;
  }
};

// Specialize FoldingSetTrait for VASTWire to avoid needing to compute
// temporary FoldingSetNodeID values.
template<>
struct FoldingSetTrait<VASTExpr> : DefaultFoldingSetTrait<VASTExpr> {
    static void Profile(const VASTExpr &X, FoldingSetNodeID& ID) {
        ID = X.FastID;
      }
    static bool Equals(const VASTExpr &X, const FoldingSetNodeID &ID,
        FoldingSetNodeID &TempID) {
            return ID == X.FastID;
        }
    static unsigned ComputeHash(const VASTExpr &X, FoldingSetNodeID &TempID) {
        return X.FastID.ComputeHash();
      }
  };

struct VASTExprBuilder {
  SmallVector<VASTValue*, 4> Operands;
  VASTExpr::Opcode Opc;
  unsigned BitWidth;
  void init(VASTExpr::Opcode opc, unsigned bitWidth) {
    Opc = opc;
    BitWidth = bitWidth;
  }

  void addOperand(VASTValue *V) { Operands.push_back(V); }
};

class VASTWire :public VASTSignal {
public:
  enum Type {
    Common,
    // Timing BlackBox, have latecy not capture by slots.
    haveExtraDelay,
    // Assignment with slot information.
    AssignCond,
    // The wire connected to an input port.
    InputPort
  };
private:
  // VASTValue pointer point to the VASTExpr.
  VASTUse U;
  uint8_t WireType() const { return i8<VASTSignal::SubClassDataBase>(); }
  void WireType(uint8_t T) { i8<VASTSignal::SubClassDataBase>(T); }

  // SlotNum and Latency share the same location.
  uint16_t Latency() const { return i16<VASTSignal::SubClassDataBase + 2>(); }
  void Latency(uint16_t L) { i16<VASTSignal::SubClassDataBase + 2>(L); }
  uint16_t SlotNum() const { return i16<VASTSignal::SubClassDataBase + 2>(); }
  void SlotNum(uint16_t S) { i16<VASTSignal::SubClassDataBase + 2>(S); }

  friend class VASTModule;

  VASTWire(const char *Name, unsigned BitWidth, const char *Attr = "")
    : VASTSignal(vastWire, Name, BitWidth, Attr), U(0, 0) {
    WireType(Common);
    Latency(0);
  }

  void setAsInput() {
    WireType(InputPort);
    // Pin the signal to prevent it from being optimized away.
    Pin();
    setTimingUndef();
  }

  void setSlot(uint16_t slotNum) {
    assert(getWireType() == VASTWire::AssignCond && "setSlot on wrong type!");
    SlotNum(slotNum);
  }

  void assign(VASTExpr *e, VASTWire::Type T = VASTWire::Common) {
    WireType(T);
    U.set(e);
    U.setUser(this);
  }

  void assignWithExtraDelay(VASTExpr *e, unsigned latency) {
    assign(e, haveExtraDelay);
    Latency(latency);
  }

  VASTValue::dp_dep_it op_begin() const { return U.isInvalid() ? 0 : &U; }
  VASTValue::dp_dep_it op_end() const { return U.isInvalid() ? 0 : &U + 1; }

  friend class VASTValue;
public:
  VASTExpr *getExpr() const {
    return dyn_cast_or_null<VASTExpr>(U.unwrap());
  }

  VASTWire::Type getWireType() const { return VASTWire::Type(WireType()); }

  unsigned getExtraDelayIfAny() const {
    return getWireType() == VASTWire::haveExtraDelay ? Latency() : 0;
  }

  uint16_t getSlotNum() const {
    assert(getWireType() == VASTWire::AssignCond &&
           "Call getSlot on bad wire type!");
    return SlotNum();
  }

  // Print the logic to the output stream.
  void print(raw_ostream &OS) const;
  void printAsOperand(raw_ostream &OS, unsigned UB, unsigned LB) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const VASTWire *A) { return true; }
  static inline bool classof(const VASTNode *A) {
    return A->getASTType() == vastWire;
  }
};

class VASTSlot : public VASTNode {
public:
  // TODO: Store the pointer to the Slot instead the slot number.
  typedef std::map<VASTSlot*, VASTUse*> SuccVecTy;
  typedef SuccVecTy::iterator succ_cnd_iterator;
  typedef SuccVecTy::const_iterator const_succ_cnd_iterator;

  // Use mapped_iterator which is a simple iterator adapter that causes a
  // function to be dereferenced whenever operator* is invoked on the iterator.
  typedef
  std::pointer_to_unary_function<std::pair<VASTSlot*, VASTUse*>, VASTSlot*>
  slot_getter;

  typedef mapped_iterator<succ_cnd_iterator, slot_getter> succ_iterator;
  typedef mapped_iterator<const_succ_cnd_iterator, slot_getter>
          const_succ_iterator;

  typedef std::map<VASTRegister*, VASTUse*> FUCtrlVecTy;
  typedef FUCtrlVecTy::const_iterator const_fu_ctrl_it;

  typedef std::map<VASTValue*, VASTUse*> FUReadyVecTy;
  typedef FUReadyVecTy::const_iterator const_fu_rdy_it;

  typedef SmallVector<VASTSlot*, 4> PredVecTy;
  typedef PredVecTy::iterator pred_it;
private:
  // The relative signal of the slot: Slot register, Slot active and Slot ready.
  VASTUse SlotReg;
  VASTUse SlotActive;
  VASTUse SlotReady;
  // The ready signals that need to wait before we go to next slot.
  FUReadyVecTy Readys;
  // The function units that enabled at this slot.
  FUCtrlVecTy Enables;
  // The function units that need to disable when condition is not satisfy.
  FUCtrlVecTy Disables;

  PredVecTy PredSlots;

  SuccVecTy NextSlots;
  uint16_t SlotNum() const { return i16<VASTNode::SubClassDataBase + 1>(); }
  void SlotNum(uint16_t S) { i16<VASTNode::SubClassDataBase + 1>(S); }
  // The start slot of parent state, can identify parent state.
  uint16_t ParentIdx() const { return i16<VASTNode::SubClassDataBase + 3>(); }
  void ParentIdx(uint16_t Idx) { i16<VASTNode::SubClassDataBase + 3>(Idx); }
  uint16_t BBNum() const { return i16<VASTNode::SubClassDataBase + 5>(); }
  void BBNum(uint16_t Num) { i16<VASTNode::SubClassDataBase + 5>(Num); }
  // Slot ranges of alias slot.
  uint16_t StartSlot() const { return i16<VASTNode::SubClassDataBase + 7>(); }
  void StartSlot(uint16_t S) { i16<VASTNode::SubClassDataBase + 7>(S); }
  uint16_t EndSlot() const { return i16<VASTNode::SubClassDataBase + 9>(); }
  void EndSlot(uint16_t S) { i16<VASTNode::SubClassDataBase + 9>(S); }
  uint16_t II() const { return i16<VASTNode::SubClassDataBase + 11>(); }
  void II(uint16_t ii) { i16<VASTNode::SubClassDataBase + 11>(ii); }

  // Successor slots of this slot.
  succ_cnd_iterator succ_cnd_begin() { return NextSlots.begin(); }
  succ_cnd_iterator succ_cnd_end() { return NextSlots.end(); }
  
  const_succ_cnd_iterator succ_cnd_begin() const { return NextSlots.begin(); }
  const_succ_cnd_iterator succ_cnd_end() const { return NextSlots.end(); }

  void addEnable(VASTRegister *R, VASTUse *Cnd);
  void addReady(VASTValue *V, VASTUse *Cnd);
  void addDisable(VASTRegister *R, VASTUse *Cnd);
  void addSuccSlot(VASTSlot *NextSlot, VASTUse *Cnd);

  friend class VASTModule;
public:
  VASTSlot(unsigned slotNum, unsigned parentIdx, VASTModule *VM);

  void buildCtrlLogic(VASTModule &Mod);
  // Print the logic of ready signal of this slot, need alias slot information.
  void buildReadyLogic(VASTModule &Mod);
  /// @briefPrint the ready expression of this slot.
  ///
  /// @param OS       The output stream.
  /// @param SrcSlot  Which Slot are the expression printing for?
  VASTValue *buildFUReadyExpr(VASTModule &VM);

  void print(raw_ostream &OS) const;

  const char *getName() const;
  // Getting the relative signals.
  VASTRegister *getRegister() const { return cast<VASTRegister>(SlotReg); }
  VASTValue *getReady() const { return *SlotReady; }
  VASTValue *getActive() const { return *SlotActive; }

  unsigned getSlotNum() const { return SlotNum(); }
  // The start slot of parent state(MachineBasicBlock)
  unsigned getParentIdx() const { return ParentIdx(); }

  // TODO: Rename to addSuccSlot.
  bool hasNextSlot(VASTSlot *NextSlot) const;
  // Dose this slot jump to some other slot conditionally instead just fall
  // through to SlotNum + 1 slot?
  bool hasExplicitNextSlots() const { return !NextSlots.empty(); }

  // Next VASTSlot iterator. 
  succ_iterator succ_begin() {
    return map_iterator(NextSlots.begin(),
                        slot_getter(pair_first<VASTSlot*, VASTUse*>));
  }

  const_succ_iterator succ_begin() const {
    return map_iterator(NextSlots.begin(),
                        slot_getter(pair_first<VASTSlot*, VASTUse*>));
  }

  succ_iterator succ_end() {
    return map_iterator(NextSlots.end(),
                        slot_getter(pair_first<VASTSlot*, VASTUse*>));
  }

  const_succ_iterator succ_end() const {
    return map_iterator(NextSlots.end(),
                        slot_getter(pair_first<VASTSlot*, VASTUse*>));
  }

  // Predecessor slots of this slot.
  pred_it pred_begin() { return PredSlots.begin(); }
  pred_it pred_end() { return PredSlots.end(); }

  // Signals need to be enabled at this slot.
  bool isEnabled(VASTRegister *R) const { return Enables.count(R); }
  const_fu_ctrl_it enable_begin() const { return Enables.begin(); }
  const_fu_ctrl_it enable_end() const { return Enables.end(); }

  // Signals need to set before this slot is ready.
  bool readyEmpty() const { return Readys.empty(); }
  const_fu_rdy_it ready_begin() const { return Readys.begin(); }
  const_fu_rdy_it ready_end() const { return Readys.end(); }

  // Signals need to be disabled at this slot.
  bool isDiabled(VASTRegister *R) const { return Disables.count(R); }
  bool disableEmpty() const { return Disables.empty(); }
  const_fu_ctrl_it disable_begin() const { return Disables.begin(); }
  const_fu_ctrl_it disable_end() const { return Disables.end(); }

  // This slots alias with this slot, this happened in a pipelined loop.
  // The slots from difference stage of the loop may active at the same time,
  // and these slot called "alias".
  void setAliasSlots(unsigned startSlot, unsigned endSlot, unsigned ii) {
    StartSlot(startSlot);
    EndSlot(endSlot);
    II(ii);
  }

  // Is the current slot the first slot of its alias slots?
  bool isLeaderSlot() const { return StartSlot() == getSlotNum(); }
  // Iterates over all alias slot
  unsigned alias_start() const { return StartSlot(); }
  unsigned alias_end() const { return EndSlot(); }
  bool hasAliasSlot() const { return alias_start() != alias_end(); }
  unsigned alias_ii() const {
    assert(hasAliasSlot() && "Dont have II!");
    return II();
  }

  bool operator<(const VASTSlot &RHS) const {
    return getSlotNum() < RHS.getSlotNum();
  }
};

template<> struct GraphTraits<VASTSlot*> {
  typedef VASTSlot NodeType;
  typedef NodeType::succ_iterator ChildIteratorType;
  static NodeType *getEntryNode(NodeType* N) { return N; }
  static inline ChildIteratorType child_begin(NodeType *N) {
    return N->succ_begin();
  }
  static inline ChildIteratorType child_end(NodeType *N) {
    return N->succ_end();
  }
};

class VASTRegister : public VASTSignal {
public:
  typedef ArrayRef<VASTValue*> AndCndVec;
private:
  uint64_t InitVal;

  // the first key VASTWire is Assignment condition. The second value is
  // assignment value.
  typedef DenseMap<VASTWire*, VASTUse*> AssignMapTy;
  AssignMapTy Assigns;

  void addAssignment(VASTUse *Src, VASTWire *AssignCnd);

  friend class VASTModule;
public:
  VASTRegister(const char *Name, unsigned BitWidth, uint64_t InitVal,
               const char *Attr = "");

  void clearAssignments() {
    assert(use_empty() && "Cannot clear assignments!");
    // TODO: Release the Uses and Wires.
    Assigns.clear();
  }

  typedef AssignMapTy::const_iterator assign_itertor;
  assign_itertor assign_begin() const { return Assigns.begin(); }
  assign_itertor assign_end() const { return Assigns.end(); }

  /*VASTUse getConstantValue() const;*/

  void printAssignment(vlang_raw_ostream &OS) const;
  void printReset(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const VASTRegister *A) { return true; }
  static inline bool classof(const VASTNode *A) {
    return A->getASTType() == vastRegister;
  }

  static void printCondition(raw_ostream &OS, const VASTSlot *Slot,
                             const AndCndVec &Cnds);
};

// The class that represent Verilog modulo.
class VASTModule : public VASTNode {
public:
  typedef SmallVector<VASTPort*, 16> PortVector;
  typedef PortVector::iterator port_iterator;
  typedef PortVector::const_iterator const_port_iterator;

  typedef SmallVector<VASTWire*, 128> WireVector;
  typedef SmallVector<VASTRegister*, 128> RegisterVector;
  typedef RegisterVector::iterator reg_iterator;

  typedef std::vector<VASTSlot*> SlotVecTy;
  typedef SlotVecTy::iterator slot_iterator;
private:
  // Dirty Hack:
  // Buffers
  raw_string_ostream DataPath, ControlBlock;
  vlang_raw_ostream LangControlBlock;
  // The slots vector, each slot represent a state in the FSM of the design.
  SlotVecTy Slots;
  // Input/Output ports of the design.
  PortVector Ports;
  // Wires and Registers of the design.
  WireVector Wires;
  RegisterVector Registers;

  // Expression in data-path
  typedef FoldingSet<VASTExpr>::iterator fs_vas_it;
  FoldingSet<VASTExpr> UniqueExprs;

  typedef StringMap<VASTNamedValue*> SymTabTy;
  SymTabTy SymbolTable;
  typedef StringMapEntry<VASTNamedValue*> SymEntTy;

  typedef DenseMap<std::pair<uint64_t, char>, VASTImmediate*> UniqueImmSetTy;
  UniqueImmSetTy UniqueImms;

  // The Name of the Design.
  std::string Name;
  BumpPtrAllocator Allocator;
  SpecificBumpPtrAllocator<VASTUse> UseAllocator;

  // The port starting offset of a specific function unit.
  SmallVector<std::map<unsigned, unsigned>, VFUs::NumCommonFUs> FUPortOffsets;
  unsigned NumArgPorts, RetPortIdx;

public:
  static std::string DirectClkEnAttr, ParallelCaseAttr, FullCaseAttr;

  enum PortTypes {
    Clk = 0,
    RST,
    Start,
    SpecialInPortEnd,
    Finish = SpecialInPortEnd,
    SpecialOutPortEnd,
    NumSpecialPort = SpecialOutPortEnd,
    ArgPort, // Ports for function arguments.
    Others,   // Likely ports for function unit.
    RetPort // Port for function return value.
  };

  VASTModule(const std::string &Name) : VASTNode(vastModule),
    DataPath(*(new std::string())),
    ControlBlock(*(new std::string())),
    LangControlBlock(ControlBlock),
    Name(Name),
    FUPortOffsets(VFUs::NumCommonFUs),
    NumArgPorts(0) {
    Ports.append(NumSpecialPort, 0);
  }

  ~VASTModule();

  const std::string &getName() const { return Name; }

  void printDatapath(raw_ostream &OS) const;
  void printRegisterAssign(vlang_raw_ostream &OS) const;

  // Print the slot control flow.
  typedef DenseMap<unsigned, const MachineBasicBlock*> StartIdxMapTy;
  void buildSlotLogic(StartIdxMapTy &StartIdxMap);
  void writeProfileCounters(VASTSlot *S, StartIdxMapTy &StartIdxMap);

  bool eliminateConstRegisters();

  VASTImmediate *getOrCreateImmediate(uint64_t Value, int8_t BitWidth) {
    UniqueImmSetTy::key_type key = std::make_pair(Value, BitWidth);
    VASTImmediate *&Imm = UniqueImms.FindAndConstruct(key).second;
    if (!Imm) // Create the immediate if it is not yet created.
      Imm = new (Allocator.Allocate<VASTImmediate>()) VASTImmediate(Value, BitWidth);
    return Imm;
  }

  VASTValue *getSymbol(const std::string &Name) const {
    SymTabTy::const_iterator at = SymbolTable.find(Name);
    assert(at != SymbolTable.end() && "Symbol not found!");
    return at->second;
  }

  template<class T>
  T *getSymbol(const std::string &Name) const {
    return cast<T>(getSymbol(Name));
  }

  VASTValue *getOrCreateSymbol(const std::string &Name, unsigned BitWidth) {
    SymEntTy &Entry = SymbolTable.GetOrCreateValue(Name);
    VASTNamedValue *&V = Entry.second;
    if (V == 0) {
       V = Allocator.Allocate<VASTNamedValue>();
       new (V) VASTSymbol(Entry.getKeyData(), BitWidth);
    }

    assert(V->getBitWidth() == BitWidth
           && "Getting symbol with wrong bitwidth!");
    return V;
  }

  VASTValue *getAlwaysTrue() {
    return getOrCreateSymbol("1'b1", 0);
  }

  VASTValue *getAlwaysFalse() {
    return getOrCreateSymbol("1'b0", 0);
  }

  void allocaSlots(unsigned TotalSlots) {
    Slots.assign(TotalSlots, 0);
  }

  VASTSlot *getOrCreateSlot(unsigned SlotNum, unsigned ParentIdx) {
    VASTSlot *&Slot = Slots[SlotNum];
    if(Slot == 0) {
      Slot = Allocator.Allocate<VASTSlot>();
      new (Slot) VASTSlot(SlotNum, ParentIdx, this);
    }

    return Slot;
  }

  VASTSlot *getOrCreateNextSlot(VASTSlot *S) {
    // TODO: Check if the next slot out of bound.
    return getOrCreateSlot(S->getSlotNum() + 1, S->getParentIdx());
  }

  VASTSlot *getSlot(unsigned SlotNum) const {
    VASTSlot *S = Slots[SlotNum];
    assert(S && "Slot not exist!");
    return S;
  }

  void addSlotEnable(VASTSlot *S, VASTRegister *R, VASTValue *Cnd);
  void addSlotReady(VASTSlot *S, VASTValue *V, VASTValue *Cnd);
  void addSlotDisable(VASTSlot *S, VASTRegister *R, VASTValue *Cnd);
  void addSlotSucc(VASTSlot *S, VASTSlot *SuccS, VASTValue *V);
  // Allow user to add ports.
  VASTPort *addInputPort(const std::string &Name, unsigned BitWidth,
                         PortTypes T = Others);

  VASTPort *addOutputPort(const std::string &Name, unsigned BitWidth,
                          PortTypes T = Others, bool isReg = true);

  void setFUPortBegin(FuncUnitId ID) {
    unsigned offset = Ports.size();
    std::pair<unsigned, unsigned> mapping
      = std::make_pair(ID.getFUNum(), offset);
    std::map<unsigned, unsigned> &Map = FUPortOffsets[ID.getFUType()];
    assert(!Map.count(mapping.first) && "Port begin mapping existed!");
    FUPortOffsets[ID.getFUType()].insert(mapping);
  }

  unsigned getFUPortOf(FuncUnitId ID) const {
    typedef std::map<unsigned, unsigned> MapTy;
    const MapTy &Map = FUPortOffsets[ID.getFUType()];
    MapTy::const_iterator at = Map.find(ID.getFUNum());
    assert(at != Map.end() && "FU do not existed!");
    return at->second;
  }

  const_port_iterator getFUPortItBegin(FuncUnitId ID) const {
    unsigned PortBegin = getFUPortOf(ID);
    return Ports.begin() + PortBegin;
  }

  void printModuleDecl(raw_ostream &OS) const;

  // Get all ports of this moudle.
  const PortVector &getPorts() const { return Ports; }
  unsigned getNumPorts() const { return Ports.size(); }

  VASTPort &getPort(unsigned i) const {
    // FIXME: Check if out of range.
    return *Ports[i];
  }

  const char *getPortName(unsigned i) const {
    return getPort(i).getName();
  }

  port_iterator ports_begin() { return Ports.begin(); }
  const_port_iterator ports_begin() const { return Ports.begin(); }

  port_iterator ports_end() { return Ports.end(); }
  const_port_iterator ports_end() const { return Ports.end(); }

  // Argument ports and return port.
  const VASTPort &getArgPort(unsigned i) const {
    // FIXME: Check if out of range.
    return getPort(i + VASTModule::SpecialOutPortEnd);
  }

  unsigned getNumArgPorts() const { return NumArgPorts; }
  unsigned getRetPortIdx() const { return RetPortIdx; }
  const VASTPort &getRetPort() const {
    assert(getRetPortIdx() && "No return port in this module!");
    return getPort(getRetPortIdx());
  }

  unsigned getNumCommonPorts() const {
    return getNumPorts() - VASTModule::SpecialOutPortEnd;
  }

  const VASTPort &getCommonPort(unsigned i) const {
    // FIXME: Check if out of range.
    return getPort(i + VASTModule::SpecialOutPortEnd);
  }

  port_iterator common_ports_begin() {
    return Ports.begin() + VASTModule::SpecialOutPortEnd;
  }
  const_port_iterator common_ports_begin() const {
    return Ports.begin() + VASTModule::SpecialOutPortEnd;
  }

  VASTValue *createExpr(VASTExpr::Opcode Opc, ArrayRef<VASTValue*> Ops,
                        unsigned BitWidth);

  VASTValue *getOrCreateCommutativeExpr(VASTExpr::Opcode Opc,
                                        ArrayRef<VASTValue*> Ops,
                                        unsigned BitWidth);

  VASTValue *buildExpr(VASTExpr::Opcode Opc, ArrayRef<VASTValue*> Ops,
                       unsigned BitWidth);
  VASTValue *buildExpr(VASTExpr::Opcode Opc, VASTValue *Op, unsigned BitWidth);
  VASTValue *buildExpr(VASTExpr::Opcode Opc, VASTValue *LHS, VASTValue *RHS,
                       unsigned BitWidth);
  VASTValue *buildExpr(VASTExpr::Opcode Opc, VASTValue *Op0, VASTValue *Op1,
                       VASTValue *Op2, unsigned BitWidth);
  VASTValue *buildExpr(VASTExprBuilder &Builder) {
    return buildExpr(Builder.Opc, Builder.Operands, Builder.BitWidth);
  }

  VASTValue *getOrCreateBitSlice(VASTValue *U, uint8_t UB, uint8_t LB);

  VASTValue *buildLogicExpr(VASTExpr::Opcode Opc, VASTValue *LHS, VASTValue *RHS,
                            unsigned BitWidth);

  VASTValue *buildMulExpr(VASTExpr::Opcode Opc, ArrayRef<VASTValue*> Ops,
                          unsigned BitWidth);

  VASTValue *buildAddExpr(VASTExpr::Opcode Opc, ArrayRef<VASTValue*> Ops,
                          unsigned BitWidth);

  VASTValue *buildNotExpr(VASTValue *U);

  VASTRegister *addRegister(const std::string &Name, unsigned BitWidth,
                            unsigned InitVal = 0,
                            const char *Attr = "");

  VASTWire *addWire(const std::string &Name, unsigned BitWidth,
                    const char *Attr = "");

  reg_iterator reg_begin() { return Registers.begin(); }
  reg_iterator reg_end() { return Registers.end(); }

  slot_iterator slot_begin() { return Slots.begin(); }
  slot_iterator slot_end() { return Slots.end(); }

  void addAssignment(VASTRegister *Dst, VASTValue *Src, VASTSlot *Slot,
                     SmallVectorImpl<VASTValue*> &Cnds,
                     bool AddSlotActive = true);
  VASTWire *buildAssignCnd(VASTSlot *Slot, SmallVectorImpl<VASTValue*> &Cnds,
                           bool AddSlotActive = true);

  VASTValue *assign(VASTWire *W, VASTValue *V,
                    VASTWire::Type T = VASTWire::Common);
  VASTValue *assignWithExtraDelay(VASTWire *W, VASTValue *V, unsigned latency);

  void printSignalDecl(raw_ostream &OS);
  void printRegisterReset(raw_ostream &OS);

  void print(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const VASTModule *A) { return true; }
  static inline bool classof(const VASTNode *A) {
    return A->getASTType() == vastModule;
  }

  vlang_raw_ostream &getControlBlockBuffer() {
    return LangControlBlock;
  }

  std::string &getControlBlockStr() {
    LangControlBlock.flush();
    return ControlBlock.str();
  }

  raw_ostream &getDataPathBuffer() {
    return DataPath;
  }

  std::string &getDataPathStr() {
    return DataPath.str();
  }

  // Out of line virtual function to provide home for the class.
  virtual void anchor();

  static const std::string GetMemBusEnableName(unsigned FUNum) {
    return VFUMemBus::getEnableName(FUNum) + "_r";
  }

  static const std::string GetFinPortName() {
    return "fin";
  }
};

// Helper functions
// Traverse the use tree to get the registers.
template<typename VisitPathFunc>
void DepthFirstTraverseDepTree(VASTValue *DepTree, VisitPathFunc VisitPath) {
  typedef VASTValue::dp_dep_it ChildIt;
  // Use seperate node and iterator stack, so we can get the path vector.
  typedef SmallVector<VASTValue*, 16> NodeStackTy;
  typedef SmallVector<ChildIt, 16> ItStackTy;
  NodeStackTy NodeWorkStack;
  ItStackTy ItWorkStack;
  // Remember what we had visited.
  std::set<VASTValue*> VisitedUses;

  // Put the root.
  NodeWorkStack.push_back(DepTree);
  ItWorkStack.push_back(VASTValue::dp_dep_begin(DepTree));

  while (!ItWorkStack.empty()) {
    VASTValue *Node = NodeWorkStack.back();

    ChildIt It = ItWorkStack.back();

    // Do we reach the leaf?
    if (VASTValue::is_dp_leaf(Node)) {
      VisitPath(NodeWorkStack);
      NodeWorkStack.pop_back();
      ItWorkStack.pop_back();
      continue;
    }

    // All sources of this node is visited.
    if (It == VASTValue::dp_dep_end(Node)) {
      NodeWorkStack.pop_back();
      ItWorkStack.pop_back();
      continue;
    }

    // Depth first traverse the child of current node.
    VASTValue *ChildNode = *It;
    ++ItWorkStack.back();

    // Had we visited this node? If the Use slots are same, the same subtree
    // will lead to a same slack, and we do not need to compute the slack agian.
    if (!VisitedUses.insert(ChildNode).second) continue;

    // If ChildNode is not visit, go on visit it and its childrens.
    NodeWorkStack.push_back(ChildNode);
    ItWorkStack.push_back(VASTValue::dp_dep_begin(ChildNode));
  }
}


std::string verilogConstToStr(Constant *C);

std::string verilogConstToStr(uint64_t value,unsigned bitwidth,
                              bool isMinValue);

std::string verilogBitRange(unsigned UB, unsigned LB = 0, bool printOneBit = true);

} // end namespace

#endif
