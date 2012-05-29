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
#include "vtm/VerilogBackendMCTargetDesc.h"
#include "vtm/FUInfo.h"
#include "vtm/LangSteam.h"

#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/StringMap.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/FoldingSet.h"
#include "llvm/ADT/PointerIntPair.h"
#include "llvm/Support/Allocator.h"

#include <map>

namespace llvm {
class MachineBasicBlock;
class MachineOperand;
class VASTModule;
template<typename T> struct PtrInvPair;
class VASTValue;
class VASTSlot;
class VASTSignal;
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
protected:
  union {
    int64_t IntVal;
    const char *Name;
    VASTSignal *Signal;
    MachineBasicBlock *ParentBB;
  } Contents;

  const uint8_t NodeT;
  explicit VASTNode(VASTTypes T) : NodeT(T) {}

  virtual void print(raw_ostream &OS) const = 0;
public:
  virtual ~VASTNode() {}

  VASTTypes getASTType() const { return VASTTypes(NodeT); }

  void dump() const;
};

template<typename T>
struct PtrInvPair : public PointerIntPair<T*, 1, bool>{
  typedef PointerIntPair<T*, 1, bool> Base;
  PtrInvPair(T *V, bool IsInvert = false)
    : PointerIntPair<T*, 1, bool>(V, IsInvert) {}

  template<typename T1>
  PtrInvPair(const PtrInvPair<T1>& RHS)
    : PointerIntPair<T*, 1, bool>(RHS.get(), RHS.isInverted()) {}

  template<typename T1>
  PtrInvPair<T> &operator=(const PtrInvPair<T1> &RHS) {
    setPointer(RHS.get());
    setInt(RHS.isInverted());
    return *this;
  }

  operator void*() const {
    return this->getOpaqueValue();
  }

  T *get() const { return this->getPointer(); }

  bool isInverted() const { return this->getInt(); }
  PtrInvPair<T> invert(bool Invert = true) const {
    return Invert ? PtrInvPair<T>(get(), !isInverted()) : *this;
  }

  T *operator->() const { return this->get(); }

  // PtrInvPairs are equal when their Opaque Value are equal, which contain the
  // pointer and Int information.
  template<typename T1>
  bool operator==(const T1 *RHS) const {  return this->getOpaqueValue() == RHS; }
  template<typename T1>
  bool operator!=(const T1 *RHS) const { return !operator==(RHS); }
  template<typename T1>
  bool operator<(const T1 *RHS) const { return this->getOpaqueValue() < RHS; }
  template<typename T1>
  bool operator>(const T1 *RHS) const { return this->getOpaqueValue() > RHS; }

  // getAsInlineOperand, with the invert flag.
  inline PtrInvPair<VASTValue> getAsInlineOperand() const {
    // Get the underlying value, and invert the underlying value if necessary.
    return cast<PtrInvPair<VASTValue> >(get()->getAsInlineOperand(isInverted()));
  }

  inline void printAsOperand(raw_ostream &OS, unsigned UB, unsigned LB) const {
    get()->printAsOperand(OS, UB, LB, isInverted());
  }

  inline void printAsOperand(raw_ostream &OS) const {
    get()->printAsOperand(OS, isInverted());
  }
};

// Casting PtrInvPair.
template<class To, class From>
struct cast_retty_impl<PtrInvPair<To>, PtrInvPair<From> >{
  typedef PtrInvPair<To> ret_type;
};

template<class ToTy, class FromTy>
struct cast_convert_val<PtrInvPair<ToTy>, PtrInvPair<FromTy>, PtrInvPair<FromTy> >{
  typedef PtrInvPair<ToTy> To;
  typedef PtrInvPair<FromTy> From;
  static typename cast_retty<To, From>::ret_type doit(const From &Val) {
    return To(cast_convert_val<ToTy, FromTy*, FromTy*>::doit(Val.get()),
              Val.isInverted());
  }
};

template <typename To, typename From>
struct isa_impl<PtrInvPair<To>, PtrInvPair<From> > {
  static inline bool doit(const PtrInvPair<From> &Val) {
    return To::classof(Val.get());
  }
};

template<class To, class From>
struct cast_retty_impl<To, PtrInvPair<From> > : public cast_retty_impl<To, From*>
{};

template<class To, class FromTy> struct cast_convert_val<To,
                                                         PtrInvPair<FromTy>,
                                                         PtrInvPair<FromTy> > {
  typedef PtrInvPair<FromTy> From;
  static typename cast_retty<To, From>::ret_type doit(const From &Val) {
    return cast_convert_val<To, FromTy*, FromTy*>::doit(Val.get());
  }
};

template <typename To, typename From>
struct isa_impl<To, PtrInvPair<From> > {
  static inline bool doit(const PtrInvPair<From> &Val) {
    return !Val.isInverted() && To::classof(Val.get());
  }
};

typedef PtrInvPair<VASTValue> VASTValPtr;
typedef PtrInvPair<VASTExpr> VASTExprPtr;
typedef PtrInvPair<VASTWire> VASTWirePtr;

class VASTUse : public ilist_node<VASTUse> {
  VASTValPtr V;
  VASTValue *User;
  friend class VASTExpr;

  friend struct ilist_sentinel_traits<VASTUse>;
  VASTUse(VASTValPtr v = 0, VASTValue *user = 0);

  void operator=(const VASTUse &RHS); // DO NOT IMPLEMENT
  VASTUse(const VASTUse &RHS); // DO NOT IMPLEMENT

  friend class VASTModule;
  friend class VASTSlot;
  friend class VASTWire;
public:
  bool isInvalid() const { return !V; }

  void set(VASTValPtr RHS) {
    assert(!V && "Already using some value!");
    V = RHS;
  }

  void replaceUseBy(VASTValPtr RHS) {
    assert(V && V != RHS && "Cannot replace!");
    V = RHS;
  }

  // Set the user of this use and insert this use to use list.
  void setUser(VASTValue *User);
  // Get the user of this use.
  VASTValue *getUser() const { return User; }
  // Remove this use from use list.
  void removeFromList();

  bool operator==(const VASTValPtr RHS) const;

  bool operator!=(const VASTValPtr RHS) const {
    return !operator==(RHS);
  }

  bool operator<(const VASTUse &RHS) const {
    return V < RHS.V;
  }

  // Return the underlying VASTValue.
  VASTValPtr get() const {
    assert(!isInvalid() && "Not a valid Use!");
    return V;
  }

  operator VASTValPtr() const { return get(); }

  VASTValPtr operator->() const { return get(); }
  inline VASTValPtr getAsInlineOperand() const {
    return get().getAsInlineOperand(); }
  inline void printAsOperand(raw_ostream &OS, unsigned UB, unsigned LB) const {
    get().printAsOperand(OS, UB, LB);
  }

  inline void printAsOperand(raw_ostream &OS) const {
    get().printAsOperand(OS);
  }

  VASTValPtr unwrap() const { return V; }

  // Prevent the user from being removed.
  void PinUser() const;

  unsigned getBitWidth() const ;
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
protected:

  VASTValue(VASTTypes T, unsigned BitWidth) : VASTNode(T), BitWidth(BitWidth) {
    assert(T >= vastFirstValueType && T <= vastLastValueType
           && "Bad DeclType!");
  }

  void addUseToList(VASTUse *U) { UseList.push_back(U); }
  void removeUseFromList(VASTUse *U) { UseList.remove(U); }

  friend class VASTUse;

  virtual void printAsOperandImpl(raw_ostream &OS, unsigned UB, unsigned LB) const;

  virtual void printAsOperandImpl(raw_ostream &OS) const {
    printAsOperandImpl(OS, getBitWidth(), 0);
  }

  // Print the value as inline operand.
  virtual VASTValPtr getAsInlineOperandImpl() { return this; }
public:
  const uint8_t BitWidth;
  unsigned getBitWidth() const { return BitWidth; }

  typedef VASTUseIterator<UseListTy::iterator, VASTValue> use_iterator;
  use_iterator use_begin() { return use_iterator(UseList.begin()); }
  use_iterator use_end() { return use_iterator(UseList.end()); }

  bool use_empty() const { return UseList.empty(); }
  size_t num_uses() const { return UseList.size(); }

  void printAsOperand(raw_ostream &OS, unsigned UB, unsigned LB,
                      bool isInverted) const;
  void printAsOperand(raw_ostream &OS, bool isInverted) const;

  VASTValPtr getAsInlineOperand(bool isInverted) {
    return getAsInlineOperandImpl().invert(isInverted);
  }

  virtual void print(raw_ostream &OS) const;

  bool replaceAllUseWith(VASTValue *To);

  typedef const VASTUse *dp_dep_it;
  static dp_dep_it dp_dep_begin(VASTValue *V);
  static dp_dep_it dp_dep_end(VASTValue *V);

  static bool is_dp_leaf(VASTValue *V) {
    return dp_dep_begin(V) == dp_dep_end(V);
  }

  // Helper function.
  static std::string printBitRange(unsigned UB, unsigned LB, bool printOneBit);

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const VASTValue *A) { return true; }
  static inline bool classof(const VASTNode *A) {
    return A->getASTType() >= vastFirstValueType &&
           A->getASTType() <= vastLastValueType;
  }
};

// simplify_type - Allow clients to treat VASTRValue just like VASTValues when
// using casting operators.
template<> struct simplify_type<const VASTUse> {
  typedef VASTValPtr SimpleType;
  static SimpleType getSimplifiedValue(const VASTUse &Val) {
    return Val.unwrap();
  }
};

template<> struct simplify_type<VASTUse> {
  typedef VASTValPtr SimpleType;
  static SimpleType getSimplifiedValue(const VASTUse &Val) {
    return Val.unwrap();
  }
};

class VASTImmediate : public VASTValue {
  VASTImmediate(uint64_t Imm, unsigned BitWidth)
    : VASTValue(vastImmediate, BitWidth) {
    Contents.IntVal = Imm;
  }

  friend class VASTModule;

  void printAsOperandImpl(raw_ostream &OS, unsigned UB, unsigned LB) const;

  void printAsOperandImpl(raw_ostream &OS) const {
    printAsOperandImpl(OS, getBitWidth(), 0);
  }
public:
  uint64_t getValue() const { return Contents.IntVal; }
  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const VASTImmediate *A) { return true; }
  static inline bool classof(const VASTNode *A) {
    return A->getASTType() == vastImmediate;
  }
};

class VASTNamedValue : public VASTValue {
protected:
  VASTNamedValue(VASTTypes T, const char *Name, unsigned BitWidth)
    : VASTValue(T, BitWidth) {
    assert(T >= vastFirstNamedType && T <= vastLastNamedType
           && "Bad DeclType!");
    Contents.Name = Name;
  }

  virtual void printAsOperandImpl(raw_ostream &OS, unsigned UB,
                                  unsigned LB) const;
  void printAsOperandImpl(raw_ostream &OS) const {
    printAsOperandImpl(OS, getBitWidth(), 0);
  }
public:
  const char *getName() const { return Contents.Name; }

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const VASTNamedValue *A) { return true; }
  static inline bool classof(const VASTNode *A) {
    return A->getASTType() >= vastFirstNamedType &&
           A->getASTType() <= vastLastNamedType;
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
  bool IsPinned;
  bool IsTimingUndef;
protected:
  uint8_t SignalType;
  uint16_t SignalData;
  // TODO: Annotate the signal so we know that some of them are port signals
  // and no need to declare again in the declaration list.
  const char *AttrStr;

  VASTSignal(VASTTypes DeclType, const char *Name, unsigned BitWidth,
             const char *Attr = "")
    : VASTNamedValue(DeclType, Name, BitWidth), IsPinned(false),
      IsTimingUndef(false), SignalType(0), SignalData(0), AttrStr(Attr) {}
public:

  // Pin the wire, prevent it from being remove.
  void Pin(bool isPinned = true) { IsPinned = isPinned; }
  bool isPinned() const { return IsPinned; }

  // The timing of a node may not captured by schedule information.
  void setTimingUndef(bool UnDef = true) { IsTimingUndef = UnDef; }
  bool isTimingUndef() const { return IsTimingUndef; }

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
public:
  const bool IsInput;

  VASTPort(VASTSignal *S, bool isInput) : VASTNode(vastPort), IsInput(isInput) {
    assert(!(isInput && isa<VASTRegister>(S)) && "Bad port decl!");
    Contents.Signal = S;
  }

  const char *getName() const { return Contents.Signal->getName(); }
  bool isRegister() const { return isa<VASTRegister>(Contents.Signal); }
  bool isInput() const { return IsInput; }
  unsigned getBitWidth() const { return Contents.Signal->getBitWidth(); }
  VASTSignal *get() const { return Contents.Signal; }
  operator VASTSignal *() const { return Contents.Signal; }

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
    // bitwise logic datapath
    dpAnd,
    dpNot,
    dpRAnd,
    dpROr,
    dpRXor,
    dpSel,
    // bit level assignment.
    dpBitCat,
    dpBitRepeat,
    LastInlinableOpc = dpBitRepeat,
    // Simple wire assignment.
    dpAssign,
    // Cannot inline.
    // FU datapath
    dpAdd,
    dpMul,
    dpShl,
    dpSRA,
    dpSRL,
    dpSCmp,
    dpUCmp,
    // Mux in datapath.
    dpMux,
    // Blackbox,
    dpBlackBox
  };
private:
  // Operands, right after this VASTExpr.
  const VASTUse *ops() const {
    return reinterpret_cast<const VASTUse*>(this + 1);
  }
  VASTUse *ops() { return reinterpret_cast<VASTUse*>(this + 1); }

  friend struct FoldingSetTrait<VASTExpr>;
  /// FastID - A reference to an Interned FoldingSetNodeID for this node.
  /// The ScalarEvolution's BumpPtrAllocator holds the data.
  const FoldingSetNodeIDRef FastID;

  VASTExpr(const VASTExpr&);             // Do not implement

  explicit VASTExpr(Opcode Opc, uint8_t numOps, unsigned UB,
                    unsigned LB, const FoldingSetNodeIDRef ID);

  friend class VASTModule;

  void printAsOperandInteral(raw_ostream &OS) const;

  void dropOperandsFromUseList() {
    for (VASTUse *I = ops(), *E = ops() + NumOps; I != E; ++I)
      I->removeFromList();
  }


  void printAsOperandImpl(raw_ostream &OS, unsigned UB, unsigned LB) const;

  void printAsOperandImpl(raw_ostream &OS) const {
    printAsOperandImpl(OS, UB, LB);
  }

  VASTValPtr getAsInlineOperandImpl() {
    // Can the expression be printed inline?
    if (getOpcode() == VASTExpr::dpAssign && !isSubBitSlice())
      return getOperand(0).getAsInlineOperand();

    return this;
  }
public:
  const uint8_t Opc, NumOps,UB, LB;
  Opcode getOpcode() const { return VASTExpr::Opcode(Opc); }

  const VASTUse &getOperand(unsigned Idx) const {
    assert(Idx < NumOps && "Index out of range!");
    return ops()[Idx];
  }

  typedef const VASTUse *op_iterator;
  op_iterator op_begin() const { return ops(); }
  op_iterator op_end() const { return ops() + NumOps; }

  //typedef VASTUse *op_iterator;
  //op_iterator op_begin() const { return ops(); }
  //op_iterator op_end() const { return ops() + num_ops(); }

  ArrayRef<VASTUse> getOperands() const {
    return ArrayRef<VASTUse>(ops(), NumOps);
  }

  bool isSubBitSlice() const {
    assert(getOpcode() == dpAssign && "Not an assignment!");
    return UB != getOperand(0)->getBitWidth() || LB != 0;
  }

  bool isInlinable() const {
    return getOpcode() <= LastInlinableOpc ||
           (getOpcode() == dpAssign && !isSubBitSlice());
  }

  void print(raw_ostream &OS) const { printAsOperandInteral(OS); }

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
  SmallVector<VASTValPtr, 4> Operands;
  VASTExpr::Opcode Opc;
  unsigned BitWidth;
  bool BuildNot;
  bool isBuildingOrExpr() { return BuildNot == true && Opc == VASTExpr::dpAnd; }

  void init(VASTExpr::Opcode opc, unsigned bitWidth, bool buildNot = false) {
    Opc = opc;
    BitWidth = bitWidth;
    BuildNot = buildNot;
  }

  void addOperand(VASTValPtr V) {
    Operands.push_back(V.getAsInlineOperand());
  }
};

class VASTWire :public VASTSignal {
public:
  enum Type {
    Common,
    // Result of a look up table, may have a large expression tree.
    LUT,
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

  friend class VASTModule;

  VASTWire(const char *Name, unsigned BitWidth, const char *Attr = "")
    : VASTSignal(vastWire, Name, BitWidth, Attr), U(0, 0) {
    SignalType = Common;
    SignalData = 0;
  }

  void setAsInput(VASTRegister *VReg);

  void setSlot(uint16_t slotNum) {
    assert(getWireType() == VASTWire::AssignCond && "setSlot on wrong type!");
    SignalData = slotNum;
  }

  void assign(VASTValPtr V, VASTWire::Type T = VASTWire::Common) {
    assert(U.isInvalid() && "The already has an expression!");
    SignalType = T;
    U.set(V);
    U.setUser(this);
  }

  void assignWithExtraDelay(VASTValPtr V, unsigned latency) {
    assign(V, haveExtraDelay);
    SignalData = latency;
  }


  void printAsOperandImpl(raw_ostream &OS, unsigned UB, unsigned LB) const;

  VASTValPtr getAsInlineOperandImpl() {
    if (getWireType() != LUT) {
      if (VASTValPtr V = getAssigningValue()) {
        // Can the expression be printed inline?
        if (VASTExprPtr E = dyn_cast<VASTExprPtr>(V)) {
          if (E->isInlinable()) return E.getAsInlineOperand();
        } else // This is a simple assignment.
          return V;
      }
    }

    return this;
  }
public:
  VASTValPtr getAssigningValue() const {
    // Ignore the virtual register of the input port, the virtual register only
    // carry the timing information.
    return getWireType() == InputPort ? VASTValPtr(0) : U.unwrap();
  }

  VASTRegister *getVirturalRegist() const {
    return cast<VASTRegister>(U.unwrap());
  }

  VASTExprPtr getExpr() const {
    return getAssigningValue()? dyn_cast<VASTExprPtr>(getAssigningValue()) : 0;
  }

  VASTWire::Type getWireType() const { return VASTWire::Type(SignalType); }

  unsigned getExtraDelayIfAny() const {
    return getWireType() == VASTWire::haveExtraDelay ? SignalData : 0;
  }

  uint16_t getSlotNum() const {
    assert(getWireType() == VASTWire::AssignCond &&
           "Call getSlot on bad wire type!");
    return SignalData;
  }

  // Print the logic to the output stream.
  void printAssignment(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const VASTWire *A) { return true; }
  static inline bool classof(const VASTNode *A) {
    return A->getASTType() == vastWire;
  }

  // Internal function used by VASTValue.
  VASTValue::dp_dep_it op_begin() const { return U.isInvalid() ? 0 : &U; }
  VASTValue::dp_dep_it op_end() const { return U.isInvalid() ? 0 : &U + 1; }
};

struct VASTWireExpressionTrait : public DenseMapInfo<VASTWire*> {
  static unsigned getHashValue(const VASTWire *Val) {
    if (Val == 0) return DenseMapInfo<void*>::getHashValue(0);

    if (VASTValPtr Ptr = Val->getAssigningValue())
      return DenseMapInfo<void*>::getHashValue(Ptr.getOpaqueValue());

    return DenseMapInfo<void*>::getHashValue(Val);
  }

  static const PtrInvPair<const VASTValue> getAssigningValue(const VASTWire *W) {
    if (W == getEmptyKey() || W == getTombstoneKey() || W == 0)
      return 0;

    if (const VASTValPtr V = W->getAssigningValue()) return V;

    return W;
  }

  static bool isEqual(const VASTWire *LHS, const VASTWire *RHS) {
    return LHS == RHS || getAssigningValue(LHS) == getAssigningValue(RHS);
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
  // Slot ranges of alias slot.
  uint16_t StartSlot;
  uint16_t EndSlot;
  uint16_t II;
  // Successor slots of this slot.
  succ_cnd_iterator succ_cnd_begin() { return NextSlots.begin(); }
  succ_cnd_iterator succ_cnd_end() { return NextSlots.end(); }
  
  const_succ_cnd_iterator succ_cnd_begin() const { return NextSlots.begin(); }
  const_succ_cnd_iterator succ_cnd_end() const { return NextSlots.end(); }

  void addEnable(VASTRegister *R, VASTValPtr Cnd, VASTModule *VM);
  void addReady(VASTValue *V, VASTValPtr Cnd, VASTModule *VM);
  void addDisable(VASTRegister *R, VASTValPtr Cnd, VASTModule *VM);
  void addSuccSlot(VASTSlot *NextSlot, VASTValPtr Cnd, VASTModule *VM);

  friend class VASTModule;
public:
  const uint16_t SlotNum;

  VASTSlot(unsigned slotNum, MachineBasicBlock *BB, VASTModule *VM);

  MachineBasicBlock *getParentBB() const { return Contents.ParentBB; }

  void buildCtrlLogic(VASTModule &Mod);
  // Print the logic of ready signal of this slot, need alias slot information.
  void buildReadyLogic(VASTModule &Mod);
  /// @briefPrint the ready expression of this slot.
  ///
  /// @param OS       The output stream.
  /// @param SrcSlot  Which Slot are the expression printing for?
  VASTValPtr buildFUReadyExpr(VASTModule &VM);

  void print(raw_ostream &OS) const;

  const char *getName() const;
  // Getting the relative signals.
  VASTRegister *getRegister() const { return cast<VASTRegister>(SlotReg); }
  VASTValue *getReady() const { return cast<VASTValue>(SlotReady); }
  VASTValue *getActive() const { return cast<VASTValue>(SlotActive); }

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
    StartSlot = startSlot ;
    EndSlot = endSlot;
    II = ii;
  }

  // Is the current slot the first slot of its alias slots?
  bool isLeaderSlot() const { return StartSlot == SlotNum; }
  // Iterates over all alias slot
  unsigned alias_start() const { return StartSlot; }
  unsigned alias_end() const { return EndSlot; }
  bool hasAliasSlot() const { return alias_start() != alias_end(); }
  unsigned alias_ii() const {
    assert(hasAliasSlot() && "Dont have II!");
    return II;
  }

  bool operator<(const VASTSlot &RHS) const {
    return SlotNum < RHS.SlotNum;
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
  typedef ArrayRef<VASTValPtr> AndCndVec;
  enum Type {
    Data,       // Common registers which hold data for data-path.
    Operand,    // Operand registers of functional units.
    Slot,       // Slot register which hold the enable signals for each slot.
    OutputPort, // The I/O register of an output port.
    Virtual     // Virtual registers that only hold timing information.
  };
private:
  uint64_t InitVal;

  // the first key VASTWire is Assignment condition. The second value is
  // assignment value.
  typedef DenseMap<VASTWire*, VASTUse*, VASTWireExpressionTrait> AssignMapTy;
  AssignMapTy Assigns;

  void addAssignment(VASTUse *Src, VASTWire *AssignCnd);

  VASTRegister(const char *Name, unsigned BitWidth, uint64_t InitVal,
               VASTRegister::Type T = Data, uint16_t RegData = 0,
               const char *Attr = "");
  friend class VASTModule;
public:
  VASTRegister::Type getRegType() const {
    return VASTRegister::Type(SignalType);
  }

  unsigned getDataRegNum() const {
    assert(getRegType() == Data && "Wrong accessor!");
    return SignalData;
  }

  unsigned getSlotNum() const {
    assert(getRegType() == Slot && "Wrong accessor!");
    return SignalData;
  }

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
  void dumpAssignment() const;

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
  void buildSlotLogic();
  void writeProfileCounters(VASTSlot *S, bool isFirstSlot);

  VASTImmediate *getOrCreateImmediate(uint64_t Value, int8_t BitWidth) {
    Value = getBitSlice64(Value, BitWidth);
    UniqueImmSetTy::key_type key = std::make_pair(Value, BitWidth);
    VASTImmediate *&Imm = UniqueImms.FindAndConstruct(key).second;
    if (!Imm) // Create the immediate if it is not yet created.
      Imm = new (Allocator.Allocate<VASTImmediate>()) VASTImmediate(Value, BitWidth);
    return Imm;
  }

  VASTImmediate *getBoolImmediate(bool Value) {
    return getOrCreateImmediate(Value ? 1 : 0, 1);
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

  // Create wrapper to allow us get a bitslice of the symbol.
  VASTValPtr getOrCreateSymbol(const std::string &Name, unsigned BitWidth,
                               bool CreateWrapper);

  void allocaSlots(unsigned TotalSlots) {
    Slots.assign(TotalSlots, 0);
  }

  VASTSlot *getOrCreateSlot(unsigned SlotNum, MachineBasicBlock *BB) {
    VASTSlot *&Slot = Slots[SlotNum];
    if(Slot == 0) {
      Slot = Allocator.Allocate<VASTSlot>();
      new (Slot) VASTSlot(SlotNum, BB, this);
    }

    return Slot;
  }

  VASTSlot *getOrCreateNextSlot(VASTSlot *S) {
    // TODO: Check if the next slot out of bound.
    return getOrCreateSlot(S->SlotNum + 1, S->getParentBB());
  }

  VASTSlot *getSlot(unsigned SlotNum) const {
    VASTSlot *S = Slots[SlotNum];
    assert(S && "Slot not exist!");
    return S;
  }

  VASTUse *allocateUse() { return Allocator.Allocate<VASTUse>(); }
  void addSlotEnable(VASTSlot *S, VASTRegister *R, VASTValPtr Cnd);
  void addSlotReady(VASTSlot *S, VASTValue *V, VASTValPtr Cnd);
  void addSlotDisable(VASTSlot *S, VASTRegister *R, VASTValPtr Cnd);
  void addSlotSucc(VASTSlot *S, VASTSlot *SuccS, VASTValPtr V);
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

  VASTValPtr createExpr(VASTExpr::Opcode Opc, ArrayRef<VASTValPtr> Ops,
                        unsigned UB, unsigned LB, bool IsInvert = false);

  VASTValPtr getOrCreateCommutativeExpr(VASTExpr::Opcode Opc,
                                        SmallVectorImpl<VASTValPtr> &Ops,
                                        unsigned BitWidth);

  VASTValPtr buildExpr(VASTExpr::Opcode Opc, ArrayRef<VASTValPtr> Ops,
                       unsigned BitWidth);
  VASTValPtr buildExpr(VASTExpr::Opcode Opc,VASTValPtr Op, unsigned BitWidth);
  VASTValPtr buildExpr(VASTExpr::Opcode Opc, VASTValPtr LHS, VASTValPtr RHS,
                       unsigned BitWidth);
  template<VASTExpr::Opcode Opc>
  static VASTValPtr buildExpr(VASTValPtr LHS, VASTValPtr RHS, unsigned BitWidth,
                              VASTModule *VM) {
    return VM->buildExpr(Opc, LHS, RHS, BitWidth);
  }

  VASTValPtr buildExpr(VASTExpr::Opcode Opc, VASTValPtr Op0, VASTValPtr Op1,
                       VASTValPtr Op2, unsigned BitWidth);
  VASTValPtr buildExpr(VASTExprBuilder &Builder) {
    VASTValPtr V = buildExpr(Builder.Opc, Builder.Operands, Builder.BitWidth);

    // If opc is dpAnd and BuildNot is true. It mean Or in And Invert Graph.
    if (Builder.BuildNot) V = buildNotExpr(V);

    return V;
  }

  VASTValPtr buildBitSliceExpr(VASTValPtr U, uint8_t UB, uint8_t LB);
  VASTValPtr buildBitCatExpr(ArrayRef<VASTValPtr> Ops, unsigned BitWidth);
  VASTValPtr buildAndExpr(ArrayRef<VASTValPtr> Ops, unsigned BitWidth);

  VASTValPtr buildMulExpr(ArrayRef<VASTValPtr> Ops, unsigned BitWidth);

  VASTValPtr buildAddExpr(ArrayRef<VASTValPtr> Ops, unsigned BitWidth);

  VASTValPtr buildNotExpr(VASTValPtr U);

  static VASTValPtr buildOr(VASTValPtr LHS, VASTValPtr RHS, unsigned BitWidth,
                            VASTModule *VM) {
    return VM->buildOrExpr(LHS, RHS, BitWidth);
  }

  VASTValPtr buildOrExpr(ArrayRef<VASTValPtr> Ops, unsigned BitWidth);

  VASTValPtr buildOrExpr(VASTValPtr LHS, VASTValPtr RHS, unsigned BitWidth) {
    VASTValPtr Ops[] = { LHS, RHS };
    return buildOrExpr(Ops, BitWidth);
  }

  static VASTValPtr buildXor(VASTValPtr LHS, VASTValPtr RHS, unsigned BitWidth,
                             VASTModule *VM) {
    VASTValPtr Ops[] = { LHS, RHS };
    return VM->buildXorExpr(Ops, BitWidth);
  }

  VASTValPtr buildXorExpr(ArrayRef<VASTValPtr> Ops, unsigned BitWidth);

  VASTValPtr flattenExprTree(VASTExpr::Opcode Opc, ArrayRef<VASTValPtr> Ops,
                             unsigned BitWidth);

  VASTRegister *addRegister(const std::string &Name, unsigned BitWidth,
                            unsigned InitVal = 0,
                            VASTRegister::Type T = VASTRegister::Data,
                            uint16_t RegData = 0, const char *Attr = "");

  VASTRegister *addOpRegister(const std::string &Name, unsigned BitWidth,
                              unsigned FUNum, const char *Attr = "") {
    return addRegister(Name, BitWidth, 0, VASTRegister::Operand, FUNum, Attr);
  }

  VASTRegister *addDataRegister(const std::string &Name, unsigned BitWidth,
                                unsigned RegNum = 0, const char *Attr = "") {
    return addRegister(Name, BitWidth, 0, VASTRegister::Data, RegNum, Attr);
  }

  VASTRegister *addSlotRegister(VASTSlot *S) {
    std::string SlotName = "Slot" + utostr_32(S->SlotNum);
    VASTRegister *R = addRegister(SlotName + "r", 1, S->SlotNum == 0 ? 1 : 0,
                                  VASTRegister::Slot, S->SlotNum,
                                  VASTModule::DirectClkEnAttr.c_str());
    // The slot enable register's timing is not captured by schedule information.
    R->setTimingUndef();
    return R;
  }

  VASTWire *addWire(const std::string &Name, unsigned BitWidth,
                    const char *Attr = "");

  reg_iterator reg_begin() { return Registers.begin(); }
  reg_iterator reg_end() { return Registers.end(); }

  slot_iterator slot_begin() { return Slots.begin(); }
  slot_iterator slot_end() { return Slots.end(); }

  void addAssignment(VASTRegister *Dst, VASTValPtr Src, VASTSlot *Slot,
                     SmallVectorImpl<VASTValPtr> &Cnds,
                     bool AddSlotActive = true);
  VASTWire *buildAssignCnd(VASTSlot *Slot, SmallVectorImpl<VASTValPtr> &Cnds,
                           bool AddSlotActive = true);

  VASTValPtr assign(VASTWire *W, VASTValPtr V,
                    VASTWire::Type T = VASTWire::Common);
  VASTValPtr assignWithExtraDelay(VASTWire *W, VASTValPtr V, unsigned latency);

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
    VASTValue *ChildNode = (*It).get().get();
    ++ItWorkStack.back();

    // Had we visited this node? If the Use slots are same, the same subtree
    // will lead to a same slack, and we do not need to compute the slack agian.
    if (!VisitedUses.insert(ChildNode).second) continue;

    // If ChildNode is not visit, go on visit it and its childrens.
    NodeWorkStack.push_back(ChildNode);
    ItWorkStack.push_back(VASTValue::dp_dep_begin(ChildNode));
  }
}
} // end namespace

#endif
