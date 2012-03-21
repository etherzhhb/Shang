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
    vastPort,
    vastWire,
    vastExpr,
    vastRegister,
    vastSymbol,
    vastSlot,
    vastFirstDeclType = vastPort,
    vastLastDeclType = vastSlot,

    vastModule
  };
private:
  const unsigned short T;
  unsigned short SubclassData;
protected:
  VASTNode(VASTTypes NodeT, unsigned short subclassData)
    : T(NodeT), SubclassData(subclassData) {}

  unsigned short getSubClassData() const { return SubclassData; }

public:
  virtual ~VASTNode() {}

  unsigned getASTType() const { return T; }

  virtual void print(raw_ostream &OS) const = 0;
  void dump() const;
};

class VASTUse : public ilist_node<VASTUse> {
  VASTValue *V;
  VASTValue *User;
  friend class VASTExpr;

  friend struct ilist_sentinel_traits<VASTUse>;
public:
  VASTUse(VASTValue *v = 0);
  bool isInvalid() const { return V == 0; }

  // Set the user of this use and insert this use to use list.
  void setUser(VASTValue *User);
  // Get the user of this use.
  VASTValue *getUser() const { return User; }
  // Remove this use from use list.
  void removeFromList();

  void set(const VASTUse &RHS) {
    V = RHS.V;
  }

  const VASTUse& operator=(const VASTUse &RHS) {
    // Do not copy the parent.
    set(RHS);
    return *this;
  }

  VASTUse(const VASTUse &RHS) {
    // Do not copy the parent.
    set(RHS);
  }

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

  VASTValue *operator->() const { return operator*(); }

  VASTValue *unwrap() const { return V; }

  // Prevent the user from being removed.
  void PinUser() const;

  // Iterators allow us to traverse the use tree.
  typedef const VASTUse *iterator;
  // Iterator for datapath traverse.
  iterator dp_src_begin();
  iterator dp_src_end();

  bool is_dp_leaf() { return dp_src_begin() == dp_src_end(); }

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

// TODO: Change VASTValue to VASTNamedNode
class VASTValue : public VASTNode {
  const char *Name;

  typedef iplist<VASTUse> UseListTy;
  UseListTy UseList;
protected:
  VASTValue(VASTTypes DeclType, const char *name, unsigned BitWidth)
    : VASTNode(DeclType, BitWidth), Name(name)
  {
    assert(DeclType >= vastFirstDeclType && DeclType <= vastLastDeclType
           && "Bad DeclType!");
  }

  void addUseToList(VASTUse *U) { UseList.push_back(U); }
  void removeUseFromList(VASTUse *U) { UseList.remove(U); }
  friend class VASTUse;
public:
  const char *getName() const { return Name; }
  unsigned short getBitWidth() const { return getSubClassData(); }
  bool isRegister() const { return getASTType() == vastRegister; }

  typedef VASTUseIterator<UseListTy::iterator, VASTValue> use_iterator;
  use_iterator use_begin() { return use_iterator(UseList.begin()); }
  use_iterator use_end() { return use_iterator(UseList.end()); }

  bool use_empty() const { return UseList.empty(); }

  virtual void print(raw_ostream &OS) const;
  virtual void printAsOperand(raw_ostream &OS, unsigned UB, unsigned LB) const;

  void printAsOperand(raw_ostream &OS) const {
    printAsOperand(OS, getBitWidth(), 0);
  }

  bool replaceAllUseWith(VASTUse To,
    SmallVectorImpl<VASTExpr*> *ReplacedUsers = 0);
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

class VASTSymbol : public VASTValue {
  VASTSymbol(const char *Name, unsigned BitWidth)
    : VASTValue(VASTNode::vastSymbol, Name, BitWidth) {}

  friend class VASTModule;
  virtual void print(raw_ostream &OS) const;
};

class VASTSignal : public VASTValue {
  // TODO: Annotate the signal so we know that some of them are port signals
  // and no need to declare again in the declaration list.
  const char *AttrStr;
  bool Pinned;
  bool HasUndefTiming;
protected:
  VASTSignal(VASTTypes DeclType, const char *Name, unsigned BitWidth,
             const char *Attr = "")
    : VASTValue(DeclType, Name, BitWidth), AttrStr(Attr), Pinned(false),
      HasUndefTiming(false) {}
public:

  // Pin the wire, prevent it from being remove.
  void Pin() { Pinned = true; }
  bool isPinned() const { return Pinned; }

  // The timing of a node may not captured by schedule information.
  void setTimingUndef(bool UnDef = true) { HasUndefTiming = UnDef; }
  bool isTimingUndef() const { return HasUndefTiming; }

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
  VASTSignal *S;
public:
  VASTPort(VASTSignal *s, bool isInput) : VASTNode(vastPort, isInput), S(s) {
    assert(!(isInput && S->isRegister()) && "Bad port decl!");
  }

  const char *getName() const { return S->getName(); }
  bool isRegister() const { return S->isRegister(); }
  unsigned getBitWidth() const { return S->getBitWidth(); }
  VASTSignal *get() const { return S; }
  operator VASTSignal *() const { return S; }

  bool isInput() const { return getSubClassData(); }

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

class VASTExpr : public VASTValue {
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
  VASTUse *Ops;
  uint8_t NumOps;
  uint8_t Opc;
  uint8_t UB;
  uint8_t LB;

  VASTExpr(const VASTExpr&);             // Do not implement

  VASTExpr(Opcode opc, VASTUse *ops, uint8_t numOps, unsigned BitWidth);

  void setExpr(VASTUse *ops, uint8_t numOps, Opcode opc);

  friend class VASTModule;

  void printAsOperandInteral(raw_ostream &OS) const;

  void buildUseList();

  void dropOperandsFromUseList() {
    for (VASTUse *I = Ops, *E = Ops + NumOps; I != E; ++I)
      I->removeFromList();
  }

public:
  VASTExpr(VASTUse *U, unsigned ub, unsigned lb);

  bool isDead() const { return Opc == Dead; }

  Opcode getOpcode() const { return VASTExpr::Opcode(Opc); }
  void setOpcode(Opcode O) { Opc = O; }

  unsigned num_operands() const { return NumOps; }

  VASTUse getOperand(unsigned Idx) const {
    assert(Idx < num_operands() && "Index out of range!");
    return Ops[Idx];
  }

  typedef const VASTUse *op_iterator;
  op_iterator op_begin() const { return Ops; }
  op_iterator op_end() const { return Ops + NumOps; }

  ArrayRef<VASTUse> getOperands() const {
    return ArrayRef<VASTUse>(Ops, NumOps);
  }

  uint8_t getUB() const { return UB; }
  uint8_t getLB() const { return LB; }

  void print(raw_ostream &OS, unsigned UB, unsigned LB) const {
    if (UB != getBitWidth() || LB != 0 || getName()) {
      VASTValue::printAsOperand(OS, UB, LB);
      return;
    }

    printAsOperandInteral(OS);
  }

  void print(raw_ostream &OS) const {
    printAsOperandInteral(OS);
  }

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const VASTExpr *A) { return true; }
  static inline bool classof(const VASTNode *A) {
    return A->getASTType() == vastExpr;
  }
};

struct VASTExprBuilder {
  SmallVector<VASTUse, 4> Operands;
  VASTExpr::Opcode Opc;
  unsigned BitWidth;
  void init(VASTExpr::Opcode opc, unsigned bitWidth) {
    Opc = opc;
    BitWidth = bitWidth;
  }

  void addOperand(VASTUse U) { Operands.push_back(U); }
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
  PointerIntPair<VASTExpr *, 2, VASTWire::Type> E;
  // TODO: move to datapath.
  union {
    uint64_t Latency;
    VASTSlot *Slot;
  } Context;

  friend class VASTModule;

  VASTWire(const char *Name, unsigned BitWidth, const char *Attr = "",
           VASTExpr *e = 0, VASTWire::Type T = VASTWire::Common)
    : VASTSignal(vastWire, Name, BitWidth, Attr), E(e, T) {
    Context.Latency = 0;
  }

  void setAsInput() {
    E.setInt(VASTWire::InputPort);
    // Pin the signal to prevent it from being optimized away.
    Pin();
    setTimingUndef();
  }

  void setSlot(VASTSlot *Slot) {
    //assert(getOpcode() == cpAssignAtSlot && "setSlot on wrong type!");
    Context.Slot = Slot;
  }
public:
  VASTExpr *getExpr() const { return E.getPointer();}

  VASTWire::Type getWireType() const { return E.getInt(); }

  void assign(VASTExpr *e, VASTWire::Type T = VASTWire::Common) {
    E.setPointer(e);
    E.setInt(T);
  }

  void assignWithExtraDelay(VASTExpr *e, unsigned latency) {
    assign(e, VASTWire::haveExtraDelay);
    Context.Latency = latency;
  }

  unsigned getExtraDelayIfAny() const {
    return getWireType() == VASTWire::haveExtraDelay ? Context.Latency : 0;
  }

  VASTSlot *getSlot() const {
    assert(getWireType() == VASTWire::AssignCond &&
           "Call getSlot on bad wire type!");
    return Context.Slot;
  }

  // Print the logic to the output stream.
  void print(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const VASTWire *A) { return true; }
  static inline bool classof(const VASTNode *A) {
    return A->getASTType() == vastWire;
  }
};

class VASTSlot : public VASTNode {
public:
  // TODO: Store the pointer to the Slot instead the slot number.
  typedef std::map<VASTSlot*, VASTUse> SuccVecTy;
  typedef SuccVecTy::iterator succ_cnd_iterator;
  typedef SuccVecTy::const_iterator const_succ_cnd_iterator;

  // Use mapped_iterator which is a simple iterator adapter that causes a
  // function to be dereferenced whenever operator* is invoked on the iterator.
  typedef
  std::pointer_to_unary_function<std::pair<VASTSlot*, VASTUse>, VASTSlot*>
  slot_getter;

  typedef mapped_iterator<succ_cnd_iterator, slot_getter> succ_iterator;
  typedef mapped_iterator<const_succ_cnd_iterator, slot_getter>
          const_succ_iterator;

  typedef std::map<VASTValue*, VASTUse> FUCtrlVecTy;
  typedef FUCtrlVecTy::const_iterator const_fu_ctrl_it;

  typedef SmallVector<VASTSlot*, 4> PredVecTy;
  typedef PredVecTy::iterator pred_it;
private:
  // The relative signal of the slot: Slot register, Slot active and Slot ready.
  VASTRegister *SlotReg;
  VASTUse       SlotActive;
  VASTUse       SlotReady;
  // The ready signals that need to wait before we go to next slot.
  FUCtrlVecTy Readys;
  // The function units that enabled at this slot.
  FUCtrlVecTy Enables;
  // The function units that need to disable when condition is not satisfy.
  FUCtrlVecTy Disables;

  PredVecTy PredSlots;

  SuccVecTy NextSlots;
  // Slot ranges of alias slot.
  unsigned StartSlot, EndSlot, II;
  // The start slot of parent state, can identify parent state.
  unsigned ParentIdx;

  // Successor slots of this slot.
  succ_cnd_iterator succ_cnd_begin() { return NextSlots.begin(); }
  succ_cnd_iterator succ_cnd_end() { return NextSlots.end(); }
  
  const_succ_cnd_iterator succ_cnd_begin() const { return NextSlots.begin(); }
  const_succ_cnd_iterator succ_cnd_end() const { return NextSlots.end(); }

public:
  VASTSlot(unsigned slotNum, unsigned parentIdx, VASTModule *VM);

  void buildCtrlLogic(VASTModule &Mod);
  // Print the logic of ready signal of this slot, need alias slot information.
  void buildReadyLogic(VASTModule &Mod);
  /// @briefPrint the ready expression of this slot.
  ///
  /// @param OS       The output stream.
  /// @param SrcSlot  Which Slot are the expression printing for?
  VASTUse buildFUReadyExpr(VASTModule &VM);

  void print(raw_ostream &OS) const;

  const char *getName() const;
  // Getting the relative signals.
  VASTRegister *getRegister() const { return SlotReg; }
  VASTUse getReady() const { return SlotReady; }
  VASTUse getActive() const { return SlotActive; }

  unsigned getSlotNum() const { return getSubClassData(); }
  // The start slot of parent state(MachineBasicBlock)
  unsigned getParentIdx() const { return ParentIdx; }

  // TODO: Rename to addSuccSlot.
  void addNextSlot(VASTSlot *NextSlot, VASTUse Cnd);
  bool hasNextSlot(VASTSlot *NextSlot) const;
  // Dose this slot jump to some other slot conditionally instead just fall
  // through to SlotNum + 1 slot?
  bool hasExplicitNextSlots() const { return !NextSlots.empty(); }

  // Next VASTSlot iterator. 
  succ_iterator succ_begin() {
    return map_iterator(NextSlots.begin(),
                        slot_getter(pair_first<VASTSlot*, VASTUse>));
  }

  const_succ_iterator succ_begin() const {
    return map_iterator(NextSlots.begin(),
                        slot_getter(pair_first<VASTSlot*, VASTUse>));
  }

  succ_iterator succ_end() {
    return map_iterator(NextSlots.end(),
                        slot_getter(pair_first<VASTSlot*, VASTUse>));
  }

  const_succ_iterator succ_end() const {
    return map_iterator(NextSlots.end(),
                        slot_getter(pair_first<VASTSlot*, VASTUse>));
  }

  // Predecessor slots of this slot.
  pred_it pred_begin() { return PredSlots.begin(); }
  pred_it pred_end() { return PredSlots.end(); }

  // Signals need to be enabled at this slot.
  void addEnable(VASTValue *V, VASTUse Cnd);
  bool isEnabled(VASTValue *V) const { return Enables.count(V); }
  const_fu_ctrl_it enable_begin() const { return Enables.begin(); }
  const_fu_ctrl_it enable_end() const { return Enables.end(); }

  // Signals need to set before this slot is ready.
  void addReady(VASTValue *V, VASTUse Cnd);
  bool readyEmpty() const { return Readys.empty(); }
  const_fu_ctrl_it ready_begin() const { return Readys.begin(); }
  const_fu_ctrl_it ready_end() const { return Readys.end(); }

  // Signals need to be disabled at this slot.
  void addDisable(VASTValue *V, VASTUse Cnd);
  bool isDiabled(VASTValue *V) const { return Disables.count(V); }
  bool disableEmpty() const { return Disables.empty(); }
  const_fu_ctrl_it disable_begin() const { return Disables.begin(); }
  const_fu_ctrl_it disable_end() const { return Disables.end(); }

  // This slots alias with this slot, this happened in a pipelined loop.
  // The slots from difference stage of the loop may active at the same time,
  // and these slot called "alias".
  void setAliasSlots(unsigned startSlot, unsigned endSlot, unsigned ii) {
    StartSlot = startSlot;
    EndSlot = endSlot;
    II = ii;
  }

  // Is the current slot the first slot of its alias slots?
  bool isLeaderSlot() const { return StartSlot == getSlotNum(); }
  // Iterates over all alias slot
  unsigned alias_start() const { return StartSlot; }
  unsigned alias_end() const { return EndSlot; }
  unsigned alias_ii() const { return II; }
  bool hasAliasSlot() const { return alias_start() != alias_end(); }

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
  typedef ArrayRef<VASTUse> AndCndVec;
private:
  unsigned InitVal;

  // the first key VASTWire is Assignment condition. The second value VASTUse is
  // assignment value.
  typedef DenseMap<VASTWire*, VASTUse*> AssignMapTy;
  AssignMapTy Assigns;

  // The slots that this register are assigned.
  std::set<VASTSlot*> Slots;

  void addAssignment(VASTUse *Src, VASTWire *AssignCnd);

  friend class VASTModule;
public:
  VASTRegister(const char *Name, unsigned BitWidth, unsigned InitVal,
               const char *Attr = "");

  void clearAssignments() {
    assert(use_empty() && "Cannot clear assignments!");
    // TODO: Release the Uses and Wires.
    Assigns.clear();
    Slots.clear();
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

  // get the Slots begin and end iterator.
  typedef std::set<VASTSlot*>::const_iterator slot_iterator;
  slot_iterator slots_begin() { return Slots.begin(); }
  slot_iterator slots_end() { return Slots.end();  }

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
  SlotVecTy Slots;
  typedef SlotVecTy::iterator slot_iterator;
private:
  // Dirty Hack:
  // Buffers
  raw_string_ostream DataPath, ControlBlock;
  vlang_raw_ostream LangControlBlock;
  PortVector Ports;
  WireVector Wires;
  RegisterVector Registers;

  std::string Name;
  BumpPtrAllocator Allocator;
  SpecificBumpPtrAllocator<VASTUse> UseAllocator;
  typedef DenseMap<unsigned, VASTUse*> RegIdxMapTy;
  RegIdxMapTy RegsMap;
  typedef StringMap<VASTValue*> SymTabTy;
  SymTabTy SymbolTable;
  typedef StringMapEntry<VASTValue*> SymEntTy;

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

  VASTModule(const std::string &Name) : VASTNode(vastModule, 0),
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

  VASTUse lookupSignal(unsigned RegNum) const {
    RegIdxMapTy::const_iterator at = RegsMap.find(RegNum);
    assert(at != RegsMap.end() && "Signal not found!");

    return *at->second;
  }

  VASTValue *getSymbol(const std::string &Name) const {
    StringMap<VASTValue*>::const_iterator at = SymbolTable.find(Name);
    assert(at != SymbolTable.end() && "Symbol not found!");
    return at->second;
  }

  template<class T>
  T *getSymbol(const std::string &Name) const {
    return cast<T>(getSymbol(Name));
  }

  VASTValue *getOrCreateSymbol(const std::string &Name, unsigned BitWidth) {
    SymEntTy &Entry = SymbolTable.GetOrCreateValue(Name);
    VASTValue *&V = Entry.second;
    if (V == 0) {
       V = Allocator.Allocate<VASTValue>();
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
    MapTy Map = FUPortOffsets[ID.getFUType()];
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

  VASTExpr *createExpr(VASTExpr::Opcode Opc, ArrayRef<VASTUse> Ops,
                       unsigned BitWidth);
  VASTUse buildExpr(VASTExpr::Opcode Opc, ArrayRef<VASTUse> Ops,
                    unsigned BitWidth);
  VASTUse buildExpr(VASTExpr::Opcode Opc, VASTUse Op, unsigned BitWidth);
  VASTUse buildExpr(VASTExpr::Opcode Opc, VASTUse LHS, VASTUse RHS,
                    unsigned BitWidth);
  VASTUse buildExpr(VASTExpr::Opcode Opc, VASTUse Op0, VASTUse Op1, VASTUse Op2,
                    unsigned BitWidth);
  VASTUse buildExpr(VASTExprBuilder &Builder) {
    return buildExpr(Builder.Opc, Builder.Operands, Builder.BitWidth);
  }

  VASTUse buildBitSlice(VASTUse U, uint8_t UB, uint8_t LB);

  VASTUse buildLogicExpr(VASTExpr::Opcode Opc, VASTUse LHS, VASTUse RHS,
                         unsigned BitWidth);
  bool replaceAndUpdateUseTree(VASTValue *From, VASTUse To);

  VASTExpr *updateExpr(VASTExpr *E, VASTExpr::Opcode Opc, ArrayRef<VASTUse> Ops);

  VASTUse buildNotExpr(VASTUse U);

  VASTRegister *addRegister(const std::string &Name, unsigned BitWidth,
                            unsigned InitVal = 0,
                            const char *Attr = "");

  VASTWire *addWire(const std::string &Name, unsigned BitWidth,
                    const char *Attr = "");

  VASTWire *addWire(unsigned WireNum, unsigned BitWidth,
                    const char *Attr = "");

  VASTRegister *addRegister(unsigned RegNum, unsigned BitWidth,
                            unsigned InitVal = 0,
                            const char *Attr = "");

  reg_iterator reg_begin() { return Registers.begin(); }
  reg_iterator reg_end() { return Registers.end(); }

  slot_iterator slot_begin() { return Slots.begin(); }
  slot_iterator slot_end() { return Slots.end(); }

  void addAssignment(VASTRegister *Dst, VASTUse Src, VASTSlot *Slot,
                     SmallVectorImpl<VASTUse> &Cnds, bool AddSlotActive = true);
  VASTWire *buildAssignCnd(VASTSlot *Slot, SmallVectorImpl<VASTUse> &Cnds,
                           bool AddSlotActive = true);

  VASTUse indexVASTValue(unsigned RegNum, VASTUse V);

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
void DepthFirstTraverseDepTree(VASTUse DepTree, VisitPathFunc VisitPath) {
  typedef VASTUse::iterator ChildIt;
  // Use seperate node and iterator stack, so we can get the path vector.
  typedef SmallVector<VASTUse, 16> NodeStackTy;
  typedef SmallVector<ChildIt, 16> ItStackTy;
  NodeStackTy NodeWorkStack;
  ItStackTy ItWorkStack;
  // Remember what we had visited.
  std::set<VASTUse> VisitedUses;

  // Put the root.
  NodeWorkStack.push_back(DepTree);
  ItWorkStack.push_back(DepTree.dp_src_begin());

  while (!ItWorkStack.empty()) {
    VASTUse Node = NodeWorkStack.back();

    ChildIt It = ItWorkStack.back();

    // Do we reach the leaf?
    if (Node.is_dp_leaf()) {
      if (VASTValue *V = *Node)
        VisitPath(NodeWorkStack);

      NodeWorkStack.pop_back();
      ItWorkStack.pop_back();
      continue;
    }

    // All sources of this node is visited.
    if (It == Node.dp_src_end()) {
      NodeWorkStack.pop_back();
      ItWorkStack.pop_back();
      continue;
    }

    // Depth first traverse the child of current node.
    VASTUse ChildNode = *It;
    ++ItWorkStack.back();

    // Had we visited this node? If the Use slots are same, the same subtree
    // will lead to a same slack, and we do not need to compute the slack agian.
    if (!VisitedUses.insert(ChildNode).second) continue;

    // If ChildNode is not visit, go on visit it and its childrens.
    NodeWorkStack.push_back(ChildNode);
    ItWorkStack.push_back(ChildNode.dp_src_begin());
  }
}


std::string verilogConstToStr(Constant *C);

std::string verilogConstToStr(uint64_t value,unsigned bitwidth,
                              bool isMinValue);

std::string verilogBitRange(unsigned UB, unsigned LB = 0, bool printOneBit = true);

} // end namespace

#endif
