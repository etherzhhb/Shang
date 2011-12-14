//===------------- VLang.h - Verilog HDL writing engine ---------*- C++ -*-===//
//
//                            The Verilog Backend
//
// Copyright: 2010 by Hongbin Zheng. all rights reserved.
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

#include "vtm/FUInfo.h"
#include "vtm/LangSteam.h"

#include "llvm/Pass.h"
#include "llvm/Function.h"
#include "llvm/Target/Mangler.h"
#include "llvm/Target/TargetData.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/StringMap.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/Support/Allocator.h"
#include "llvm/Support/raw_ostream.h"

#include <map>
#include <list>

namespace llvm {
class MachineBasicBlock;
class ucOperand;
class VASTModule;
class VASTValue;
class VASTSlot;
class VASTWire;
class VASTRegister;
class VASTUse;

class VASTNode {
public:
  // Leaf node type of Verilog AST.
  enum VASTTypes {
    vastPort,
    vastWire,
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
  enum VASTUseTy {
    USE_Value,              // Using a VASTValue
    USE_Immediate,          // Simply a immediate
    USE_Symbol,             // A external symbol
    Invalid                 // For sentinel
  };
  // The ast node or simply the symbol.
  union {
    VASTValue *V;           // For USE_Value.
    int64_t ImmVal;         // For USE_Immediate.
    const char *SymbolName; // For USE_Symbol
  } Data;

  PointerIntPair<VASTValue*, 2, VASTUseTy> User;// VASTUseTy
  friend class VASTWire;

  VASTUseTy getUseKind() const { return User.getInt(); }
  void setUseKind(VASTUseTy K) { User.setInt(K); }

  friend struct ilist_sentinel_traits<VASTUse>;
public:
  VASTUse() : User(0, Invalid) {}
  bool isInvalid() const { return getUseKind() == Invalid; }

  // The bit range of this value.
  /*const*/ unsigned UB :8;
  /*const*/ unsigned LB :8;

  VASTUse(VASTValue *v, uint8_t ub, uint8_t lb)
    : User(0, USE_Value), UB(ub), LB(lb){
    Data.V = v;
  }

  VASTUse(VASTValue *v);

  VASTUse(int64_t immVal, uint8_t width)
    : User(0, USE_Immediate), UB(width), LB(0) {
      Data.ImmVal = immVal;
  }

  VASTUse(const char *S, uint8_t width)
    : User(0, USE_Symbol), UB(width), LB(0) {
      Data.SymbolName = S;
  }

  VASTUse getBitSlice(unsigned UB, unsigned LB) const;

  void setUser(VASTValue *User);

  VASTValue *getUser() const { return User.getPointer(); }

  void set(const VASTUse &RHS) {
    Data.V = RHS.Data.V;
    UB = RHS.UB;
    LB = RHS.LB;
    setUseKind(RHS.getUseKind());
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
    if (getUseKind() != RHS.getUseKind())
      return getUseKind() < RHS.getUseKind();

    if (Data.ImmVal != RHS.Data.ImmVal) return Data.ImmVal < RHS.Data.ImmVal;

    if (UB!= RHS.UB) return UB < RHS.UB;

    return LB < RHS.LB;
  }

  // Return the underlying VASTValue.
  VASTValue *get() const {
    assert(getUseKind() == USE_Value && "Call get on wrong VASTUse type!");
    return Data.V;
  }

  // Return the underlying VASTValue if the Use hold a VASTValue, null otherwise
  VASTValue *getOrNull() const {
    if (getUseKind() == USE_Value)
      return Data.V;

    return 0;
  }

  bool isImm() const { return getUseKind() == USE_Immediate; }
  int64_t getImm() const {
    assert(isImm() && "Call getImm on wrong type!");
    return Data.ImmVal;
  }

  // Iterators allow us to traverse the use tree.
  typedef const VASTUse *iterator;
  // Iterator for datapath traverse.
  iterator dp_src_begin();
  iterator dp_src_end();

  bool is_dp_leaf() { return dp_src_begin() == dp_src_end(); }

  unsigned getBitWidth() const { return UB - LB; }
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

  bool replaceAllUseWith(VASTUse To);
  void dropAllUses() { UseList.clear(); }
};

// simplify_type - Allow clients to treat VASTRValue just like VASTValues when
// using casting operators.
template<> struct simplify_type<const VASTUse> {
  typedef VASTNode *SimpleType;
  static SimpleType getSimplifiedValue(const VASTUse &Val) {
    return static_cast<SimpleType>(Val.get());
  }
};

template<> struct simplify_type<VASTUse> {
  typedef VASTNode *SimpleType;
  static SimpleType getSimplifiedValue(const VASTUse &Val) {
    return static_cast<SimpleType>(Val.get());
  }
};

class VASTSymbol : public VASTValue {
public:
  VASTSymbol(const char *Name, unsigned BitWidth)
    : VASTValue(VASTNode::vastSymbol, Name, BitWidth) {}

  virtual void print(raw_ostream &OS) const;
};

class VASTSignal : public VASTValue {
  // TODO: Annotate the signal so we know that some of them are port signals
  // and no need to declare again in the declaration list.
  const char *AttrStr;
protected:
  VASTSignal(VASTTypes DeclType, const char *Name, unsigned BitWidth,
             const char *Attr = "")
    : VASTValue(DeclType, Name, BitWidth), AttrStr(Attr) {}
public:

  void printDecl(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const VASTSignal *A) { return true; }
  static inline bool classof(const VASTWire *A) { return true; }
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

class VASTWire : public VASTSignal {
public:
  enum Opcode {
    // Datapath opcode.
    dpUnknown,
    // FU datapath
    dpAdd, dpMul, dpShl, dpSRA, dpSRL, dpSCmp, dpUCmp,
    // bitwise logic datapath
    dpAnd, dpOr, dpXor, dpNot, dpRAnd, dpROr, dpRXor,
    // bit level assignment.
    dpBitCat, dpBitRepeat,
    // Simple wire assignment.
    dpAssign,
    // Mux in datapath.
    dpMux,
    // Timing BlackBox, have latecy not capture by slots.
    dpVarLatBB,
    // Register Assignment.
    // Assignment condtion, Slot active signal should be included in ops if
    // necessary.
    cpAssignCnd
  };
private:
  VASTUse *Ops;
  // TODO: move to datapath.
  union {
    uint64_t Latency;
    VASTSlot *Slot;
  } Context;

  Opcode Opc;
  uint8_t NumOps;

  VASTWire(const VASTWire&);             // Do not implement

  VASTWire(const char *Name, unsigned BitWidth, const char *Attr = "",
           VASTUse *ops = 0, uint8_t numOps = 0, Opcode opc = dpUnknown);

  void setExpr(VASTUse *ops, uint8_t numOps, Opcode opc);

  friend class VASTModule;

  void printAsOperandInteral(raw_ostream &OS) const;

  void buildUseList();

  void setSlot(VASTSlot *Slot) {
    assert(getOpcode() == cpAssignCnd && "Call setSlot on bad wire type!");
    Context.Slot = Slot;
  }
public:
  bool hasExpr() const { return Ops != 0; }

  unsigned getLatency() const {
    assert(getOpcode() == dpVarLatBB && "Call getLatency on bad wire type!");
    return Context.Latency;
  }

  void setLatency(unsigned latency) {
    assert(getOpcode() == dpVarLatBB && "Call setLatency on bad wire type!");
    Context.Latency = latency;
  }

  VASTSlot *getSlot() const {
    assert(getOpcode() == cpAssignCnd && "Call getSlot on bad wire type!");
    return Context.Slot;
  }

  Opcode getOpcode() const { return Opc; }
  unsigned num_operands() const { return NumOps; }

  VASTUse getOperand(unsigned Idx) const {
    assert(Idx < num_operands() && "Index out of range!");
    return Ops[Idx];
  }

  typedef const VASTUse *op_iterator;
  op_iterator op_begin() const { return Ops; }
  op_iterator op_end() const { return Ops + NumOps; }


  // Print the logic to the output stream.
  void print(raw_ostream &OS) const;
  void printAsOperand(raw_ostream &OS, unsigned UB, unsigned LB) const {
    if (UB != getBitWidth() || LB != 0 || getName()) {
      VASTValue::printAsOperand(OS, UB, LB);
      return;
    }

    printAsOperandInteral(OS);
  }

  void printAsOperand(raw_ostream &OS) const {
    printAsOperandInteral(OS);
  }

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const VASTWire *A) { return true; }
  static inline bool classof(const VASTNode *A) {
    return A->getASTType() == vastWire;
  }
};

struct VASTWireBuilder {
  SmallVector<VASTUse, 4> Operands;
  VASTWire::Opcode Opc;
  unsigned BitWidth;
  void init(VASTWire::Opcode opc, unsigned bitWidth) {
    Opc = opc;
    BitWidth = bitWidth;
  }

  void addOperand(VASTUse U) { Operands.push_back(U); }
};

class VASTSlot : public VASTNode {
public:
  // TODO: Store the pointer to the Slot instead the slot number.
  typedef std::map<unsigned, VASTUse> SuccVecTy;
  typedef SuccVecTy::const_iterator const_succ_iterator;

  typedef std::map<VASTValue*, VASTUse> FUCtrlVecTy;
  typedef FUCtrlVecTy::const_iterator const_fu_ctrl_it;

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

  SuccVecTy NextSlots;
  // Slot ranges of alias slot.
  unsigned StartSlot, EndSlot, II;
  // The start slot of parent state, can identify parent state.
  unsigned ParentIdx;

public:
  VASTSlot(unsigned slotNum, unsigned parentIdx, VASTModule *VM);

  void printCtrl(vlang_raw_ostream &OS, VASTModule &Mod);
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
  // The slots passed after the parent state start before reach the current
  // slot.
  unsigned getSlackFromParentStart() const {
    return getSlotNum() - getParentIdx();
  }

  // TODO: Rename to addSuccSlot.
  void addNextSlot(unsigned NextSlotNum, VASTUse Cnd = VASTUse(true, 1));
  bool hasNextSlot(unsigned NextSlotNum) const;
  // Dose this slot jump to some other slot conditionally instead just fall
  // through to SlotNum + 1 slot?
  bool hasExplicitNextSlots() const { return !NextSlots.empty(); }

  // Successor slots of this slot.
  const_succ_iterator succ_begin() const { return NextSlots.begin(); }
  const_succ_iterator succ_end() const { return NextSlots.end(); }

  // Signals need to be enabled at this slot.
  void addEnable(VASTValue *V, VASTUse Cnd = VASTUse(true, 1));
  bool isEnabled(VASTValue *V) const { return Enables.count(V); }
  const_fu_ctrl_it enable_begin() const { return Enables.begin(); }
  const_fu_ctrl_it enable_end() const { return Enables.end(); }

  // Signals need to set before this slot is ready.
  void addReady(VASTValue *V, VASTUse Cnd = VASTUse(true, 1));
  bool readyEmpty() const { return Readys.empty(); }
  const_fu_ctrl_it ready_begin() const { return Readys.begin(); }
  const_fu_ctrl_it ready_end() const { return Readys.end(); }

  // Signals need to be disabled at this slot.
  void addDisable(VASTValue *V, VASTUse Cnd = VASTUse(true, 1));
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

  bool operator<(const VASTSlot &RHS) const {
    return getSlotNum() < RHS.getSlotNum();
  }
};

class VASTRegister : public VASTSignal {
public:
  typedef ArrayRef<VASTUse> AndCndVec;
private:
  unsigned InitVal;
  typedef DenseMap<VASTWire*, VASTUse*> AssignMapTy;
  AssignMapTy Assigns;
  // FIXME: We need a VAST live interval analysis pass to hold this.
  std::set<VASTSlot*, less_ptr<VASTSlot> > Slots;
  // The "Slack" in VAST means the extra cycles that after data appear in
  // the output pin of the src register before the dst register read the data.
  // i.e. if we assign reg0 at cycle 1, and the data will appear at the output
  // pin of reg0 at cycle 2, and now reg1 can read the data. In this case
  // becasue the data appear at cycle 2 and we read the data at the same cycle,
  // the slack is 0. But if we read the data at cycle 3, the slack is 1.

  // FIXME: These function should be the "SlackInfo" pass member function.
  unsigned findSlackFrom(const VASTRegister *Src, VASTSlot *UseSlot);
  // Find the nearest slot before Dst that assigning this register.
  VASTSlot *findNearestAssignSlot(VASTSlot *Dst) const;
  void DepthFristTraverseDataPathUseTree(VASTUse Root, VASTSlot *UseSlot);
  void addAssignment(VASTUse *Src, VASTWire *AssignCnd);

  friend class VASTModule;
public:
  VASTRegister(const char *Name, unsigned BitWidth, unsigned InitVal,
               const char *Attr = "");

  void printAssignment(vlang_raw_ostream &OS) const;
  void printReset(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const VASTRegister *A) { return true; }
  static inline bool classof(const VASTNode *A) {
    return A->getASTType() == vastRegister;
  }

  // Compute the slack of the assignment.
  void computeAssignmentSlack();
  void computeSlackThrough(VASTUse Def, VASTSlot *UseSlot);

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

  // The variable latency data-path whose latency information not capture
  // by schedule information.
  typedef std::map<unsigned, VASTWire*> VarLatBBMap;
  VarLatBBMap BBLatInfo;

  typedef std::vector<VASTSlot*> SlotVecTy;
  SlotVecTy Slots;
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
  void buildSlotLogic();
  void printSlotCtrls(vlang_raw_ostream &CtrlS);

  // Compute the register assignments slack information, callable from lua.
  void computeAssignmentSlacks();

  void addBBLatInfo(unsigned FNNum, VASTWire *W) {
    assert(W->getOpcode() == VASTWire::dpVarLatBB && "Wrong datapath type!");
    bool inserted = BBLatInfo.insert(std::make_pair(FNNum, W)).second;
    assert(inserted && "Latency information for BlackBox already exist!");
    (void)inserted;
  }

  VASTWire *getBBLatInfo(unsigned FNNum) const {
    VarLatBBMap::const_iterator at = BBLatInfo.find(FNNum);
    return at == BBLatInfo.end() ? 0 : at->second;
  }

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
       new (V) VASTSymbol(Entry.first(), BitWidth);
    }

    assert(V->getBitWidth() == BitWidth
           && "Getting symbol with wrong bitwidth!");
    return V;
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

  VASTWire *createExpr(VASTWire::Opcode Opc, ArrayRef<VASTUse> Ops,
                       unsigned BitWidth, VASTWire *DstWire = 0);
  VASTUse buildExpr(VASTWire::Opcode Opc, ArrayRef<VASTUse> Ops,
                    unsigned BitWidth, VASTWire *DstWire = 0);
  VASTUse buildExpr(VASTWire::Opcode Opc, VASTUse Op,
                    unsigned BitWidth, VASTWire *DstWire = 0);
  VASTUse buildExpr(VASTWire::Opcode Opc, VASTUse LHS, VASTUse RHS,
                    unsigned BitWidth, VASTWire *DstWire = 0);
  VASTUse buildExpr(VASTWire::Opcode Opc, VASTUse Op0, VASTUse Op1, VASTUse Op2,
                    unsigned BitWidth, VASTWire *DstWire = 0);
  VASTUse buildExpr(VASTWireBuilder &Builder, VASTWire *DstWire = 0) {
    return buildExpr(Builder.Opc, Builder.Operands, Builder.BitWidth, DstWire);
  }

  VASTUse buildNotExpr(VASTUse U);

  VASTRegister *addRegister(const std::string &Name, unsigned BitWidth,
                            unsigned InitVal = 0,
                            const char *Attr = "");

  VASTWire *addWire(const std::string &Name, unsigned BitWidth,
                    const char *Attr = "");

  VASTRegister *addRegister(unsigned RegNum, unsigned BitWidth,
                            unsigned InitVal = 0,
                            const char *Attr = "");

  VASTWire *addWire(unsigned WireNum, unsigned BitWidth,
                    const char *Attr = "");

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

std::string verilogConstToStr(Constant *C);

std::string verilogConstToStr(uint64_t value,unsigned bitwidth,
                              bool isMinValue);

std::string verilogBitRange(unsigned UB, unsigned LB = 0, bool printOneBit = true);

} // end namespace

#endif
