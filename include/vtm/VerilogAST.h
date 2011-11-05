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

namespace llvm {
class MachineBasicBlock;
class ucOperand;
class VASTModule;
class VASTSlot;

class VASTNode {
public:
  // Leaf node type of Verilog AST.
  enum VASTTypes {
    vastPort,
    vastWire,
    vastRegister,
    vastSymbol,
    vastDatapath,
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
};

// TODO: Change VASTValue to VASTNamedNode
class VASTValue : public VASTNode {
  const char *Name;
public:
  VASTValue(VASTTypes DeclType, const char *name, unsigned BitWidth)
    : VASTNode(DeclType, BitWidth), Name(name)
  {
    assert(DeclType >= vastFirstDeclType && DeclType <= vastLastDeclType
      && "Bad DeclType!");
  }

  const char *getName() const { return Name; }
  unsigned short getBitWidth() const { return getSubClassData(); }
  bool isRegister() const { return getASTType() == vastRegister; }

  virtual void print(raw_ostream &OS) const;
};

class VASTUse {
protected:
  // The ast node or simply the symbol.
  VASTValue *V;

public:
  // The bit range of this value.
  /*const*/ uint8_t UB, LB;

  VASTUse(VASTValue *v, uint8_t ub, uint8_t lb)
    : V(v),UB(ub), LB(lb) {}

  VASTUse(VASTValue *v) : V(v),UB(v->getBitWidth()), LB(0) {}

  VASTUse() : V(0), UB(0), LB(0) {}

  //const VASTRValue& operator=(const VASTRValue &RHS) {
  //  if (&RHS == this) return *this;

  //  V = RHS.V;
  //  UB = RHS.UB;
  //  LB = RHS.LB;
  //  return *this;
  //}

  operator bool() const { return V != 0; }
  VASTValue *get() const { return V; }
  // Implicit cast to VASTValue.
  operator VASTValue *() const { return V; }
  VASTValue *operator->() const { return V; }

  void print(raw_ostream &OS) const;
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

struct VASTRValueLess  {
  bool operator() (const VASTUse &LHS, const VASTUse &RHS) const {
    return ((LHS.get() < RHS.get()
            || (LHS.get() == RHS.get() && LHS.UB < RHS.UB))
            || (LHS.get() == RHS.get() && LHS.UB == RHS.UB && LHS.UB < RHS.UB));
  }
};

// The predicate condition, maybe a inverted value.
class VASTCnd : public VASTUse {
  bool Inverted;
public:
  /*implicit*/ VASTCnd(VASTValue *V, bool inverted = false,
                       unsigned ub = 0, unsigned lb = 0)
    : VASTUse(V, ub, lb), Inverted(inverted)
  {
    //assert((V == 0 || V->getBitWidth() == 1) && "Expected 1 bit condition!");
  }

  /*implicit*/ VASTCnd(VASTUse V, bool inverted = false)
    : VASTUse(V), Inverted(inverted)
  {
    //assert((V == 0 || V->getBitWidth() == 1) && "Expected 1 bit condition!");
  }

  /*implicit*/ VASTCnd(bool Cnd = true) : VASTUse(), Inverted(!Cnd) {}

  //const VASTCnd& operator=(const VASTCnd &RHS) {
  //  if (&RHS == this) return *this;
  //  Inverted = RHS.Inverted;
  //  VASTRValue::operator=(RHS);
  //  return *this;
  //}

  bool isInverted() const { return Inverted; }
  VASTUse getCndVal() const { return VASTUse(*this); }

  // Return the "not" condition of current condition;
  VASTCnd invert() const { return VASTCnd(getCndVal(), !isInverted()); }

  void print(raw_ostream &OS) const;
};

class VASTSignal : public VASTValue {
  const char *AttrStr;
protected:
  VASTSignal(VASTTypes DeclType, const char *Name, unsigned BitWidth,
             const char *Attr = "")
    : VASTValue(DeclType, Name, BitWidth), AttrStr(Attr) {}
public:

  void printDecl(raw_ostream &OS) const;

  // Out of line virtual function to provide home for the class.
  virtual void anchor();
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
  typedef raw_string_ostream builder_stream;
private:
  SmallVector<VASTUse, 4> Operands;
  std::string Code;
  builder_stream *S;
public:
  VASTWire(const char *Name, unsigned BitWidth,
           const char *Attr = "");
  builder_stream &openCodeBuffer();
  builder_stream &getCodeBuffer() {
    assert(S && "Code buffer not open!");
    return *S;
  }

  void closeCodeBuffer();

  VASTUse getOperand(unsigned Idx) const {
    assert(Idx < Operands.size() && "Index out of range!");
    return Operands[Idx];
  }

  void addOperand (VASTUse Input)   { Operands.push_back(Input); }
  void print(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const VASTWire *A) { return true; }
  static inline bool classof(const VASTNode *A) {
    return A->getASTType() == vastWire;
  }
};

class VASTRegister : public VASTSignal {
public:
  typedef ArrayRef<VASTCnd> AndCndVec;
  typedef std::pair<VASTSlot*, AndCndVec> AssignCndTy;
private:
  unsigned InitVal;
  typedef std::vector<AssignCndTy>  OrCndVec;
  typedef std::map<VASTUse, OrCndVec, VASTRValueLess> AssignMapTy;
  AssignMapTy Assigns;
public:
  VASTRegister(const char *Name, unsigned BitWidth, unsigned InitVal,
               const char *Attr = "");

  void addAssignment(VASTUse Src, AndCndVec Cnd, VASTSlot *S);

  void printAssignment(vlang_raw_ostream &OS) const;
  void printReset(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const VASTRegister *A) { return true; }
  static inline bool classof(const VASTNode *A) {
    return A->getASTType() == vastRegister;
  }

  static void printCondition(raw_ostream &OS, const VASTSlot *Slot,
                             const AndCndVec Cnds);

  static void printCondition(raw_ostream &OS, AssignCndTy Cnd) {
    printCondition(OS, Cnd.first, Cnd.second);
  }
};

class VASTSlot : public VASTNode {
public:
  typedef std::map<unsigned, VASTCnd> SuccVecTy;
  typedef SuccVecTy::const_iterator const_succ_iterator;

  typedef std::map<const VASTValue*, VASTCnd> FUCtrlVecTy;
  typedef FUCtrlVecTy::const_iterator const_fu_ctrl_it;

private:
  VASTRegister *R;
  // The ready signals that need to wait before we go to next slot.
  FUCtrlVecTy Readys;
  // The function units that enabled at this slot.
  FUCtrlVecTy Enables;
  // The function units that need to disable when condition is not satisfy.
  FUCtrlVecTy Disables;

  SuccVecTy NextSlots;

  unsigned StartSlot, EndSlot, II;
public:
  VASTSlot(unsigned slotNum, VASTRegister *r)
    : VASTNode(vastSlot, slotNum), R(r), StartSlot(slotNum), EndSlot(slotNum),
    II(~0) {}

  void printCtrl(vlang_raw_ostream &OS, const VASTModule &Mod) const;
  void printActive(raw_ostream &OS, const VASTModule &Mod) const;
  void printReady(raw_ostream &OS) const;

  void print(raw_ostream &OS) const {}

  const char *getName() const { return R->getName(); }
  unsigned getSlotNum() const { return getSubClassData(); }

  void addNextSlot(unsigned NextSlotNum, VASTCnd Cnd = VASTCnd());
  bool hasExplicitNextSlots() const { return !NextSlots.empty(); }

  const_succ_iterator succ_begin() const { return NextSlots.begin(); }
  const_succ_iterator succ_end() const { return NextSlots.end(); }

  void addEnable(const VASTValue *V, VASTCnd Cnd = VASTCnd());
  bool isEnabled(const VASTValue *V) const { return Enables.count(V); }
  const_fu_ctrl_it enable_begin() const { return Enables.begin(); }
  const_fu_ctrl_it enable_end() const { return Enables.end(); }

  void addReady(const VASTValue *V, VASTCnd Cnd = VASTCnd());
  bool readyEmpty() const { return Readys.empty(); }
  const_fu_ctrl_it ready_begin() const { return Readys.begin(); }
  const_fu_ctrl_it ready_end() const { return Readys.end(); }

  void addDisable(const VASTValue *V, VASTCnd Cnd = VASTCnd());
  bool isDiabled(const VASTValue *V) const { return Disables.count(V); }
  bool disableEmpty() const { return Disables.empty(); }
  const_fu_ctrl_it disable_begin() const { return Disables.begin(); }
  const_fu_ctrl_it disable_end() const { return Disables.end(); }

  void setAliasSlots(unsigned startSlot, unsigned endSlot, unsigned ii) {
    StartSlot = startSlot;
    EndSlot = endSlot;
    II = ii;
  }
};

// The class that represent Verilog modulo.
class VASTModule : public VASTNode {
public:
  typedef SmallVector<VASTPort*, 16> PortVector;
  typedef PortVector::iterator port_iterator;
  typedef PortVector::const_iterator const_port_iterator;

  typedef SmallVector<VASTSignal*, 128> SignalVector;

private:
  // Dirty Hack:
  // Buffers
  raw_string_ostream DataPath, ControlBlock;
  vlang_raw_ostream LangControlBlock;
  PortVector Ports;
  SignalVector Signals;

  std::string Name;
  BumpPtrAllocator Allocator;
  typedef std::map<unsigned, VASTUse> RegIdxMapTy;
  RegIdxMapTy RegsMap;
  typedef StringMap<VASTValue*> SymTabTy;
  SymTabTy SymbolTable;
  typedef StringMapEntry<VASTValue*> SymEntTy;

  typedef std::vector<VASTSlot*> SlotVecTy;
  SlotVecTy Slots;
  // The port starting offset of a specific function unit.
  SmallVector<std::map<unsigned, unsigned>, VFUs::NumCommonFUs> FUPortOffsets;
  unsigned NumArgPorts, RetPortIdx;

public:
  static std::string DirectClkEnAttr, ParallelCaseAttr;

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
  void clear();

  const std::string &getName() const { return Name; }

  void printDatapath(raw_ostream &OS) const;
  void printRegisterAssign(vlang_raw_ostream &OS) const;

  // Print the slot control flow.
  void printSlotActives(raw_ostream &OS) const;
  void printSlotCtrls(vlang_raw_ostream &CtrlS) const;

  VASTUse lookupSignal(unsigned RegNum) const {
    RegIdxMapTy::const_iterator at = RegsMap.find(RegNum);
    if(at == RegsMap.end()) return VASTUse();

    return at->second;
  }

  VASTValue *getSymbol(const std::string &Name) const {
    StringMap<VASTValue*>::const_iterator at = SymbolTable.find(Name);
    return at->second;
  }

  template<class T>
  T *getSymbol(const std::string &Name) const {
    return cast<T>(getSymbol(Name));
  }

  VASTValue *getOrCreateSymbol(const std::string &Name) {
    SymEntTy &Entry = SymbolTable.GetOrCreateValue(Name);
    VASTValue *&V = Entry.second;
    if (V == 0) {
       V = Allocator.Allocate<VASTValue>();
       new (V) VASTValue(vastSymbol, Entry.first(), 0);
    }

    return V;
  }

  void allocaSlots(unsigned TotalSlots) {
    Slots.assign(TotalSlots, 0);
  }

  VASTSlot *getSlot(unsigned SlotNum) {
    VASTSlot *&Slot = Slots[SlotNum];
    if(Slot == 0) {
      std::string SlotName = "Slot" + utostr_32(SlotNum);
      VASTRegister *R = addRegister(SlotName, 1, SlotNum == 0,
                                    DirectClkEnAttr.c_str());
      Slot = new (Allocator.Allocate<VASTSlot>()) VASTSlot(SlotNum, R);
    }
    return Slot;
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

  VASTRegister::AndCndVec allocateAndCndVec(SmallVectorImpl<VASTCnd> &Cnds);
  void addAssignment(VASTRegister *Dst, VASTUse Src, VASTSlot *Slot,
                     SmallVectorImpl<VASTCnd> &Cnds);

  VASTValue *indexVASTValue(unsigned RegNum, VASTUse V);

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

raw_ostream &verilogParam(raw_ostream &ss, const std::string &Name,
                          unsigned BitWidth, unsigned Val);

} // end namespace

#endif
