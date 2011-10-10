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

// Leaf node type of Verilog AST.
enum VASTTypes {
  vastPort,
  vastSignal,
  vastWire,
  vastRegister,
  vastSymbol,
  vastDatapath,
  vastSlot,
  vastRegAssign,
  vastParameter,
  vastFirstDeclType = vastPort,
  vastLastDeclType = vastParameter,

  vastModule
};

class VASTNode {
  std::string Comment;
  const unsigned short T;
  unsigned short SubclassData;
protected:
  VASTNode(VASTTypes NodeT, unsigned short subclassData,
           const std::string &comment)
    : Comment(comment), T(NodeT), SubclassData(subclassData) {}

  unsigned short getSubClassData() const { return SubclassData; }

public:
  virtual ~VASTNode() {}

  unsigned getASTType() const { return T; }

  virtual void print(raw_ostream &OS) const = 0;
};

class VASTValue : public VASTNode {
  std::string Name;
  bool IsReg;
  unsigned InitVal;
protected:
  VASTValue(VASTTypes DeclType, const std::string name, unsigned BitWidth, bool isReg,
           unsigned initVal, const std::string &Comment)
    : VASTNode(DeclType, BitWidth, Comment),Name(name), IsReg(isReg), InitVal(initVal)
  {
    assert(DeclType >= vastFirstDeclType && DeclType <= vastLastDeclType
           && "Bad DeclType!");
  }
public:
  const std::string &getName() const { return Name; }
  unsigned short getBitWidth() const { return getSubClassData(); }
  bool isRegister() const { return IsReg; }


  void printReset(raw_ostream &OS) const;

  // Out of line virtual function to provide home for the class.
  virtual void anchor();
};

// The predicate condition, maybe a inverted value.
class VASTCnd : public PointerIntPair<VASTValue*, 1, bool> {
  typedef PointerIntPair<VASTValue*, 1, bool> BaseTy;
public:
  /*implicit*/ VASTCnd(VASTValue *V = 0, bool Inverted = false)
    : BaseTy(V, Inverted)
  {
    assert((V == 0 || V->getBitWidth() == 1) && "Expected 1 bit condition!");
  }

  /*implicit*/ VASTCnd(bool Cnd) : BaseTy(0, !Cnd) {}

  bool isInverted() const { return getInt(); }
  VASTValue *getCndVal() const { return getPointer(); }

  // Return the "not" condition of current condition;
  VASTCnd invert() const { return VASTCnd(getCndVal(), !isInverted()); }

  void print(raw_ostream &OS) const;

  static VASTCnd Create(VASTModule *M, ucOperand &Op);
};

class VASTPort : public VASTValue {
  bool IsInput;
public:
  VASTPort(const std::string Name, unsigned BitWidth, bool isInput, bool isReg,
           const std::string &Comment)
    : VASTValue(vastPort, Name, BitWidth, isReg, 0, Comment), IsInput(isInput) {
    assert(!(isInput && isRegister()) && "Bad port decl!");
  }

  bool isInput() const { return IsInput; }

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

class VASTSignal : public VASTValue {
public:
  VASTSignal(const std::string Name, unsigned BitWidth, bool isReg,
             const std::string &Comment, VASTTypes DeclType = vastSignal,
             unsigned InitVal = 0)
    : VASTValue(DeclType, Name, BitWidth, isReg, InitVal, Comment) {}

  virtual void print(raw_ostream &OS) const;
  void printDecl(raw_ostream &OS) const;

  // Out of line virtual function to provide home for the class.
  virtual void anchor();
};

class VASTWire : public VASTSignal {
public:
  VASTWire(const std::string Name, unsigned BitWidth,
    const std::string &Comment)
    : VASTSignal(Name, BitWidth, 0, Comment, vastWire) {}

  void print(raw_ostream &OS) const {};

  // Out of line virtual function to provide home for the class.
  virtual void anchor();
};

class VASTRegister : public VASTSignal {
public:
  VASTRegister(const std::string Name, unsigned BitWidth,
    const std::string &Comment)
    : VASTSignal(Name, BitWidth, 1, Comment, vastRegister) {}

  void print(raw_ostream &OS) const {};

  // Out of line virtual function to provide home for the class.
  virtual void anchor();
};

class VASTSymbol : public VASTValue {
  public:
  VASTSymbol(const std::string Name, unsigned BitWidth,
             const std::string &Comment)
    : VASTValue(vastSymbol, Name, BitWidth, 0, 0, Comment) {}

  void print(raw_ostream &OS) const;
};

class VASTDatapath : public VASTNode {
public:
  typedef raw_string_ostream builder_stream;
private:
  std::vector<VASTValue*> Inputs, Outputs;
  std::string Code;
  builder_stream CodeStream;
public:
  VASTDatapath() : VASTNode(vastDatapath, 0, ""), CodeStream(Code) {}

  void print(raw_ostream &OS) const;

  builder_stream &getCodeBuffer() {
    return CodeStream;
  }

  void addInput (VASTValue *input)   { Inputs.push_back(input); }
  void addOutput(VASTValue *output)  { Outputs.push_back(output); }
};

class VASTSlot : public VASTSignal {
public:
  typedef std::map<unsigned, VASTCnd> SuccVecTy;
  typedef SuccVecTy::const_iterator const_succ_iterator;

  typedef std::map<const VASTValue*, VASTCnd> FUCtrlVecTy;
  typedef FUCtrlVecTy::const_iterator const_fu_ctrl_it;

private:
  unsigned SlotNum;
  // The ready signals that need to wait before we go to next slot.
  FUCtrlVecTy Readys;
  // The function units that enabled at this slot.
  FUCtrlVecTy Enables;
  // The function units that need to disable when condition is not satisfy.
  FUCtrlVecTy Disables;

  SuccVecTy NextSlots;
public:
  VASTSlot(unsigned slotNum)
    : VASTSignal("Slot" + utostr_32(slotNum), 1, true, "", vastSlot,
                 slotNum == 0), SlotNum(slotNum) {}

  void printCtrl(vlang_raw_ostream &OS, const VASTModule &Mod) const;
  void printActive(raw_ostream &OS) const;

  unsigned getSlotNum() const { return SlotNum; }

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
  bool disableEmpty() const { return Disables.empty(); }
  const_fu_ctrl_it disable_begin() const { return Disables.begin(); }
  const_fu_ctrl_it disable_end() const { return Disables.end(); }

};

class VASTRegAssign : public VASTNode {
  VASTSlot *RegAssignSlot;
  std::vector<VASTValue*> Predicates;
  VASTValue *Src;
  VASTValue *Dst;
public:
  VASTRegAssign() : VASTNode(vastRegAssign, 0, ""),
    RegAssignSlot(), Predicates(), Src(), Dst() {}

  void print(raw_ostream &OS) const {};

  void addPredicate(VASTValue *predicate){
    Predicates.push_back(predicate);
  }

  VASTValue *getSrc() { return Src; }
  VASTValue *getDst() { return Dst; }
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
  raw_string_ostream StateDecl, DataPath, ControlBlock;
  vlang_raw_ostream LangControlBlock;
  PortVector Ports;
  SignalVector Signals;

  std::string Name;
  BumpPtrAllocator Allocator;
  std::vector<VASTDatapath*> Datapaths;
  std::map<unsigned, VASTValue*> RegsMap;
  StringMap<VASTValue*> SymbolTable;
  typedef std::vector<VASTSlot*> SlotVecTy;
  SlotVecTy Slots;
  // The port starting offset of a specific function unit.
  SmallVector<std::map<unsigned, unsigned>, VFUs::NumCommonFUs> FUPortOffsets;
  unsigned NumArgPorts, RetPortIdx;

  void insertVASTValue(StringRef Name, VASTValue *V) {
    StringMapEntry<VASTValue*> *Entry =
      StringMapEntry<VASTValue*>::Create(Name.begin(), Name.end(),
                                         SymbolTable.getAllocator(), V);
    SymbolTable.insert(Entry);
  }

public:
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

  VASTModule(const std::string &Name) : VASTNode(vastModule, 0, ""),
    StateDecl(*(new std::string())),
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

  VASTDatapath *createDatapath(){
    VASTDatapath *Datapath = new VASTDatapath();
    Datapaths.push_back(Datapath);
    return Datapath;
  }

  void printDatapath(raw_ostream &OS) const;
  // Print the slot control flow.
  void printSlotActives(raw_ostream &OS) const;
  void printSlotCtrls(vlang_raw_ostream &CtrlS) const;

  void addVASTValue(unsigned RegNum, VASTValue *V);

  VASTValue *getVASTValue(unsigned RegNum) const {
    std::map<unsigned, VASTValue*>::const_iterator at = RegsMap.find(RegNum);
    if(at == RegsMap.end()) return 0;

    return at->second;
  }

  VASTValue *getVASTValue(const std::string &Name) const {
    return SymbolTable.lookup(Name);
  }

  VASTValue *getVASTSymbol(const std::string &Name) {
    VASTValue *&S = SymbolTable[Name];
    if (!S)
      S = new (Allocator.Allocate<VASTSymbol>()) VASTSymbol(Name, 1, "");

    return S;
  }

  void allocaSlots(unsigned TotalSlots) {
    Slots.assign(TotalSlots, 0);
  }

  VASTSlot *getSlot(unsigned SlotNum) {
    VASTSlot *&Slot = Slots[SlotNum];
    if(Slot == 0)
      Slot = new (Allocator.Allocate<VASTSlot>()) VASTSlot(SlotNum);

    return Slot;
  }

  VASTSlot *getSlot(unsigned SlotNum) const {
    VASTSlot *S = Slots[SlotNum];
    assert(S && "Slot not exist!");
    return S;
  }

  // Allow user to add ports.
  VASTPort *addInputPort(const std::string &Name, unsigned BitWidth,
                         PortTypes T = Others,
                         const std::string &Comment = "");

  VASTPort *addOutputPort(const std::string &Name, unsigned BitWidth,
                          PortTypes T = Others, bool isReg = true,
                          const std::string &Comment = "");

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

  const std::string &getPortName(unsigned i) const {
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

  VASTSignal *addRegister(const std::string &Name, unsigned BitWidth,
                          const std::string &Comment = "");

  VASTSignal *addWire(const std::string &Name, unsigned BitWidth,
                      const std::string &Comment = "");

  void printSignalDecl(raw_ostream &OS);
  void printRegisterReset(raw_ostream &OS);

  void print(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const VASTModule *A) { return true; }
  static inline bool classof(const VASTNode *A) {
    return A->getASTType() == vastModule;
  }

  raw_ostream &getStateDeclBuffer() {
    return StateDecl;
  }

  std::string &getStateDeclStr() {
    return StateDecl.str();
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
