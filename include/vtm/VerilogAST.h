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
#include "llvm/ADT/STLExtras.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/Support/raw_ostream.h"

#include <map>

namespace llvm {
class MachineBasicBlock;

// Leaf node type of Verilog AST.
enum VASTTypes {
  vastPort,
  vastSignal,
  vastWire,
  vastRegister,
  vastConstant,
  vastDatapath,
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
  VASTValue(VASTTypes DeclType, const std::string &name, unsigned BitWidth, bool isReg,
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

class VASTPort : public VASTValue {
  bool IsInput;
public:
  VASTPort(const std::string &Name, unsigned BitWidth, bool isInput, bool isReg,
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
  VASTSignal(const std::string &Name, unsigned BitWidth, bool isReg,
             const std::string &Comment, VASTTypes DeclType = vastSignal)
    : VASTValue(DeclType, Name, BitWidth, isReg, 0, Comment) {}

  void print(raw_ostream &OS) const;
  void printDecl(raw_ostream &OS) const;

  // Out of line virtual function to provide home for the class.
  virtual void anchor();
};

class VASTWire : public VASTSignal {
public:
  VASTWire(const std::string &Name, unsigned BitWidth,
    const std::string &Comment)
    : VASTSignal(Name, BitWidth, 0, Comment, vastWire) {}

  void print(raw_ostream &OS) const {};

  // Out of line virtual function to provide home for the class.
  virtual void anchor();
};

class VASTRegister : public VASTSignal {
public:
  VASTRegister(const std::string &Name, unsigned BitWidth,
    const std::string &Comment)
    : VASTSignal(Name, BitWidth, 1, Comment, vastRegister) {}

  void print(raw_ostream &OS) const {};

  // Out of line virtual function to provide home for the class.
  virtual void anchor();
};

class VASTConstant : public VASTValue {
  public:
  VASTConstant(const std::string &Name, unsigned BitWidth,
    const std::string &Comment)
    : VASTValue(vastConstant, Name, BitWidth, 0, 0, Comment) {}

  void print(raw_ostream &OS) const {};

  // Out of line virtual function to provide home for the class.
  virtual void anchor();
};

class VASTDatapath : public VASTNode {
  std::vector<VASTValue*> Inputs, Outputs;
  std::string Code;
public:
  VASTDatapath() : VASTNode(vastDatapath, 0, ""),
                   Inputs(), Outputs(), Code() {}

  void print(raw_ostream &OS) const;

  typedef raw_string_ostream builder_stream;

  builder_stream getCodeBuffer() {
    return raw_string_ostream(Code);
  }

  void addInput (VASTValue *input)   { Inputs.push_back(input); }
  void addOutput(VASTValue *output)  { Outputs.push_back(output); }
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
  std::vector<VASTDatapath *> Datapaths;
  std::map<unsigned, VASTValue *> ValueIndex;
  // The port starting offset of a specific function unit.
  SmallVector<std::map<unsigned, unsigned>, VFUs::NumCommonFUs> FUPortOffsets;
  unsigned NumArgPorts, RetPortIdx;
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
    Name(Name),
    StateDecl(*(new std::string())),
    DataPath(*(new std::string())),
    ControlBlock(*(new std::string())),
    LangControlBlock(ControlBlock),
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

  VASTValue *getValue(unsigned Index){
    std::map<unsigned, VASTValue *>::iterator at = ValueIndex.find(Index);
    if(at == ValueIndex.end())
      return 0;
    return at->second;
  }

  // Allow user to add ports.
  VASTPort *addInputPort(const std::string &Name, unsigned BitWidth,
                         PortTypes T = Others,
                         const std::string &Comment = "") {
    VASTPort *Port = new VASTPort(Name, BitWidth, true, false, Comment);
    if (T < SpecialInPortEnd) {
      assert(Ports[T] == 0 && "Special port exist!");
      Ports[T] = Port;
      return Port;
    }

    // Return port is a output port.
    assert(T < RetPort && "Wrong port type!");
    if (T == ArgPort) {
      assert(NumArgPorts == Ports.size() - NumSpecialPort
             && "Unexpected port added before arg port!");
      ++NumArgPorts;
    }

    Ports.push_back(Port);
    return Port;
  }

  VASTPort *addOutputPort(const std::string &Name, unsigned BitWidth,
                          PortTypes T = Others, bool isReg = true,
                          const std::string &Comment = "") {
    VASTPort *Port = new VASTPort(Name, BitWidth, false, isReg, Comment);
    if (SpecialInPortEnd <= T && T < SpecialOutPortEnd) {
      assert(Ports[T] == 0 && "Special port exist!");
      Ports[T] = Port;
      return Port;
    }

    assert(T <= RetPort && "Wrong port type!");
    if (T == RetPort) {
      RetPortIdx = Ports.size();
      assert(RetPortIdx == NumArgPorts + NumSpecialPort
             && "Unexpected port added before return port!");
    }

    Ports.push_back(Port);
    return Port;
  }

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

  const VASTPort &getPort(unsigned i) const {
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
                          const std::string &Comment = "") {
    VASTSignal *Reg = new VASTSignal(Name, BitWidth, true, Comment);
    Signals.push_back(Reg);
    return Reg;
  }

  VASTSignal *addWire(const std::string &Name, unsigned BitWidth,
                      const std::string &Comment = "") {
    VASTSignal *Signal = new VASTSignal(Name, BitWidth, false, Comment);
    Signals.push_back(Signal);
    return Signal;
  }

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
};

std::string verilogConstToStr(Constant *C);

std::string verilogConstToStr(uint64_t value,unsigned bitwidth,
                              bool isMinValue);

std::string verilogBitRange(unsigned UB, unsigned LB = 0, bool printOneBit = true);

raw_ostream &verilogParam(raw_ostream &ss, const std::string &Name,
                          unsigned BitWidth, unsigned Val);

} // end namespace

#endif
