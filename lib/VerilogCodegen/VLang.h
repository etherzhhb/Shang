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

#include "llvm/Pass.h"
#include "llvm/Function.h"
#include "llvm/Target/Mangler.h"
#include "llvm/Target/TargetData.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/Format.h"


using namespace llvm;

namespace esyn {

// Leaf node type of Verilog AST.
enum VASTTypes {
  vastPort,
  vastSignal,
  vastParameter,
  vastFirstDeclType = vastPort,
  vastLastDeclType = vastParameter,

  vastModule
};

class VASTNode {
  std::string Name;
  std::string Comment;
  const unsigned short T;
  unsigned short SubclassData;
protected:
  VASTNode(VASTTypes NodeT, const std::string &name, unsigned short subclassData,
           const std::string &comment)
    : T(NodeT), Name(name), Comment(comment), SubclassData(subclassData) {}

  unsigned short getSubClassData() const { return SubclassData; }

public:
  unsigned getASTType() const { return T; }

  const std::string &getName() const { return Name; } 
  
  virtual void print(raw_ostream &OS) const = 0;
};

class VASTValue : public VASTNode {
  bool IsReg;
  unsigned InitVal;
protected:
  VASTValue(VASTTypes DeclType, const std::string &Name, unsigned BitWidth, bool isReg,
           unsigned initVal, const std::string &Comment)
    : VASTNode(DeclType, Name, BitWidth, Comment), IsReg(isReg), InitVal(initVal)
  {
    assert(DeclType >= vastFirstDeclType && DeclType <= vastLastDeclType
           && "Bad DeclType!");
  }
public:
  unsigned short getBitWidth() const { return getSubClassData(); }
  bool isRegister() const { return IsReg; }

  static void print(const VASTValue &V, raw_ostream &OS);
};

class VASTPort : public VASTValue {
  bool IsInput;
public:
  VASTPort(const std::string &Name, unsigned BitWidth, bool isInput, bool isReg,
           const std::string &Comment)
    : VASTValue(vastPort, Name, BitWidth, isReg, 0, Comment), IsInput(isInput)
  {
    assert(!(isInput && isRegister()) && "Bad port decl!");
  }
  
  bool isInput() const { return IsInput; }

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const VASTPort *A) { return true; }
  static inline bool classof(const VASTNode *A) {
    return A->getASTType() == vastPort;
  }

  void print(raw_ostream &OS) const;
  void printExternal(raw_ostream &OS) const;
};

// The class that represent Verilog modulo.
class VModule : public VASTNode {
public:
  typedef SmallVector<VASTPort*, 8> PortVector;
private:
  // Dirty Hack:
  // Buffers
  raw_string_ostream StateDecl, SignalDecl, DataPath,
    ControlBlock, ResetBlock, SeqCompute;
  PortVector Ports;
public:
  VModule(const std::string &Name) : VASTNode(vastModule, Name, 0, ""),
    StateDecl(*(new std::string())),
    SignalDecl(*(new std::string())),
    DataPath(*(new std::string())),
    ControlBlock(*(new std::string())),
    ResetBlock(*(new std::string())),
    SeqCompute(*(new std::string())) {}

  ~VModule();
  void clear();

  // Allow user to add ports.
  VASTPort *addInputPort(const std::string &Name, unsigned BitWidth,
                         const std::string &Comment = "") {
    VASTPort *Port = new VASTPort(Name, BitWidth, true, false, Comment);
    Ports.push_back(Port);
    return Port;
  }

  VASTPort *addOutputPort(const std::string &Name, unsigned BitWidth,
                          const std::string &Comment = "") {
    VASTPort *Port = new VASTPort(Name, BitWidth, false, true, Comment);
    Ports.push_back(Port);
    return Port;
  }

  void printModuleDecl(raw_ostream &OS) const;

  const PortVector &getPorts() const { return Ports; }

  typedef PortVector::iterator port_iterator;
  typedef PortVector::const_iterator const_port_iterator;

  port_iterator ports_begin() { return Ports.begin(); }
  const_port_iterator ports_begin() const { return Ports.begin(); }

  port_iterator ports_end() { return Ports.end(); }
  const_port_iterator ports_end() const { return Ports.end(); }

  void print(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const VModule *A) { return true; }
  static inline bool classof(const VASTNode *A) {
    return A->getASTType() == vastModule;
  }


  raw_ostream &getStateDeclBuffer(unsigned ind = 2) {
    return StateDecl.indent(ind);
  }

  std::string &getStateDeclStr() {
    return StateDecl.str();
  }

  raw_ostream &getSignalDeclBuffer(unsigned ind = 2) {
    return SignalDecl.indent(ind);
  }

  std::string &getSignalDeclStr() {
    return SignalDecl.str();
  }

  raw_ostream &getResetBlockBuffer(unsigned ind = 6) {
    return ResetBlock.indent(ind);
  }

  std::string &getResetBlockStr() {
    return ResetBlock.str();
  }

  raw_ostream &getSeqComputeBuffer(unsigned ind = 0) {
    return SeqCompute.indent(ind);
  }

  std::string &getSeqComputeStr() {
    return SeqCompute.str();
  }

  raw_ostream &getControlBlockBuffer(unsigned ind = 0) {
    return ControlBlock.indent(ind);
  }

  std::string &getControlBlockStr() {
    return ControlBlock.str();
  }

  raw_ostream &getDataPathBuffer(unsigned ind = 0) {
    return DataPath.indent(ind);
  }

  std::string &getDataPathStr() {
    return DataPath.str();
  }
};

class VLang : public ImmutablePass {
  const TargetData* TD;
  Mangler *Mang;
  const MCAsmInfo* TAsm;
  MCContext *TCtx;
  // For unname value
  DenseMap<const Value*, unsigned> AnonValueNumbers;
  unsigned NextAnonValueNumber;

  void clear() {
    AnonValueNumbers.clear();
  }

public:
  static char ID;
  explicit VLang() : ImmutablePass(ID), NextAnonValueNumber(0) {}

  ~VLang() {
    clear();
    delete Mang;
    delete TAsm;
    delete TCtx;
  }

  /// @name Value and Type printing
  //{
  /// @brief Get the name of Value, if the Value have no name, 
  ///       just create one for it.
  ///
  /// @param Operand The Value to get the name.
  ///
  /// @return The unique name of the Value.
  std::string GetValueName(const Value *Operand); 

  std::string printConstant(Constant *C);
  std::string printConstantInt(uint64_t value,unsigned bitwidth, bool isMinValue);

  static std::string printBitWitdh(unsigned BitWidth, int LowestBit = 0, 
    bool printOneBit = false);
  //}

  raw_ostream &comment(raw_ostream &ss) const;
  
  raw_ostream &moduleBegin(raw_ostream &ss, std::string &ModuleName);
  
  raw_ostream &endModuleDecl(raw_ostream &ss);

  raw_ostream &alwaysBegin(raw_ostream &ss, unsigned ind,
                          const std::string &Clk = "clk",
                          const std::string &ClkEdge = "posedge",
                          const std::string &Rst = "rstN",
                          const std::string &RstEdge = "negedge");

  raw_ostream &resetRegister(raw_ostream &ss, const std::string &Name,
                            unsigned BitWidth, unsigned InitVal = 0);

  
  raw_ostream &param(raw_ostream &ss, const std::string &Name,
                    unsigned BitWidth, unsigned Val);

  raw_ostream &declSignal(raw_ostream &ss, const std::string &Name,
                          unsigned BitWidth, unsigned Val,
                          bool isReg = true, bool isSigned = false,
                          const std::string &Term = ";");

  raw_ostream &alwaysEnd(raw_ostream &ss, unsigned ind);
  
  raw_ostream &switchCase(raw_ostream &ss, const std::string &StateName);
  
  raw_ostream &matchCase(raw_ostream &ss, const std::string &StateName);

  raw_ostream &ifBegin(raw_ostream &ss, const std::string &Condition);
  
  raw_ostream &ifElse(raw_ostream &ss, bool Begin = true);

  raw_ostream &begin(raw_ostream &ss);

  raw_ostream &end(raw_ostream &ss);
  
  raw_ostream &endSwitch(raw_ostream &ss);
  
  raw_ostream &endModule(raw_ostream &ss);

  virtual void initializePass();

  virtual void getAnalysisUsage(AnalysisUsage &AU) {
    AU.addRequired<TargetData>();
  }
};
} // end namespace

#endif
