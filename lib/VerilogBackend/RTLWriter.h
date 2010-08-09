//===------------- RTLWriter.h - HWAtom to RTL verilog  ---------*- C++ -*-===//
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
// This file define the RTLWriter pass, which write out HWAtom into RTL
// verilog form.
//
//===----------------------------------------------------------------------===//
#ifndef VBE_RTL_WRITER_H
#define VBE_RTL_WRITER_H

#include "llvm/Pass.h"
#include "llvm/Function.h"
#include "llvm/Instructions.h"
#include "llvm/Constants.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Module.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/Support/CFG.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/DerivedTypes.h"
#include "llvm/Target/Mangler.h"

#include "vbe/utils.h"
#include "vbe/ResourceConfig.h"
#include "vbe/HWAtom.h"

#include "VLang.h"

using namespace llvm;


namespace esyn {
class HWAtomInfo;

class RTLWriter : public FunctionPass {
  raw_ostream &Out;
  TargetData *TD;
  VLang *vlang;
  HWAtomInfo *HI;
  ResourceConfig *RC;
  // Buffers
  raw_string_ostream  ModDecl, StateDecl, SignalDecl, DataPath,
    ControlBlock, ResetBlock, SeqCompute;

  unsigned TotalFSMStatesBit, CurFSMStateNum;

  // Mapping used resouces to the using atoms
  typedef std::vector<HWAPreBind*> HWAPreBindVecTy;
  typedef std::map<HWFUnitID,HWAPreBindVecTy> ResourceMapType;

  ResourceMapType ResourceMap;

  std::string getSlotEnable(BasicBlock &BB, unsigned Slot);

  void emitFunctionSignature(Function &F);
  void emitCommonPort();
  void emitBasicBlock(BasicBlock &BB);

  // Resource
  void emitResources();

  template<class ResType>
  void emitResource(HWAPreBindVecTy &Atoms);

  template<class ResType>
  void emitResourceDecl(HWAPreBindVecTy &Atoms);

  template<class ResType>
  void emitResourceOp(HWAPreBind *A);

  template<class ResType>
  void emitResourceDefaultOp(HWFUnit FU);
  void opMemBus(HWAPreBind *PreBind);

  void opAddSub(HWAPreBind *PreBind);


  // Atoms
  void emitAtom(HWAtom *A);
  void emitPreBind(HWAPreBind *PreBind);
  void emitPostBind(HWAPostBind *PreBind);
  void emitWrReg(HWAWrReg *DR);
  void emitRdReg(HWARdReg *DR);

  std::set<const HWRegister*> UsedRegs;

  void emitAllRegisters();

  // Helper function for state end
  // Copy incoming value for Phi node.
  void emitPHICopiesForSucc(BasicBlock &CurBlock, BasicBlock &Succ,
                            unsigned ind = 0);

  void clear();
  
  std::string getAsOperand(Value *V, const std::string &postfix = "");
  std::string getAsOperand(HWEdge &E);
  std::string getAsOperand(HWAtom *A);
  std::string getAsOperand(HWRegister *R);
  static std::string getFURegisterName(HWFUnitID FUID);

  raw_ostream &getStateDeclBuffer() {
    return StateDecl.indent(2);
  }

  raw_ostream &getModDeclBuffer() {
    return ModDecl.indent(4);
  }

  raw_ostream &getSignalDeclBuffer() {
    return SignalDecl.indent(2);
  }

  raw_ostream &getResetBlockBuffer() {
    return ResetBlock.indent(6);
  }

  void emitNextFSMState(raw_ostream &ss, BasicBlock &BB);
  void emitNextMicroState(raw_ostream &ss, BasicBlock &BB,
                          const std::string &NewState);
  std::string computeSelfLoopEnable(FSMState *State);

  /// @name InstVisitor interface
  //{
  void visitReturnInst(HWAPostBind &A);
  void visitBranchInst(HWAPostBind &A);
  void visitSwitchInst(HWAPostBind &A){}
  void visitIndirectBrInst(HWAPostBind &A){}
  void visitInvokeInst(HWAPostBind &A) {
    llvm_unreachable("Lowerinvoke pass didn't work!");
  }

  void visitUnwindInst(HWAPostBind &A) {
    llvm_unreachable("Lowerinvoke pass didn't work!");
  }
  void visitUnreachableInst(HWAPostBind &A){}

  void visitPHINode(HWAPostBind &A) {}

  void visitBinaryOperator(HWAPostBind &A);
  void visitICmpInst(HWAPostBind &A);
  void visitFCmpInst(HWAPostBind &A){}

  void visitTruncInst(HWAPostBind &A);

  void visitExtInst (HWAPostBind &A);
  void visitZExtInst(HWAPostBind &A)      { visitExtInst(A); }
  void visitSExtInst(HWAPostBind &A)      { visitExtInst(A); }

  //
  void visitFPTruncInst(HWAPostBind &A)   { }
  void visitFPExtInst(HWAPostBind &A)     { }
  void visitFPToUIInst(HWAPostBind &A)    { }
  void visitFPToSIInst(HWAPostBind &A)    { }
  void visitUIToFPInst(HWAPostBind &A)    { }
  void visitSIToFPInst(HWAPostBind &A)    { }
  void visitPtrToIntInst(HWAPostBind &A)  { }
  void visitIntToPtrInst(HWAPostBind &A)  { }
  void visitBitCastInst(HWAPostBind &A)   { }

  void visitSelectInst(HWAPostBind &A);
  void visitCallInst (HWAPostBind &A){}
  void visitInlineAsm(HWAPostBind &A){}
  bool visitBuiltinCall(CallInst &I, Intrinsic::ID ID, bool &WroteCallee){}

  void visitAllocaInst(HWAPostBind &A) {}
  void visitLoadInst  (HWAPostBind &A){}
  void visitStoreInst (HWAPostBind &A){}
  void visitGetElementPtrInst(HWAPostBind &A);
  void visitVAArgInst (HWAPostBind &A){}

  void visitInsertElementInst(HWAPostBind &A){}
  void visitExtractElementInst(HWAPostBind &A){}
  void visitShuffleVectorInst(HWAPostBind &A){}

  void visitInsertValueInst(HWAPostBind &A){}
  void visitExtractValueInst(HWAPostBind &A){}

  void visitInstruction(HWAPostBind &A) {
    llvm_unreachable("Unknown instruction!");
  }


#define HANDLE_INST(NUM, OPCODE, CLASS) \
  void visit##OPCODE(HWAPostBind &A) { visit##CLASS(A); }
#include "llvm/Instruction.def"

  void visit(HWAPostBind &A) {
    switch (A.getOpcode()) {
    default: llvm_unreachable("Unknown instruction type encountered!");
      // Build the switch statement using the Instruction.def file...
#define HANDLE_INST(NUM, OPCODE, CLASS) \
    case Instruction::OPCODE: return visit##OPCODE(A);
#include "llvm/Instruction.def"
    }
  }
  //}

public:
  /// @name FunctionPass interface
  //{
  static char ID;
  explicit RTLWriter(raw_ostream &O)
    : FunctionPass(&ID), Out(O), TD(0), vlang(0), HI(0), RC(0),
    ModDecl(*(new std::string())),
    StateDecl(*(new std::string())),
    SignalDecl(*(new std::string())),
    DataPath(*(new std::string())),
    ControlBlock(*(new std::string())),
    ResetBlock(*(new std::string())),
    SeqCompute(*(new std::string())),
    TotalFSMStatesBit(0), CurFSMStateNum(0) {
  }
  ~RTLWriter();

  bool runOnFunction(Function &F);
  void releaseMemory() { clear(); }
  void getAnalysisUsage(AnalysisUsage &AU) const;
  virtual void print(raw_ostream &O, const Module *M) const;
  //}
};

} //end of namespace
#endif // h guard
