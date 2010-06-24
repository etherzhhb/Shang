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

class RTLWriter : public FunctionPass, public InstVisitor<RTLWriter> {
  raw_ostream &Out;
  TargetData *TD;
  VLang *vlang;
  HWAtomInfo *HI;

  // Buffers
  raw_string_ostream  ModDecl, StateDecl, SignalDecl, DataPath,
    ControlBlock, ResetBlock;

  void emitFunctionSignature(const Function &F);
  void emitCommonPort();
  void emitBasicBlock(BasicBlock &BB);

  // Resource
  void emitResources(HWResource &Resource);
  void emitMemBus(HWMemBus &MemBus);

  // Atoms
  void emitRegister(HWARegister *Register);
  void emitOpRes(HWAOpRes *OpRes);
  //
  void emitOpInst(HWAOpInst *OpRes);
  void emitStateEnd(HWAStateEnd *StateEnd);
  // Helper function for state end
  // Copy incoming value for Phi node.
  void emitPHICopiesForSucc(BasicBlock &CurBlock, BasicBlock &Succ,
                            unsigned ind = 0);

  void clear();

  std::string getAsOperand(Value *V);
  std::string getAsOperand(HWAtom &A);


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

  void emitNextState(raw_ostream &ss, BasicBlock &BB, unsigned offset = 0);

  /// @name InstVisitor interface
  //{
  friend class InstVisitor<RTLWriter>;

  void visitReturnInst(ReturnInst &I);
  void visitBranchInst(BranchInst &I);
  void visitSwitchInst(SwitchInst &I){}
  void visitIndirectBrInst(IndirectBrInst &I){}
  void visitInvokeInst(InvokeInst &I) {
    llvm_unreachable("Lowerinvoke pass didn't work!");
  }

  void visitUnwindInst(UnwindInst &I) {
    llvm_unreachable("Lowerinvoke pass didn't work!");
  }
  void visitUnreachableInst(UnreachableInst &I){}

  void visitPHINode(PHINode &I);
  void visitBinaryOperator(Instruction &I){}
  void visitICmpInst(ICmpInst &I);
  void visitFCmpInst(FCmpInst &I){}

  void visitCastInst (CastInst &I);
  void visitSelectInst(SelectInst &I){}
  void visitCallInst (CallInst &I){}
  void visitInlineAsm(CallInst &I){}
  bool visitBuiltinCall(CallInst &I, Intrinsic::ID ID, bool &WroteCallee){}

  void visitAllocaInst(AllocaInst &I){}
  void visitLoadInst  (LoadInst   &I){}
  void visitStoreInst (StoreInst  &I){}
  void visitGetElementPtrInst(GetElementPtrInst &I){}
  void visitVAArgInst (VAArgInst &I){}

  void visitInsertElementInst(InsertElementInst &I){}
  void visitExtractElementInst(ExtractElementInst &I){}
  void visitShuffleVectorInst(ShuffleVectorInst &SVI){}

  void visitInsertValueInst(InsertValueInst &I){}
  void visitExtractValueInst(ExtractValueInst &I){}

  void visitInstruction(Instruction &I) {
#ifndef NDEBUG
    errs() << "C Writer does not know about " << I;
#endif
    llvm_unreachable(0);
  }
  //}

public:
  /// @name FunctionPass interface
  //{
  static char ID;
  explicit RTLWriter(raw_ostream &O)
    : FunctionPass(&ID), Out(O), TD(0), vlang(0), HI(0),
    ModDecl(*(new std::string())),
    StateDecl(*(new std::string())),
    SignalDecl(*(new std::string())),
    DataPath(*(new std::string())),
    ControlBlock(*(new std::string())),
    ResetBlock(*(new std::string())){
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
