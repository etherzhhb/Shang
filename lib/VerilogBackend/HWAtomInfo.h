//===------------- HWAtom.h - Translate LLVM IR to HWAtom  -------*- C++ -*-===//
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
// This file also define the HWAtomInfo class, which construction the HWAtom
// represent from LLVM IR.
//
//===----------------------------------------------------------------------===//
//

#ifndef VBE_HARDWARE_ATOMINFO_H
#define VBE_HARDWARE_ATOMINFO_H

#include "llvm/Support/MathExtras.h"

#include "vbe/HWAtom.h"

namespace llvm {
  class LoopInfo;
}
using namespace llvm;

namespace esyn {
/// @brief Hardware atom construction pass
///
class HWAtomInfo : public FunctionPass, public InstVisitor<HWAtomInfo> {

  /// @name InstVisitor interface
  //{
  friend class InstVisitor<HWAtomInfo>;
  void visitTerminatorInst(TerminatorInst &I);

  void visitPHINode(PHINode &I);
  void visitBinaryOperator(Instruction &I);
  void visitICmpInst(ICmpInst &I);
  void visitFCmpInst(FCmpInst &I){
    llvm_unreachable("Instruction not support yet!");
  }

  void visitCastInst (CastInst &I);
  void visitSelectInst(SelectInst &I);
  void visitCallInst (CallInst &I){
    llvm_unreachable("Instruction not support yet!");}
  void visitInlineAsm(CallInst &I){
    llvm_unreachable("Instruction not support yet!");
  }
  bool visitBuiltinCall(CallInst &I, Intrinsic::ID ID, bool &WroteCallee) {
    llvm_unreachable("Instruction not support yet!");
  }

  void visitAllocaInst(AllocaInst &I) {
    llvm_unreachable("Instruction not support yet!");
  }
  void visitLoadInst  (LoadInst   &I);
  void visitStoreInst (StoreInst  &I);
  void visitGetElementPtrInst(GetElementPtrInst &I);
  void visitVAArgInst (VAArgInst &I){
    llvm_unreachable("Instruction not support yet!");
  }

  void visitInsertElementInst(InsertElementInst &I){
    llvm_unreachable("Instruction not support yet!");
  }
  void visitExtractElementInst(ExtractElementInst &I){
    llvm_unreachable("Instruction not support yet!");
  }
  void visitShuffleVectorInst(ShuffleVectorInst &SVI){
    llvm_unreachable("Instruction not support yet!");
  }

  void visitInsertValueInst(InsertValueInst &I){
    llvm_unreachable("Instruction not support yet!");
  }
  void visitExtractValueInst(ExtractValueInst &I){
    llvm_unreachable("Instruction not support yet!");
  }

  void visitInstruction(Instruction &I) {
#ifndef NDEBUG
    errs() << "HWAtomInfo does not know about " << I;
#endif
    llvm_unreachable(0);
  }
  //}

  // Allocator
  BumpPtrAllocator HWAtomAllocator;

  // 
  FoldingSet<HWAtom> UniqiueHWAtoms;

  HWAtom *getConstant(Value &V);

  HWARegister *getRegister(Value &V, HWAtom *Using);

  HWAtom *getSigned(HWAtom *Using);

  HWAPreBind *getPreBind(Instruction &I, SmallVectorImpl<HWAtom*> &Deps,
                     size_t OpNum, HWResource &Res,  unsigned latency, 
                     unsigned ResInst = 0);
  HWAPreBind *getPreBind(Instruction &I, SmallVectorImpl<HWAtom*> &Deps,
                     HWResource &Res,  unsigned latency, 
                     unsigned ResInst = 0) {
    return getPreBind(I, Deps, I.getNumOperands(), Res, latency, ResInst);
  }

  HWAPostBind *getPostBind(Instruction &I, SmallVectorImpl<HWAtom*> &Deps,
    size_t OpNum, unsigned latency, enum HWResource::ResTypes OpClass);

  HWAPostBind *getPostBind(Instruction &I, SmallVectorImpl<HWAtom*> &Deps,
                       unsigned latency, enum HWResource::ResTypes OpClass) {
    return getPostBind(I, Deps, I.getNumOperands(), latency, OpClass);
  }

  HWAEntryRoot *getEntryRoot(BasicBlock *BB);

  typedef DenseMap<const Value*, HWAtom*> AtomMapType;
  AtomMapType ValueToHWAtoms;

  typedef DenseMap<const BasicBlock*, ExecStage*> StateMapType;
  StateMapType BBToStates;
  HWAtom *ControlRoot;
  std::set<HWAtom*> AtomsInCurState;

  void SetControlRoot(HWAtom *NewRoot) {
    ControlRoot = NewRoot;
  }

  HWAtom *getControlRoot() {
    return ControlRoot;
  }

  HWAtom *getAtomInState(Value &V, BasicBlock *BB) {
    // Is this not a instruction?
    if (!isa<Instruction>(V))
      return getConstant(V);

    // Now it is an Instruction
    Instruction &Inst = cast<Instruction>(V);
    if (BB == Inst.getParent())
      return getAtomFor(Inst);
    else
      return getEntryRoot(BB);
  }
  void addOperandDeps(Instruction &I, SmallVectorImpl<HWAtom*> &Deps) {
    BasicBlock *ParentBB = I.getParent();
    for (ReturnInst::op_iterator OI = I.op_begin(), OE = I.op_end();
        OI != OE; ++OI) {
      if (Instruction *OpI = dyn_cast<Instruction>(OI))
        // Restrict the dependences in the current state
        Deps.push_back(getAtomInState(*OpI, ParentBB));
      else if(!isa<BasicBlock>(OI)) // Ignore the basic Block.
        // Otherwise it is a constant
        Deps.push_back(getConstant(**OI));
    }
  }

  void clear();

  // The loop Info
  LoopInfo *LI;
  ResourceConfig *RC;
  // Total states
  unsigned totalCycle;

public:

  /// @name FunctionPass interface
  //{
  static char ID;
  HWAtomInfo() : FunctionPass(&ID), ControlRoot(0), LI(0), totalCycle(1) {}

  bool runOnFunction(Function &F);
  void releaseMemory();
  void getAnalysisUsage(AnalysisUsage &AU) const;
  virtual void print(raw_ostream &O, const Module *M) const;
  //}

  ExecStage &getStateFor(BasicBlock &BB) const {
    StateMapType::const_iterator At = BBToStates.find(&BB);
    assert(At != BBToStates.end() && "Can not get the State!");
    return  *(At->second);
  }

  HWAtom *getAtomFor(Value &V) const {
    AtomMapType::const_iterator At = ValueToHWAtoms.find(&V);
    if (At != ValueToHWAtoms.end())
      return  At->second;

    assert((isa<Constant>(V) || isa<Argument>(V))
      && "Only Constant missing!");
    return const_cast<HWAtomInfo*>(this)->getConstant(V);
  }

  unsigned getTotalCycle() const {
    return totalCycle;
  }

  unsigned getTotalCycleBitWidth() const {
    return Log2_32_Ceil(totalCycle);
  }

  void incTotalCycle() { ++totalCycle; }
};
} // end namespace
#endif
