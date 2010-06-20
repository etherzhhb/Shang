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

  void visitReturnInst(ReturnInst &I) {
    visitTerminatorInst(I);
  }
  void visitBranchInst(BranchInst &I) {
    visitTerminatorInst(I);
  }
  void visitSwitchInst(SwitchInst &I){
    llvm_unreachable("Instruction not support yet!");
  }
  void visitIndirectBrInst(IndirectBrInst &I){
    llvm_unreachable("Instruction not support yet!");
  }
  void visitInvokeInst(InvokeInst &I) {
    llvm_unreachable("HWAtomFilterPass pass didn't work!");
  }

  void visitUnwindInst(UnwindInst &I) {
    llvm_unreachable("HWAtomFilterPass pass didn't work!");
  }
  void visitUnreachableInst(UnreachableInst &I){}

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

  HWAState *getState(BasicBlock &BB);

  HWAStateEnd *getStateEnd(TerminatorInst &Term,
    SmallVectorImpl<HWAtom*> &Deps);

  HWARegister *getRegister(Instruction &I, HWAtom *Using);

  HWASigned *getSigned(HWAtom *Using);

  HWAWireOp *getWireOp(Instruction &I, HWAtom *Using);

  HWAOpRes *getOpRes(Instruction &I, SmallVectorImpl<HWAtom*> &Deps, 
                     HWResource &Res, unsigned ResInst = 0);

  // Maping Instruction to HWAtoms
  typedef DenseMap<const Instruction*, HWAtom*> AtomMapType;
  AtomMapType InstToHWAtoms;



  HWAtom *getAtomFor(Instruction &Inst) const {
    AtomMapType::const_iterator At = InstToHWAtoms.find(&Inst);
    assert(At != InstToHWAtoms.end() && "Can not get the Atom!");
    return  At->second;
  }

  typedef DenseMap<const BasicBlock*, HWAState*> StateMapType;
  StateMapType BBToStates;
  HWAtom *ControlRoot;
  HWAState *CurState;

  HWAState *getCurState() {
    return CurState;
  }

  void updateStateTo(BasicBlock &BB) {
    CurState = getState(BB);
    BBToStates.insert(
      std::make_pair<const BasicBlock*, HWAState*>(&BB, CurState));
    SetControlRoot(CurState);
  }

  HWAtom *getControlRoot() {
    return ControlRoot;
  }

  HWAtom *getAtomInState(Instruction &Inst, BasicBlock *BB) {
    if (BB == Inst.getParent())
      return getAtomFor(Inst);
    else
      // Create the register
      return getRegister(Inst, CurState);
  }
  void addOperandDeps(Instruction &I, SmallVectorImpl<HWAtom*> &Deps) {
    BasicBlock *ParentBB = I.getParent();
    HWAState &CurState = getStateFor(*ParentBB);
    for (ReturnInst::op_iterator OI = I.op_begin(), OE = I.op_end();
      OI != OE; ++OI) {
        if (Instruction *OpI = dyn_cast<Instruction>(OI))
          // Restrict the dependences in the current state
          Deps.push_back(getAtomInState(*OpI, ParentBB));
    }
  }

  void SetControlRoot(HWAtom *NewRoot) {
    ControlRoot = NewRoot;
  }

  void clear();

  // The loop Info
  LoopInfo *LI;
  HWResTable RT;
public:

  /// @name FunctionPass interface
  //{
  static char ID;
  explicit HWAtomInfo(ResourceConfig &RC)
    : FunctionPass(&ID), ControlRoot(0), CurState(0), LI(0), RT(RC) {}

  explicit HWAtomInfo();

  HWAState &getStateFor(BasicBlock &BB) const {
    StateMapType::const_iterator At = BBToStates.find(&BB);
    assert(At != BBToStates.end() && "Can not get the State!");
    return  *(At->second);
  }

  bool runOnFunction(Function &F);
  void releaseMemory();
  void getAnalysisUsage(AnalysisUsage &AU) const;
  virtual void print(raw_ostream &O, const Module *M) const;
  //}
};
} // end namespace
#endif
