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

  HWAState *getState(BasicBlock &BB);

  HWAtom *getConstant(Value &V);

  HWARegister *getRegister(Instruction &I, HWAtom *Using);

  HWAtom *getSigned(HWAtom *Using);

  HWAOpRes *getOpRes(Instruction &I, SmallVectorImpl<HWAtom*> &Deps,
                     size_t OpNum, HWResource &Res,  unsigned latency, 
                     unsigned ResInst = 0);
  HWAOpRes *getOpRes(Instruction &I, SmallVectorImpl<HWAtom*> &Deps,
                     HWResource &Res,  unsigned latency, 
                     unsigned ResInst = 0) {
    return getOpRes(I, Deps, I.getNumOperands(), Res, latency, ResInst);
  }

  HWAOpInst *getOpInst(Instruction &I, SmallVectorImpl<HWAtom*> &Deps,
                       size_t OpNum, unsigned latency);

  HWAOpInst *getOpInst(Instruction &I, SmallVectorImpl<HWAtom*> &Deps,
                       unsigned latency) {
    return getOpInst(I, Deps, I.getNumOperands(), latency);
  }

  typedef DenseMap<const Value*, HWAtom*> AtomMapType;
  AtomMapType ValueToHWAtoms;

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

  HWAtom *getAtomInState(Value &V, BasicBlock *BB) {
    // Is this not a instruction?
    if (!isa<Instruction>(V))
      return getConstant(V);

    // Now it is an Instruction
    Instruction &Inst = cast<Instruction>(V);
    if (BB == Inst.getParent())
      return getAtomFor(Inst);
    else
      // Create the register
      // FIXME: Return the state and then Reassign the real register
      // After schedule
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
      else 
        // Otherwise it is a constant
        Deps.push_back(getConstant(**OI));
    }
  }

  void SetControlRoot(HWAtom *NewRoot) {
    ControlRoot = NewRoot;
  }

  void clear();

  // The loop Info
  LoopInfo *LI;
  HWResTable RT;

  // Total states
  unsigned totalCycle;

public:

  /// @name FunctionPass interface
  //{
  static char ID;
  explicit HWAtomInfo(ResourceConfig &RC)
    : FunctionPass(&ID), ControlRoot(0), CurState(0), LI(0), RT(RC), totalCycle(1) {}

  explicit HWAtomInfo();

  bool runOnFunction(Function &F);
  void releaseMemory();
  void getAnalysisUsage(AnalysisUsage &AU) const;
  virtual void print(raw_ostream &O, const Module *M) const;
  //}

  HWAState &getStateFor(BasicBlock &BB) const {
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

  typedef HWResTable::iterator resource_iterator;
  typedef HWResTable::const_iterator const_resource_iterator;

  resource_iterator resource_begin() { return RT.begin(); }
  const_resource_iterator resource_begin() const { return RT.begin(); }

  resource_iterator resource_end() { return RT.end(); }
  const_resource_iterator resource_end() const { return RT.end(); }
};
} // end namespace
#endif
