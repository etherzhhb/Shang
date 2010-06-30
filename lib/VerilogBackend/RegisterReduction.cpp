//===----- RegisterReduction.cpp - Reduce unnecessary registers --*- C++ -*-===//
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
// This file implement a pass that reduce unnecessary registers
//
//===----------------------------------------------------------------------===//

#include "HWAtomPasses.h"
#include "HWAtomInfo.h"

#define DEBUG_TYPE "vbe-reg-reduction"
#include "llvm/Support/Debug.h"

using namespace llvm;
using namespace esyn;

namespace {
struct RegReduction : public FunctionPass {
  static char ID;
  explicit RegReduction() : FunctionPass(&ID) {}
  bool runOnFunction(Function &F);
  bool runOnBasicBlock(BasicBlock &BB);
  void getAnalysisUsage(AnalysisUsage &AU) const;
};
}

bool RegReduction::runOnFunction(Function &F) {
  for (Function::iterator I = F.begin(), E = F.end(); I != E; ++I)
    runOnBasicBlock(*I);
  return false;
}

bool RegReduction::runOnBasicBlock(BasicBlock &BB) {
  HWAtomInfo &HI = getAnalysis<HWAtomInfo>();
  ExecStage &State = HI.getStateFor(BB);
  HWAEntryRoot *EntryRoot = &State.getEntryRoot();
  // For each atom
  for (ExecStage::iterator I = State.begin(), E = State.end(); I != E; ++I) {
    if (HWAOpInst *A = dyn_cast<HWAOpInst>(*I)) {
      Instruction &Inst = A->getInst<Instruction>();
      for (unsigned i = 0, e = A->getInstNumOps(); i != e; ++i) {
        // Remove the false dependence for live in registers
        if (A->getDep(i) == EntryRoot)
          A->setDep(i, HI.getAtomFor(*Inst.getOperand(i)));
        
        // For all register
        while (HWARegister *R = dyn_cast<HWARegister>(A->getOperand(i))) {
          // Only remove unnecessary register level for
          // "inline" operation
          // FIXME: if (not compute in datapath)
          if ((isa<HWAPreBind>(A) || A->getLatency() == 0)
                   && R->getSlot() == A->getSlot())
            A->setDep(i, R->getRefVal());
          else // This is just a normal register
            break;
        }
      }
    } else if (HWASigned *S = dyn_cast<HWASigned>(*I)) {
      if (S->getRefVal() == EntryRoot)
        S->setDep(0, HI.getAtomFor(S->getValue()));

      while (HWARegister *R = dyn_cast<HWARegister>(S->getRefVal()))
        S->setDep(0, R->getRefVal());
    }
  }
  
  return false;
}

void RegReduction::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<HWAtomInfo>();
  AU.setPreservesAll();
}

char RegReduction::ID = 0;

Pass *esyn::createRegisterReductionPass() {
  return new RegReduction();
}
