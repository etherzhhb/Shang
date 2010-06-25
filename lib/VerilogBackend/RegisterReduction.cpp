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
  HWAState &State = HI.getStateFor(BB);
  // For each atom
  for (HWAState::iterator I = State.begin(), E = State.end(); I != E; ++I) {
    if (HWASchedable *A = dyn_cast<HWASchedable>(*I)) {
      Instruction &Inst = A->getInst<Instruction>();

      for (unsigned i = 0, e = A->getEffectiveNumOps(); i != e; ++i)
        // For all register
        // FIXME: use get operand! but this fail on terminator
        while (HWARegister *R = dyn_cast<HWARegister>(A->getDep(i))) {
          // Remove the dummy registers
          if (R->isDummy())
            A->setDep(i, HI.getAtomFor(R->getValue()));
            // delete R?
          // Remove unnecessary register level
          else if (R->getSlot() == A->getSlot())
            A->setDep(i, R->getDVal());
          else // This is just a normal register
            break;
        }
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
