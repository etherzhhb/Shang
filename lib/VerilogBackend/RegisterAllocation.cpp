//===-------- RegisterAllocation.cpp - Allocation registers -----*- C++ -*-===//
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

#define DEBUG_TYPE "vbe-reg-alloca"
#include "llvm/Support/Debug.h"

using namespace llvm;
using namespace esyn;

namespace {
struct RegAllocation : public FunctionPass {
  static char ID;
  explicit RegAllocation() : FunctionPass(&ID) {}
  bool runOnFunction(Function &F);
  bool runOnBasicBlock(BasicBlock &BB, HWAtomInfo &HI);
  void getAnalysisUsage(AnalysisUsage &AU) const;
};
}

bool RegAllocation::runOnFunction(Function &F) {
  HWAtomInfo &HI = getAnalysis<HWAtomInfo>();

  for (Function::iterator I = F.begin(), E = F.end(); I != E; ++I)
    runOnBasicBlock(*I, HI);

  return false;
}

bool RegAllocation::runOnBasicBlock(BasicBlock &BB, HWAtomInfo &HI) {
  FSMState &State = HI.getStateFor(BB);
  HWAVRoot *EntryRoot = &State.getEntryRoot();

  SmallVector<HWAtom*, 32> Worklist(usetree_iterator::begin(EntryRoot),
                                    usetree_iterator::end(EntryRoot));

  while(!Worklist.empty()) {
    HWAOpInst *A = dyn_cast<HWAOpInst>(Worklist.back());
    Worklist.pop_back();
    
    if (A == 0)
      continue;

    DEBUG(A->print(dbgs()));
    DEBUG(dbgs() << " Visited\n");

    for (unsigned i = 0, e = A->getNumDeps(); i != e; ++i)
      if (HWValDep *VD = dyn_cast<HWValDep>(A->getDep(i)))
        if (HWAOpInst *DI = dyn_cast<HWAOpInst>(VD->getDagSrc())) {
          if (DI->getLatency() != 0) {
            Value &V = DI->getValue();
            DEBUG(DI->print(dbgs()));
            DEBUG(dbgs() << " Registered\n");
            HWAWrReg *DR = HI.getDrvReg(DI, HI.getRegNumForLiveVal(V));

            A->setDep(i, DR);
          }
        }
  }

  return false;
}

void RegAllocation::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<HWAtomInfo>();
  AU.setPreservesAll();
}

char RegAllocation::ID = 0;

Pass *esyn::createRegisterAllocationPass() {
  return new RegAllocation();
}
