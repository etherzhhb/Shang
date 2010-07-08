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


  SmallVector<HWAValDep*, 32> NewRegs;

  while(!Worklist.empty()) {
    if (HWAValDep* D = dyn_cast<HWAValDep>(Worklist.back())) {
      NewRegs.push_back(D);
      Worklist.pop_back();
      continue;
    }
    

    HWAOpInst *A = dyn_cast<HWAOpInst>(Worklist.back());
    Worklist.pop_back();
    
    if (A == 0)
      continue;

    DEBUG(A->print(dbgs()));
    DEBUG(dbgs() << " Visited\n");

    for (unsigned i = 0, e = A->getInstNumOps(); i != e; ++i) {
      HWAtom *dep = A->getOperand(i);
      Value &V = *A->getIOperand(i);

      bool isSigned = (((A->getOpcode() == Instruction::AShr) && (i == 0))
                        || ((A->getOpcode() == Instruction::ICmp)
                        && (A->getInst<ICmpInst>().isSigned())));

      if (HWAOpInst *DI = dyn_cast<HWAOpInst>(dep)) {
        if (DI->getLatency() != 0) {
          DEBUG(DI->print(dbgs()));
          DEBUG(dbgs() << " Registered\n");
          HWAValDep *R = HI.getValDepEdge(DI, V, HI.getRegNumForLiveVal(V), isSigned);
          NewRegs.push_back(R);
          A->setDep(i, R);
        }
      }
    }
  }

  while (!NewRegs.empty()) {
    HWAValDep *D = NewRegs.back();
    NewRegs.pop_back();
    D->scheduledTo(D->getSrc()->getSlot() + D->getSrc()->getLatency());
  }
  
  HWAOpInst &Exit = State.getExitRoot();
  if (Exit.getInstNumOps()) {
    HWAValDep *D = cast<HWAValDep>(Exit.getOperand(0));
    Value &V = *Exit.getIOperand(0);
    // Create a register for condition.
    if (D->getSlot() != Exit.getSlot()) {
      HWAValDep *R = HI.getValDepEdge(D->getSrc(), V, HI.getRegNumForLiveVal(V));
      Exit.setDep(0, R);
      R->scheduledTo(D->getSlot());
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
