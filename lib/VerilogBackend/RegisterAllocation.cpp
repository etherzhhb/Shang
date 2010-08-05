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

  // Allocate register for argument.
  //for (Function::arg_iterator I = F.arg_begin(), E = F.arg_end();
  //    I != E; ++I) {
  //  Argument *Arg = I;
  //  HI.getRegForValue(Arg, 1, 1);
  //}

  for (Function::iterator I = F.begin(), E = F.end(); I != E; ++I)
    runOnBasicBlock(*I, HI);

  return false;
}

bool RegAllocation::runOnBasicBlock(BasicBlock &BB, HWAtomInfo &HI) {
  FSMState &State = HI.getStateFor(BB);
  HWAVRoot *EntryRoot = &State.getEntryRoot();

  // Emit the operand of PHINode.
  for (BasicBlock::iterator II = BB.begin(), IE = BB.getFirstNonPHI();
      II != IE; ++II) {
    PHINode *PN = cast<PHINode>(II);
    for (unsigned i = 0, e = PN->getNumIncomingValues(); i != e; ++i) {
      Value *IV = PN->getIncomingValue(i);
      FSMState &IncomingStage = HI.getStateFor(*PN->getIncomingBlock(i));
      if (isa<Instruction>(IV) && !IncomingStage.getLiveOutRegAtTerm(IV)) {
        HWReg *IR = HI.getRegForValue(IV, EntryRoot->getSlot(),
                                      EntryRoot->getSlot());
        IncomingStage.updateLiveOutReg(IV, IR);
      }
    }
  }

  SmallVector<HWAtom*, 32> Worklist(usetree_iterator::begin(EntryRoot),
                                    usetree_iterator::end(EntryRoot));

  while(!Worklist.empty()) {
    HWAOpInst *A = dyn_cast<HWAOpInst>(Worklist.back());
    Worklist.pop_back();
    
    if (A == 0)
      continue;

    DEBUG(A->print(dbgs()));
    DEBUG(dbgs() << " Visited\n");

    for (unsigned i = 0, e = A->getInstNumOps(); i != e; ++i) {
      if (HWValDep *VD = dyn_cast<HWValDep>(A->getDep(i))) {
        Value *V = A->getIOperand(i);
        if (VD->isImport()) {
          // Insert the import node.
          HWAVRoot *Root = cast<HWAVRoot>(VD->getDagSrc());
          HWReg *R = HI.getRegForValue(V, Root->getSlot(), A->getSlot());
          // Update the live out value.
          if (!State.getLiveOutRegAtTerm(V))
            State.updateLiveOutReg(V, R);

          HWAImpStg *ImpStg = HI.getImpStg(Root, R, *V);
          A->setDep(i, ImpStg);
        } else if (HWAOpInst *DI = dyn_cast<HWAOpInst>(VD->getDagSrc())) {
          if (DI->getLatency() != 0) {
            DEBUG(DI->print(dbgs()));
            DEBUG(dbgs() << " Registered\n");
            // Store the value to register.
            HWReg *R = HI.getRegForValue(V, DI->getFinSlot(), A->getSlot());
            HWAWrStg *WR = HI.getWrStg(DI, R);
            A->setDep(i, WR);
          }
        } else if (HWAWrStg *WrStg = dyn_cast<HWAWrStg>(VD->getDagSrc())) {
          // Move the value out of the Function unit register.
          assert(WrStg->getReg()->isFuReg()
                 && "Only Expect function unit register!");
          if (WrStg->getReg()->getEndSlot() < A->getSlot()) {
            DEBUG(WrStg->print(dbgs()));
            DEBUG(dbgs() << " extended\n");
            HWReg *R = HI.getRegForValue(V, WrStg->getFinSlot(), A->getSlot());
            HWAWrStg *WR = HI.getWrStg(WrStg, R);
            A->setDep(i, WR);
          }
        }
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
