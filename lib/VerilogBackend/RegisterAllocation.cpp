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
// This file implement the register allocation pass.
//
//===----------------------------------------------------------------------===//

#include "HWAtomPasses.h"
#include "HWAtomInfo.h"

#define DEBUG_TYPE "vbe-reg-alloca"
#include "llvm/Support/Debug.h"

using namespace llvm;
using namespace esyn;

namespace {
struct RegAllocation : public BasicBlockPass {
  static char ID;
  RegAllocation() : BasicBlockPass(&ID) {}
  bool runOnBasicBlock(BasicBlock &BB);
  void getAnalysisUsage(AnalysisUsage &AU) const;
};
}

bool RegAllocation::runOnBasicBlock(BasicBlock &BB) {
  HWAtomInfo &HI = getAnalysis<HWAtomInfo>();
  FSMState *State = HI.getStateFor(BB);

  // Emit the operand of PHINode.
  for (BasicBlock::iterator II = BB.begin(), IE = BB.getFirstNonPHI();
      II != IE; ++II) {
    PHINode *PN = cast<PHINode>(II);
    for (unsigned i = 0, e = PN->getNumIncomingValues(); i != e; ++i) {
      Value *IV = PN->getIncomingValue(i);
      FSMState *InStage = HI.getStateFor(*PN->getIncomingBlock(i));


      //if (InStage->getPHISrc(IV) != 0)
      //  continue;

      //if (isa<Constant>(IV))
      //  continue;
      //
      //// We may read the value from function unit register.
      //if (isa<Instruction>(IV)) {
      //  HWAtom *A = HI.getAtomFor(*IV);
      //  if (HWAPreBind *PB = dyn_cast<HWAPreBind>(A)) {
      //    HWAtom *Use = PB->use_back();
      //    if (HWAWrReg *WR = dyn_cast<HWAWrReg>(Use))
      //      if (WR->getFinSlot() == lastSlot) {
      //        InStage->updatePHISrc(IV, WR->getReg());
      //        continue;
      //      }
      //  }
      //}
      //HWRegister *IR = HI.getRegForValue(IV, lastSlot, lastSlot);
      //InStage->updatePHISrc(IV, IR);
    }
  }

  SmallVector<HWAtom*, 32> Worklist(State->usetree_begin(),
                                    State->usetree_end());

  while(!Worklist.empty()) {
    HWAtom *A = Worklist.back();
    Worklist.pop_back();

    HWAOpInst *OI = dyn_cast<HWAOpInst>(A);
    if (OI == 0)
      continue;

    DEBUG(OI->print(dbgs()));
    DEBUG(dbgs() << " Visited\n");

    std::vector<HWAtom *> Users(OI->use_begin(), OI->use_end());
    while (!Users.empty()) {
      HWAtom *Dst = Users.back();
      Users.pop_back();
    
      // We need to register the value if the value life through
      // several cycle. Or we need to keep the value until the computation
      // finish.
      if (Dst->getSlot() == OI->getFinSlot() && Dst->getLatency() == 0)
        continue;

      // Extend the life time of value by move the value to a register.
      Dst->replaceDep(OI, HI.getWrReg(OI, Dst));
    }

    for (unsigned i = 0, e = OI->getNumDeps(); i != e; ++i) {
      if (HWValDep *VD = dyn_cast<HWValDep>(&OI->getDep(i))) {
        Value *V = OI->getIOperand(i);
        if (VD->getDepType() == HWValDep::Import)
          // Insert the import node.
          OI->setDep(i, HI.getRdReg(State, OI, *V));
      }
    }
  }
  
  // Emit the exported register.
  HWAOpInst *Exit = State->getExitRoot();
  for (unsigned i = Exit->getInstNumOps(), e = Exit->getNumDeps(); i != e; ++i) {
    if (HWValDep *VD = dyn_cast<HWValDep>(&Exit->getDep(i))) {
      HWAtom *SrcAtom = VD->getSrc();
      Value *V = &SrcAtom->getValue();
      if (VD->getDepType() == HWValDep::Export) {

        // If we already emit the register, just skip it.
        if (isa<HWAWrReg>(SrcAtom))
            continue;
      } else if (VD->getDepType() == HWValDep::PHI) {
        unsigned lastSlot = State->haveSelfLoop() ? State->getIISlot()
                                                  : State->getEndSlot();
        // Just read value from the atom is ok.
        if (SrcAtom->getFinSlot() == lastSlot) {
          State->updatePHISrc(cast<Instruction>(V), SrcAtom);
          continue;
        }

        assert(!isa<HWAWrReg>(SrcAtom) && "Unexpected Register for phi node!");
      }


      DEBUG(SrcAtom->print(dbgs()));
      DEBUG(dbgs() << " Registered for export.\n");
      // Store the value to register.
      HWAWrReg *WR = HI.getWrReg(SrcAtom, Exit);
      Exit->setDep(i, WR);
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
