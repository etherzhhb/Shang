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
  // Emit the exported register.
  HWAOpFU *Exit = State->getExitRoot();

  // Emit register for PHI incoming value.
  for (BasicBlock::iterator I = BB.begin(), E = BB.getFirstNonPHI();
       I != E; ++I) {
    PHINode *PN = cast<PHINode>(I);
    for (unsigned i = 0, e = PN->getNumIncomingValues(); i != e; ++i)
      (void)HI.getRegForValue(PN->getIncomingValue(i));
  }

  SmallVector<HWAtom*, 32> Worklist(State->begin(), State->end());

  while(!Worklist.empty()) {
    HWAtom *SrcAtom = Worklist.back();
    Worklist.pop_back();

    if (SrcAtom == State) continue;
    // Emit the register.
    if (HWALIReg *LI = dyn_cast<HWALIReg>(SrcAtom)) {
      Value *V = &LI->getValue();
      (void)HI.getRegForValue(V);
      DEBUG(V->dump());
      continue;
    }

    DEBUG(SrcAtom->print(dbgs()));
    DEBUG(dbgs() << " Visited\n");

    std::vector<HWAtom *> Users(SrcAtom->use_begin(), SrcAtom->use_end());
    while (!Users.empty()) {
      HWAtom *Dst = Users.back();
      Users.pop_back();

      // Only insert register for normal edge.
      if (HWValDep *VD = dyn_cast<HWValDep>(Dst->getEdgeFrom(SrcAtom))) {
        if (VD->getDepType() != HWValDep::Normal)
          continue;
      } else
        continue;

      // We need to register the value if the value life through
      // several cycle. Or we need to keep the value until the computation
      // finish.
      if (Dst->getSlot() == SrcAtom->getFinSlot())  {
        // Already registered by function unit register.
        if (HWAOpFU *OF = dyn_cast<HWAOpFU>(SrcAtom))
          if (OF->isBinded() && isa<HWAWrReg>(Dst))
            continue;

        // Do not need to register, the computation will finish in time.
        if (Dst->getLatency() == 0)
          continue;
        
      } else if (HWAWrReg *WR = dyn_cast<HWAWrReg>(SrcAtom)) {
        // Otherwise if the function unit register could hold the value untill
        // the computation finish.
        //assert(WR->writeFUReg()
        //  && "Expect write to function unit register!");
        if (WR->getFinSlot() == Dst->getFinSlot())          
          continue;
        else if (WR->getSlot() == Dst->getFinSlot()) {
          // Function unit reigster driving combintional logic.
          assert(Dst->getLatency() == 0 && Dst != Exit
                 && "Bad timing!");
          continue;
        }
      }

      HWAWrReg *WrReg = HI.getWrReg(SrcAtom);
      DEBUG(dbgs() << "---------------->Insert ");
      DEBUG(WrReg->dump());
      DEBUG(dbgs() << "before ");
      DEBUG(Dst->dump());
      // Extend the life time of value by move the value to a register.
      Dst->replaceDep(SrcAtom, WrReg);
    }
  }

  for (unsigned i = Exit->getInstNumOps(), e = Exit->getNumDeps(); i != e; ++i) {
    if (HWValDep *VD = dyn_cast<HWValDep>(&Exit->getDep(i))) {
      HWAtom *SrcAtom = VD->getSrc();
      Value *V = &SrcAtom->getValue();
      if (VD->getDepType() == HWValDep::Export) {

        // If we already emit the register, just skip it.
        if (HWAWrReg *WR = dyn_cast<HWAWrReg>(SrcAtom)) {
          if (!WR->writeFUReg())          
            continue;
        }

        DEBUG(dbgs() << " Registered\n");
        // Store the value to register.
        Exit->setDep(i, HI.getWrReg(SrcAtom));
      } else if (VD->getDepType() == HWValDep::PHI) {
        PHINode &PN = cast<PHINode>(SrcAtom->getValue());
        // PHINode define in other BB is just ok.
        if (PN.getParent() != &BB) continue;
        // Get the PHIAtom, and check if we break the anti dependence.
        // If so, preserve the anti dependence by copy the origin PHINode out.
        HWALIReg *PHIAtom = cast<HWALIReg>(HI.getAtomFor(PN));

        std::vector<HWAtom*> WorkList(PHIAtom->use_begin(),
                                      PHIAtom->use_end());
        HWAWrReg *NewWrReg = 0;
        while (!WorkList.empty()) {
          HWAtom *U = WorkList.back();
          WorkList.pop_back();
          // Anti-dependence preserved.
          if (U->getSlot() < SrcAtom->getSlot())
            continue;

          if (NewWrReg == 0) {
            unsigned Slot = SrcAtom->getSlot();
            HWRegister *Reg = HI.allocaRegister(PHIAtom->getBitWidth());
            NewWrReg = HI.getWrReg(PHIAtom, Reg, Slot);
          }
          U->replaceDep(PHIAtom, NewWrReg);
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
