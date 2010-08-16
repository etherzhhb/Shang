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
  HWAOpInst *Exit = State->getExitRoot();

  SmallVector<HWAtom*, 32> Worklist(State->usetree_begin(),
                                    State->usetree_end());

  while(!Worklist.empty()) {
    HWAtom *SrcAtom = Worklist.back();
    Worklist.pop_back();

    if (SrcAtom == State) continue;
    
    if (HWAOpInst *OI = dyn_cast<HWAOpInst>(SrcAtom)) {
      for (unsigned i = 0, e = OI->getInstNumOps(); i != e; ++i) {
        if (HWValDep *VD = dyn_cast<HWValDep>(&OI->getDep(i))) {
          Value *V = OI->getIOperand(i);
          if (VD->getDepType() == HWValDep::Import)
            // Insert the import node.
            OI->setDep(i, HI.getRdReg(State, OI, *V));
        }
      }
    }

    DEBUG(SrcAtom->print(dbgs()));
    DEBUG(dbgs() << " Visited\n");

    // Already registered.
    if (isa<HWAPreBind>(SrcAtom))
      continue;


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
        // Value registered.
        // FIXME: the function unit register only valid for one cycle, but its
        // user may require the register stable for mutiple cycles
        // (i.e. multi cycle function unit.)

        // Do not need to register, the computation will finish in time.
        if (Dst->getLatency() == 0)
          continue;
        
      } else if (HWAWrReg *WR = dyn_cast<HWAWrReg>(SrcAtom)) {
        // Otherwise if the function unit register could hold the value untill
        // the computation finish.
        assert(WR->writeFUReg()
          && "Expect write to function unit register!");
        if (WR->getFinSlot() == Dst->getFinSlot())          
          continue;
        else if (WR->getSlot() == Dst->getFinSlot()) {
          // Function unit reigster driving combintional logic.
          assert(Dst->getLatency() == 0 && Dst != Exit
                 && "Bad timing!");
          continue;
        }
      }

      HWAWrReg *WrReg = HI.getWrReg(SrcAtom, Dst);
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
        if (isa<HWAWrReg>(SrcAtom))
            continue;

        DEBUG(dbgs() << " Registered\n");
        // Store the value to register.
        Exit->setDep(i, HI.getWrReg(SrcAtom, Exit));
      }
    }
  }

  // Foreach PHINode in succ.
  for (succ_iterator SI = succ_begin(&BB), SE = succ_end(&BB); SI != SE; ++SI){
    BasicBlock *SuccBB = *SI;
    for (BasicBlock::iterator II = SuccBB->begin(),
        IE = SuccBB->getFirstNonPHI(); II != IE; ++II) {
      PHINode *PN = cast<PHINode>(II);
      HWValDep *VD = State->getPHIEdge(PN);
      // If the edge had been ignored.
      if (VD == 0) continue;
      
      HWAtom *SrcAtom = VD->getSrc();
      Value *V = &SrcAtom->getValue();
      DEBUG(dbgs() << "Visit value: " << *V << "use by PHI: " << *PN << "\n");
      DEBUG(SrcAtom->dump());
      DEBUG(dbgs() << "At slot " << SrcAtom->getSlot() << "\n");
      unsigned lastSlot = (SuccBB == &BB) ? State->getIISlot()
                                          : State->getEndSlot();

      // Just read value from the atom is ok.
      if (SrcAtom->getFinSlot() == lastSlot) {
        DEBUG(dbgs() << "Do not need register\n");
        continue;
      }
      // FIXME: Create read atom for argument or PHINode as operand of PHINode.
      // Create register for PHINode.
      assert((!isa<HWAWrReg>(SrcAtom) || cast<HWAWrReg>(SrcAtom)->writeFUReg())
             && "Unexpected Register for phi node!");
      HWAWrReg *WR = HI.getWrReg(SrcAtom, Exit);
      DEBUG(dbgs() << "Registered by:\n");
      DEBUG(WR->dump());
      Exit->replaceDep(SrcAtom, WR);
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
