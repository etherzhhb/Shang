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

  SmallVector<HWAtom*, 32> Worklist(State->usetree_begin(),
                                    State->usetree_end());

  while(!Worklist.empty()) {
    HWAtom *A = Worklist.back();
    Worklist.pop_back();

    HWAOpInst *OI = dyn_cast<HWAOpInst>(A);
    if (OI == 0)
      continue;

    for (unsigned i = 0, e = OI->getInstNumOps(); i != e; ++i) {
      if (HWValDep *VD = dyn_cast<HWValDep>(&OI->getDep(i))) {
        Value *V = OI->getIOperand(i);
        if (VD->getDepType() == HWValDep::Import)
          // Insert the import node.
          OI->setDep(i, HI.getRdReg(State, OI, *V));
      }
    }

    DEBUG(OI->print(dbgs()));
    DEBUG(dbgs() << " Visited\n");

    std::vector<HWAtom *> Users(OI->use_begin(), OI->use_end());
    while (!Users.empty()) {
      HWAtom *Dst = Users.back();
      Users.pop_back();

      // Only insert register for normal edge.
      if (HWValDep *VD = dyn_cast<HWValDep>(Dst->getEdgeFrom(OI))) {
        if (VD->getDepType() != HWValDep::Normal)
          continue;
      } else
        continue;

      // We need to register the value if the value life through
      // several cycle. Or we need to keep the value until the computation
      // finish.
      if (Dst->getSlot() == OI->getFinSlot() && Dst->getLatency() == 0)
        continue;

      HWAWrReg *WrReg = HI.getWrReg(OI, Dst);

      DEBUG(dbgs() << "Insert ");
      DEBUG(WrReg->dump());
      DEBUG(dbgs() << "before ");
      DEBUG(Dst->dump());
      // Extend the life time of value by move the value to a register.
      Dst->replaceDep(OI, WrReg);
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

        DEBUG(dbgs() << " Registered\n");
        // Store the value to register.
        Exit->setDep(i, HI.getWrReg(SrcAtom, Exit));
      } else if (VD->getDepType() == HWValDep::SelfLoop) {
        DEBUG(dbgs() << "Visit value use by PHI: " << SrcAtom->getValue() << "\n");
        DEBUG(SrcAtom->dump());
        // It is ok to read the value at II slot even the the value is export
        // for exit blocks, because the value at the last iteration will not
        // be overwritten.
        unsigned IISlot = State->getIISlot();
        // Just read value from the atom is ok.
        if (SrcAtom->getFinSlot() == IISlot) {
          DEBUG(dbgs() << "Do not need register\n");
          State->updateSelfPHISrc(cast<Instruction>(V), SrcAtom);
          continue;
        }

        assert(!isa<HWAWrReg>(SrcAtom) && "Unexpected Register for phi node!");
        HWAWrReg *WR = HI.getWrReg(SrcAtom, Exit);
        State->updateSelfPHISrc(cast<Instruction>(V), WR);
        Exit->setDep(i, WR);
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
