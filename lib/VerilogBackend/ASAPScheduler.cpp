//===--- ASAPScheduler.cpp - The As Soon As possible scheduler  -*- C++ -*-===//
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
// This file implement the ScheduleDriver Pass, which run difference schedulers
// on a llvm function to schedule the Hardware atoms.
//
//===----------------------------------------------------------------------===//
//

#include "vbe/SchedulerBase.h"
#include "HWAtomInfo.h"
#include "HWAtomPasses.h"


#define DEBUG_TYPE "vbe-asap-schedule"
#include "llvm/Support/Debug.h"

using namespace llvm;
using namespace esyn;

namespace {
struct ASAPScheduler : public BasicBlockPass, public Scheduler {
  HWAtomInfo *HI;
  ResourceConfig *RC;
  static char ID;
  ASAPScheduler() : BasicBlockPass(&ID), Scheduler() {}
  bool runOnBasicBlock(BasicBlock &BB);
  void releaseMemory();
  void getAnalysisUsage(AnalysisUsage &AU) const;
  virtual void print(raw_ostream &O, const Module *M) const;
};
} //end namespace

char ASAPScheduler::ID = 0;

void ASAPScheduler::print(raw_ostream &O, const Module *M) const {

}

void ASAPScheduler::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<HWAtomInfo>();
  AU.addRequired<ResourceConfig>();
  AU.setPreservesAll();
}

bool ASAPScheduler::runOnBasicBlock(BasicBlock &BB) {
  HI = &getAnalysis<HWAtomInfo>();
  RC = &getAnalysis<ResourceConfig>();

  ExecStage &State = HI->getStateFor(BB);
  createAtomList(HI, BB);

  HWAVRoot &EntryRoot = State.getEntryRoot();
  HWAtom *ExitRoot = &State.getExitRoot();
  EntryRoot.scheduledTo(HI->getTotalCycle());
  //HI->incTotalCycle();
  // Schedule EntryRoot

  // TODO: Check if the atoms are empty
  while (!isOperationFinish(ExitRoot, HI->getTotalCycle())) {
    DEBUG(dbgs() << "======Cycle " << HI->getTotalCycle() << "\n");
    // Find all ready atoms

    // For each ready atoms
    while(HWAtom *ReadyAtom = getReadyAtoms(HI->getTotalCycle())){
      if (HWAPreBind *PreBind = dyn_cast<HWAPreBind>(ReadyAtom)) {
        HWResource::ResIdType ResId = PreBind->getResourceId();
        // Is the resource available?
        unsigned readyCyc = getReadyCycle(ResId);
        // This may cause dead loop when there are only ready atoms for
        // unready resource.
        if (readyCyc > HI->getTotalCycle())
          continue;
        
        //
        rememberReadyCycle(ResId, HI->getTotalCycle() + PreBind->getLatency());
      }
      ReadyAtom->scheduledTo(HI->getTotalCycle());

      DEBUG(ReadyAtom->print(dbgs()));
      DEBUG(dbgs() << " scheduled\n");
      // Remove the exist atom
      removeFromList(ReadyAtom);
    }

    // Advance the state
    HI->incTotalCycle();
  }
  //// schedule the state end;

  DEBUG(State.print(dbgs()));
  return false;
}

void ASAPScheduler::releaseMemory() {
  clearSchedulerBase();
}


Pass *esyn::createASAPSchedulePass() {
  return new ASAPScheduler();
}
