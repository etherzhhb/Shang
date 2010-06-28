//===------- ScheduleDriver.cpp - The Scheduler driver pass  ----*- C++ -*-===//
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

#include "ScheduleDriver.h"
#include "HWAtomPasses.h"

#include "llvm/Analysis/LoopInfo.h"
#include "llvm/Support/Debug.h"

using namespace llvm;
using namespace esyn;

namespace {
class ASAPScheduler : public Scheduler {
public:
  static char ID;
  explicit ASAPScheduler() : Scheduler(&ID) {}
  void scheduleBasicBlock(HWAState &State);
  void getAnalysisUsage(AnalysisUsage &AU) const;
  virtual void print(raw_ostream &O, const Module *M) const;
};
} //end namespace

char ASAPScheduler::ID = 0;

void ASAPScheduler::print(raw_ostream &O, const Module *M) const {

}

void ASAPScheduler::getAnalysisUsage(AnalysisUsage &AU) const {
  Scheduler::getAnalysisUsage(AU);
  AU.setPreservesAll();
}

void ASAPScheduler::scheduleBasicBlock(HWAState &State) {
  HWAOpInst *StateEnd = State.getStateEnd();
  assert(StateEnd && "Why StateEnd is null?");

  typedef SmallVector<HWAtom*, 64> HWAtomVec;
  HWAtomVec Atoms(State.begin(), State.end());
  Atoms.push_back(StateEnd);
  // TODO: sort the atoms

  // Remember the state start, so we can schedule this bb again
  State.scheduledTo(HI->getTotalCycle());
  //HI->incTotalCycle();
  // Schedule StateBegin

  // TODO: Check if the atoms are empty
  while (!StateEnd->isOperationFinish(HI->getTotalCycle())) {
    DEBUG(dbgs() << "======Cycle " << HI->getTotalCycle() << "\n");
    // Find all ready atoms

    // For each ready atoms
    while(HWAtom *ReadyAtom = getReadyAtoms(Atoms, HI->getTotalCycle())){
      if (HWAOpRes *OpRes = dyn_cast<HWAOpRes>(ReadyAtom)) {
        if (!OpRes->isResAllocated()) {// Not allocate?
          // Try to allocate a resource for it
          // Or just get the "idle instance?"
          assert(0 && "Resource allocate not support yet!");
        }
        HWResource::ResIdType ResId = OpRes->getResourceId();
        // Is the resource available?
        unsigned readyCyc = getReadyCycle(ResId);
        if (readyCyc > HI->getTotalCycle())
          continue;
        
        //
        rememberReadyCycle(ResId, HI->getTotalCycle() + OpRes->getLatency());
      }
      ReadyAtom->scheduledTo(HI->getTotalCycle());

      DEBUG(ReadyAtom->print(dbgs()));
      DEBUG(dbgs() << " scheduled\n");
      // Remove the exist atom
      HWAtomVec::iterator at = std::find(Atoms.begin(), Atoms.end(), ReadyAtom);
      Atoms.erase(at);
    }

    // Advance the state
    HI->incTotalCycle();
  }
  //// schedule the state end;

  DEBUG(State.print(dbgs()));
}


Pass *esyn::createListSchedulePass() {
  return new ASAPScheduler();
}