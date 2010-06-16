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

#include "llvm/Support/Debug.h"


using namespace llvm;
using namespace esyn;

void ScheduleDriver::getAnalysisUsage(AnalysisUsage &AU) const {
}


void ScheduleDriver::releaseMemory() {

}

bool ScheduleDriver::runOnFunction(Function &F) {
  return false;
}

void ScheduleDriver::print(raw_ostream &O, const Module *M) const {

}

void ScheduleDriver::clear() {
}

char ScheduleDriver::ID = 0;

void Scheduler::clear() {
  ResCycMap.clear();
}

Scheduler::~Scheduler() {
  clear();
}

unsigned Scheduler::getReadyCycle(const HWResource *Resource,
                                  unsigned Instance) {
  return ResCycMap[Resource][Instance];
}

void Scheduler::rememberReadyCycle(const HWResource *Resource,
                                   unsigned Instance,
                                   unsigned ReadyCycle){
  ResCycMap[Resource][Instance] = ReadyCycle;
}

HWAtom* Scheduler::getReadyAtoms(SmallVectorImpl<HWAtom*> &ToSchedAtoms,
                                 unsigned Cycle) const {
  typedef SmallVectorImpl<HWAtom*> AtomVec;
  for (AtomVec::iterator I = ToSchedAtoms.begin(), E = ToSchedAtoms.end();
      I != E; ++I) {
    HWAtom *atom = *I;
    if (atom->getSlot() > Cycle
        && atom->isAllDepsOpFin(Cycle)) {
      
      atom->print(dbgs());
      dbgs() << " is Ready\n";

      return atom;
    }
  }
  return 0;
}
