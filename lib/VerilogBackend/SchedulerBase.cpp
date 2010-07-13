//===------ ScheduleBase.cpp - The BaseClass of all scheduler  ----*- C++ -*-===//
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
// This file implement the SchedulerBase class, which provide common service for
// schedulers.
//
//===----------------------------------------------------------------------===//
//

#include "vbe/SchedulerBase.h"

#include "HWAtomInfo.h"
#include "llvm/Support/Debug.h"


using namespace llvm;
using namespace esyn;

//===----------------------------------------------------------------------===//
bool Scheduler::isOperationFinish(const HWAtom *Atom, unsigned CurSlot) {
  return Atom->getSlot() +Atom->getLatency() <= CurSlot;
}

bool Scheduler::isAllDepsOpFin(const HWAOpInst *Atom, unsigned CurSlot) {
  for (HWAOpInst::const_dep_iterator I = Atom->dep_begin(), E = Atom->dep_end();
      I != E; ++I)
    if (!isOperationFinish(*I, CurSlot))
      return false;

  return true;
}

bool Scheduler::isAllDepsScheduled(const HWAOpInst *Atom) {
  for (HWAOpInst::const_dep_iterator I = Atom->dep_begin(), E = Atom->dep_end();
    I != E; ++I)
    if (!(*I)->isScheduled())
      return false;

  return true;
}

void Scheduler::clearSchedulerBase() {
  ScheduleAtoms.clear();
  ResCycMap.clear();
}

Scheduler::~Scheduler() {
  clearSchedulerBase();
}

unsigned Scheduler::getReadyCycle(HWResource::ResIdType ResId) {
  return ResCycMap[ResId];
}

void Scheduler::rememberReadyCycle(HWResource::ResIdType ResId,
                                   unsigned ReadyCycle) {
  ResCycMap[ResId] = ReadyCycle;
}

Scheduler::ListIt Scheduler::getReadyAtoms(unsigned Cycle) {
  for (SchedAtomVec::iterator I = ScheduleAtoms.begin(),
      E = ScheduleAtoms.end(); I != E; ++I) {
    HWAOpInst *atom = *I;
    if (isAllDepsOpFin(atom, Cycle)) {
      DEBUG(atom->print(dbgs()));
      DEBUG(dbgs() << " is Ready\n");
      return I;
    }
  }
  return ScheduleAtoms.end();
}

void Scheduler::removeFromList(ListIt At) {
  ScheduleAtoms.erase(At);
}

void Scheduler::createAtomList() {
  for (usetree_iterator I = CurStage->usetree_begin(), E = CurStage->usetree_end();
      I != E; ++I)
    if (HWAOpInst *A = dyn_cast<HWAOpInst>(*I))    
      ScheduleAtoms.push_back(A);
}