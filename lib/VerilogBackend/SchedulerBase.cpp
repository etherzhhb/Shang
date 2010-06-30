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
  if (const HWAOpInst *Op = dyn_cast<HWAOpInst>(Atom))
    return Op->getSlot() + Op->getLatency() <= CurSlot;

  if (isa<HWAInline>(Atom))
    return Atom->getSlot() <= CurSlot;

  // Constant is always finish
  // Entry root is always finish
  return true;
}

bool Scheduler::isAllDepsOpFin(const HWAtom *Atom, unsigned CurSlot) {
  for (HWAtom::const_dep_iterator I = Atom->dep_begin(), E = Atom->dep_end();
      I != E; ++I)
    if (!isOperationFinish(*I, CurSlot))
      return false;

  return true;
}

void Scheduler::clear() {
  ScheduleAtoms.clear();
  ResCycMap.clear();
}

Scheduler::~Scheduler() {
  clear();
}

unsigned Scheduler::getReadyCycle(HWResource::ResIdType ResId) {
  return ResCycMap[ResId];
}

void Scheduler::rememberReadyCycle(HWResource::ResIdType ResId,
                                   unsigned ReadyCycle) {
  ResCycMap[ResId] = ReadyCycle;
}

HWAtom *Scheduler::getReadyAtoms(unsigned Cycle) {
  for (SchedAtomVec::iterator I = ScheduleAtoms.begin(),
      E = ScheduleAtoms.end(); I != E; ++I) {
    HWAtom *atom = *I;
    if (isAllDepsOpFin(atom, Cycle)) {
      DEBUG(atom->print(dbgs()));
      DEBUG(dbgs() << " is Ready\n");
      return atom;
    }
  }
  return 0;
}

void Scheduler::removeFromList(HWAtom *Atom) {
  SchedAtomVec::iterator at = std::find(ScheduleAtoms.begin(),
                                        ScheduleAtoms.end(), Atom);
  ScheduleAtoms.erase(at);
}

void esyn::Scheduler::createAtomList(HWAtomInfo *HI, BasicBlock &BB) {
  ExecStage &Stage = HI->getStateFor(BB);
  for (ExecStage::entry_iterator I = Stage.entry_begin(), E = Stage.entry_end();
      I != E; ++I)
    ScheduleAtoms.push_back(*I);
}