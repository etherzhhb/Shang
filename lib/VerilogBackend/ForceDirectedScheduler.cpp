//===- ForceDirectedScheduler.cpp - The ForceDirected Scheduler  -*- C++ -*-===//
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
// This file implement the Force Direct Scheduler pass describe in
// Force-Directed Scheduling for the Behavioral Synthesis of ASIC's
//
//===----------------------------------------------------------------------===//

#include "vbe/SchedulerBase.h"
#include "HWAtomInfo.h"
#include "HWAtomPasses.h"


#define DEBUG_TYPE "vbe-fd-schedule"
#include "llvm/Support/Debug.h"

using namespace llvm;
using namespace esyn;

namespace {

struct FDLScheduler : public BasicBlockPass, public Scheduler {

  HWAtomInfo *HI;
  ResourceConfig *RC;
  
  // Time Frame {asap step, alap step }
  typedef std::pair<unsigned, unsigned> TimeFrame;
  // Mapping hardware atoms to time frames.
  typedef std::map<HWAtom*, TimeFrame> TimeFrameMapType;

  TimeFrameMapType AtomToTF;
  void resetTimeFrame(HWAtom *Root);
  unsigned getASAPStep(HWAtom *A);
  void setASAPStep(HWAtom *A, unsigned step);

  unsigned getALAPStep(HWAtom *A);
  void setALAPStep(HWAtom *A, unsigned step);

  void printTimeFrame(HWAtom *A, raw_ostream &OS);

  void clear();

  /// @name Common pass interface
  //{
  static char ID;
  FDLScheduler() : BasicBlockPass(&ID), Scheduler() {}
  bool runOnBasicBlock(BasicBlock &BB);
  void releaseMemory();
  void getAnalysisUsage(AnalysisUsage &AU) const;
  virtual void print(raw_ostream &O, const Module *M) const;
  //}
};
} //end namespace

char FDLScheduler::ID = 0;

void FDLScheduler::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<HWAtomInfo>();
  AU.addRequired<ResourceConfig>();
  AU.setPreservesAll();
}

bool FDLScheduler::runOnBasicBlock(BasicBlock &BB) {
  DEBUG(dbgs() << "==================== " << BB.getName() << '\n');
  HI = &getAnalysis<HWAtomInfo>();
  ExecStage &Stage = HI->getStateFor(BB);
  HWAVRoot &Entry = Stage.getEntryRoot();
  // Build the time frame
  setASAPStep(&Entry, 1);
  DEBUG(printTimeFrame(&Entry, dbgs()));
  for (usetree_iterator I = usetree_iterator::begin(&Entry),
      E = usetree_iterator::end(&Entry); I != E; ++I) {
    HWAtom *A = *I;
    A->scheduledTo(HI->getTotalCycle() + AtomToTF[A].first - 1);
  }
  HI->setTotalCycle(Stage.getExitRoot().getSlot() + 1);
  // Build the Distribution Graphs
  return false;
}

void FDLScheduler::releaseMemory() {
  clear();
  clearSchedulerBase();
}

void FDLScheduler::resetTimeFrame(HWAtom *Root) {
  // Reset the time frame
  for (usetree_iterator I = usetree_iterator::begin(Root),
      E = usetree_iterator::end(Root); I != E; ++I)
    AtomToTF.erase(*I);
}

unsigned FDLScheduler::getASAPStep(HWAtom *A) {
  unsigned ret = 0;
  for (HWAtom::dep_iterator I = A->dep_begin(), E = A->dep_end();
      I != E; ++I) {
    HWAtom *Dep = *I;
    unsigned AsapStep = AtomToTF[Dep].first;
    if (AsapStep == 0) // The node not visit yet
      return 0;

    if (HWAOpInst *OI = dyn_cast<HWAOpInst>(Dep))
      AsapStep += OI->getLatency();
    // The node will be schedule as soon as the last dependence scheduled
    if(AsapStep > ret)
      ret = AsapStep;
  }

  return ret;
}


void FDLScheduler::setASAPStep(HWAtom *A, unsigned step) {
  DEBUG(A->dump());
  DEBUG(dbgs() << "set to asap " << step << '\n');
  AtomToTF[A].first = step;;
  
  for (HWAtom::use_iterator I = A->use_begin(), E = A->use_end();
      I != E; ++I) {
    HWAtom *U = *I;
    // U was already been schedule to a asap step.
    if (AtomToTF[U].first != 0)
      continue;
    
    unsigned dep_step = getASAPStep(U);
    // Some dependencies of U not finish
    if (dep_step == 0)
      continue;

    // All depends of U finish, schedule it to a step.
    setASAPStep(U, dep_step);
  }
}

unsigned FDLScheduler::getALAPStep(HWAtom *A) {
  unsigned ret = 0;
  for (HWAtom::use_iterator I = A->use_begin(), E = A->use_end();
    I != E; ++I) {
      HWAtom *Dep = *I;
      unsigned AsapStep = AtomToTF[Dep].second;
      if (AsapStep == 0) // The node not visit yet
        return 0;
      // The Node will be schedule only when the first use need it.
      else if(AsapStep < ret)
        ret = AsapStep;
  }
  return ret;
}

void FDLScheduler::clear() {
  AtomToTF.clear();
}

void FDLScheduler::printTimeFrame(HWAtom *A, raw_ostream &OS) {
  for (usetree_iterator I = usetree_iterator::begin(A),
    E = usetree_iterator::end(A); I != E; ++I) {
    (*I)->print(OS);
    OS << "\n{" << AtomToTF[*I].first << "," << AtomToTF[*I].second << "}\n";
  }
  
}

void FDLScheduler::print(raw_ostream &O, const Module *M) const { }


Pass *esyn::createFDLSchedulePass() {
  return new FDLScheduler();
}
