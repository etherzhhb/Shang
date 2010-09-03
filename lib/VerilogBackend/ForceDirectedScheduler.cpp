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

#include "ForceDirectedScheduling.h"
#include "ModuloScheduleInfo.h"
#include "HWAtomPasses.h"

#include "llvm/Analysis/LoopInfo.h"
#include "llvm/ADT/OwningPtr.h"

#define DEBUG_TYPE "vbe-fd-sched"
#include "llvm/Support/Debug.h"

using namespace llvm;
using namespace esyn;

namespace {
struct FDSPass : public BasicBlockPass {
  /// @name Common pass interface
  //{
  static char ID;
  FDSPass() : BasicBlockPass(&ID) {}
  bool runOnBasicBlock(BasicBlock &BB);
  void getAnalysisUsage(AnalysisUsage &AU) const;
  void print(raw_ostream &O, const Module *M) const;
  //}
};

} //end namespace

//===----------------------------------------------------------------------===//
char FDSPass::ID = 0;

void FDSPass::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<HWAtomInfo>();
  AU.addRequired<LoopInfo>();
  AU.setPreservesAll();
}

bool FDSPass::runOnBasicBlock(BasicBlock &BB) {
  DEBUG(dbgs() << "==================== " << BB.getName() << '\n');
  HWAtomInfo *HI = &getAnalysis<HWAtomInfo>();

  FSMState *CurState = HI->getStateFor(BB);

  // Create the FDInfo.
  ModuloScheduleInfo MSInfo(HI, &getAnalysis<LoopInfo>(), CurState);
  
  OwningPtr<ForceDirectedSchedulingBase> fds(0);

  if (MSInfo.isModuloSchedulable()) {
    fds.reset(new ForceDirectedModuloScheduler(HI, CurState, MSInfo.computeMII()));
  } else
    fds.reset(new ForceDirectedListScheduler(HI, CurState, MSInfo.computeMII()));

  fds->scheduleState();

  HI->setTotalCycle(CurState->getEndSlot() + 1);

  // Do not forget to schedule the delay atom;
  for (FSMState::iterator I = CurState->begin(), E = CurState->end();
       I != E; ++I) {
    HWAtom *A = *I;
    assert(A->isScheduled() && "Schedule incomplete!");
  }

  return false;
}

void FDSPass::print(raw_ostream &O, const Module *M) const { }

Pass *esyn::createFDLSPass() {
  return new FDSPass();
}
