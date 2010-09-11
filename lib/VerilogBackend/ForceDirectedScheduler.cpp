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

static cl::opt<bool>
NoFDLS("disable-fdls",
       cl::desc("vbe - Do not preform force-directed list schedule"),
       cl::Hidden, cl::init(false));


static cl::opt<bool>
NoFDMS("disable-fdms",
       cl::desc("vbe - Do not preform force-directed modulo schedule"),
       cl::Hidden, cl::init(false));

namespace {
struct FDSPass : public BasicBlockPass {
  FSMState *State;
  ForceDirectedSchedulingBase *Scheduler;
  /// @name Common pass interface
  //{
  static char ID;
  FDSPass() : BasicBlockPass(&ID), State(0) {}
  bool runOnBasicBlock(BasicBlock &BB);
  void releaseMemory();
  void scheduleACyclicCodeRegion();
  void scheduleCyclicCodeRegion(unsigned II);
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

  State = HI->getStateFor(BB);

  // Create the FDInfo.
  ModuloScheduleInfo MSInfo(HI, &getAnalysis<LoopInfo>(), State);
  
  if (NoFDLS)
    Scheduler = new ForceDirectedScheduler(HI, State);
  else
    Scheduler = new ForceDirectedListScheduler(HI, State);

  if (MSInfo.isModuloSchedulable()) {
    if (NoFDMS) {
      delete Scheduler;
      Scheduler = new IMS(HI, State);
    }
    
    scheduleCyclicCodeRegion(MSInfo.computeMII());
  } else
    scheduleACyclicCodeRegion();

  HI->setTotalCycle(State->getEndSlot() + 1);

  // Do not forget to schedule the delay atom;
  for (FSMState::iterator I = State->begin(), E = State->end();
       I != E; ++I) {
    HWAtom *A = *I;
    assert(A->isScheduled() && "Schedule incomplete!");
  }

  return false;
}

void FDSPass::print(raw_ostream &O, const Module *M) const { }

void FDSPass::scheduleACyclicCodeRegion() {
  while (!Scheduler->scheduleState())
    Scheduler->lengthenCriticalPath();

  // Set the Initial Interval to the total slot, so we can generate the correct
  // control logic for loop if MS is disable.
  if (State->haveSelfLoop())
    State->setNoOverlapII();
  DEBUG(Scheduler->dumpTimeFrame());
}

void FDSPass::scheduleCyclicCodeRegion(unsigned II) {
  // Ensure us can schedule the critical path.
  while (!Scheduler->scheduleCriticalPath(true))
    Scheduler->lengthenCriticalPath();

  Scheduler->setMII(II);
  while (!Scheduler->scheduleCriticalPath(true))
    Scheduler->increaseMII();

  // The point of current solution.
  typedef std::pair<unsigned, unsigned> SolutionPoint;
  SolutionPoint CurPoint
    = std::make_pair(Scheduler->getMII(), Scheduler->getCriticalPathLength());
  SmallVector<SolutionPoint, 3> NextPoints;

  double lastReq = 1e9;

  while (!Scheduler->scheduleState()) {
    double CurReq = Scheduler->getExtraResReq();
    if (lastReq > CurReq) {
      CurPoint = std::make_pair(Scheduler->getMII(),
                                Scheduler->getCriticalPathLength());
      lastReq = CurReq;
      NextPoints.clear();
    }

    if (NextPoints.empty()) {
      NextPoints.push_back(std::make_pair(CurPoint.first + 1, CurPoint.second  + 1));
      if (Scheduler->getCriticalPathLength() >= Scheduler->getMII())
        NextPoints.push_back(std::make_pair(CurPoint.first + 1, CurPoint.second));
      NextPoints.push_back(std::make_pair(CurPoint.first, CurPoint.second  + 1));
      // Add both by default.
      CurPoint = std::make_pair(CurPoint.first + 1, CurPoint.second  + 1);
    }

    Scheduler->setMII(NextPoints.back().first);
    Scheduler->setCriticalPathLength(NextPoints.back().second);
    NextPoints.pop_back();
  }

  //Scheduler->getCriticalPathLength() < Scheduler->getMII())
  State->setII(Scheduler->getMII());
}

void FDSPass::releaseMemory() {
  State = 0;
  delete Scheduler;
  Scheduler = 0;
}

Pass *esyn::createFDLSPass() {
  return new FDSPass();
}
