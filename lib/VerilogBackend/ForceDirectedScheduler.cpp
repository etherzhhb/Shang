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

#include "ForceDirectedInfo.h"
#include "ModuloScheduleInfo.h"
#include "HWAtomPasses.h"

#include "llvm/ADT/PriorityQueue.h"

#define DEBUG_TYPE "vbe-fd-sched"
#include "llvm/Support/Debug.h"

using namespace llvm;
using namespace esyn;

namespace {
struct fds_sort {
  ForceDirectedInfo *Info;
  fds_sort(ForceDirectedInfo *s) : Info(s) {}
  bool operator() (const HWAOpInst* LHS, const HWAOpInst* RHS) const;
};

struct FDLScheduler : public BasicBlockPass {
  HWAtomInfo *HI;
  ResourceConfig *RC;
  ForceDirectedInfo *FDInfo;
  ModuloScheduleInfo *MSInfo;
  FSMState *CurState;

  HWAVRoot *Entry; 
  HWAOpInst *Exit;

  enum SchedResult {
    SchedSucc, SchedFailCricitalPath, SchedFailII
  };

  /// @name PriorityQueue
  //{
  typedef PriorityQueue<HWAOpInst*, std::vector<HWAOpInst*>, fds_sort> AtomQueueType;
  
  template<class It>
  void fillQueue(AtomQueueType &Queue, It begin, It end);

  // Return the first node that we fail to schedule, return null if all nodes are scheduled.
  SchedResult scheduleQueue(AtomQueueType &Queue);
  SchedResult scheduleAtII(unsigned II);
  //}

  unsigned findBestStep(HWAOpInst *A);

  void clear();

  void FDListSchedule(unsigned StartStep);
  void FDModuloSchedule(unsigned StartStep);

  /// @name Common pass interface
  //{
  static char ID;
  FDLScheduler() : BasicBlockPass(&ID) {}
  bool runOnBasicBlock(BasicBlock &BB);
  void releaseMemory();
  void getAnalysisUsage(AnalysisUsage &AU) const;
  void print(raw_ostream &O, const Module *M) const;
  //}
};
} //end namespace

//===----------------------------------------------------------------------===//
bool fds_sort::operator()(const HWAOpInst* LHS, const HWAOpInst* RHS) const {
  // Schedule the low mobility nodes first.
  if (Info->getTimeFrame(LHS) > Info->getTimeFrame(RHS))
    return true; // Place RHS first.
  
  //unsigned LHSLatency = FDS->getASAPStep(LHS);
  //unsigned RHSLatency = FDS->getASAPStep(RHS);
  //// Schedule as soon as possible?
  //if (LHSLatency < RHSLatency) return true;
  //if (LHSLatency > RHSLatency) return false;

  return false;
}

//===----------------------------------------------------------------------===//
char FDLScheduler::ID = 0;

void FDLScheduler::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<HWAtomInfo>();
  AU.addRequired<ResourceConfig>();
  AU.addRequired<ForceDirectedInfo>();
  //
  AU.addRequired<ModuloScheduleInfo>();
  AU.setPreservesAll();
}

template<class It>
void FDLScheduler::fillQueue(AtomQueueType &Queue, It begin, It end) {
  for (It I = begin, E = end; I != E; ++I)
    if (HWAOpInst *A = dyn_cast<HWAOpInst>(*I))    
      Queue.push(A);
}

bool FDLScheduler::runOnBasicBlock(BasicBlock &BB) {
  DEBUG(dbgs() << "==================== " << BB.getName() << '\n');
  HI = &getAnalysis<HWAtomInfo>();
  FDInfo = &getAnalysis<ForceDirectedInfo>();
  MSInfo = &getAnalysis<ModuloScheduleInfo>();
  CurState = &(HI->getStateFor(BB));

  // Build the time frame
  Entry = &CurState->getEntryRoot(); 
  Exit = &CurState->getExitRoot();

  unsigned StartStep = HI->getTotalCycle();
  Entry->scheduledTo(StartStep);

  if (MSInfo->isModuloSchedulable(*CurState))
    FDModuloSchedule(StartStep);
  else
    FDListSchedule(StartStep);
  
  HI->setTotalCycle(CurState->getExitRoot().getSlot() + 1);

  // Do not forget to schedule the delay atomp;
  for (usetree_iterator I = CurState->usetree_begin(),
      E = CurState->usetree_end(); I != E; ++I) {
    HWAtom *A = *I;
    if (!A->isScheduled()) {
      assert(isa<HWADelay>(A) && "Some atom not scheduled!");
      A->scheduledTo(FDInfo->getASAPStep(A));
    }
  }
  
  return false;
}

void FDLScheduler::FDModuloSchedule(unsigned StartStep) {
  // Compute MII.
  unsigned RecMII = MSInfo->computeRecMII(*CurState);
  unsigned ResMII = MSInfo->computeResMII(*CurState);
  unsigned MII = std::max(RecMII, ResMII);
  HWAOpInst *FailNode = 0;

  unsigned EndStep = 0;

  for(;;) {
    // Set up Resource table
    FDInfo->clear();
    FDInfo->enableModuleFD(MII);
    EndStep = FDInfo->buildFDInfo(CurState, StartStep, EndStep);

    switch (scheduleAtII(MII)) {
    case FDLScheduler::SchedSucc:
      DEBUG(FDInfo->dumpTimeFrame(CurState));
      DEBUG(FDInfo->dumpDG(CurState));
      // Set up the initial interval.
      CurState->setII(MII);
      return;
    case FDLScheduler::SchedFailCricitalPath:
      ++EndStep;
      continue;
    case FDLScheduler::SchedFailII:
      ++MII;
      continue;
    }
  }
}

FDLScheduler::SchedResult FDLScheduler::scheduleAtII(unsigned II) {
  fds_sort s(FDInfo);
  AtomQueueType AQueue(s);

  typedef ModuloScheduleInfo::rec_iterator rec_iterator;
  typedef ModuloScheduleInfo::scc_vector scc_vector;
  // Schedule all SCCs.
  for (unsigned i = II; i > 0; --i) {
    for (rec_iterator I = MSInfo->rec_begin(i), E = MSInfo->rec_end(i);
        I != E; ++I) {
      scc_vector &SCC = I->second;
      AQueue.clear();
      // TODO:
      // First of all Schedule the node that do not have any predecessor in
      // the SCC (Schedule First Node).
      // First Node = ...
      // And schedule rest nodes in SCC, but the max latency of the SCC
      // should not exceed II.
      fillQueue(AQueue, SCC.begin(), SCC.end());
      //
      SchedResult Result = scheduleQueue(AQueue);
      if (Result != FDLScheduler::SchedSucc)
        return Result;
      //assert(((SCC.front()->getSlot() - SCC.back()->getSlot() < II)
      //         || (SCC.front()->getSlot() - SCC.back()->getSlot() < II) ) 
      //       && "Dependencies break!");
    }
  }

  return FDLScheduler::SchedSucc;
}

void FDLScheduler::FDListSchedule(unsigned StartStep) {
  HWAOpInst *FailNode = 0;

  unsigned EndStep = 0;

  for(;;) {
    FDInfo->clear();
    EndStep = FDInfo->buildFDInfo(CurState, StartStep, EndStep);

    fds_sort s(FDInfo);
    AtomQueueType AQueue(s);

    fillQueue(AQueue, CurState->usetree_begin(), CurState->usetree_end());

    if (scheduleQueue(AQueue) != FDLScheduler::SchedSucc)
      ++EndStep;
    else // Break the loop if we schedule successful.
      break;
  }
  // Set the Initial Interval to the total slot, so we can generate the correct
  // control logic for loop if MS is disable.
  if (CurState->haveSelfLoop())
    CurState->setII(CurState->getTotalSlot());
  DEBUG(FDInfo->dumpTimeFrame(CurState));
}


unsigned FDLScheduler::findBestStep(HWAOpInst *A) {
  std::pair<unsigned, double> BestStep = std::make_pair(0, 1e32);

  // For each possible step:
  for (unsigned i = FDInfo->getASAPStep(A), e = FDInfo->getALAPStep(A) + 1;
      i != e; ++i) {
    DEBUG(dbgs() << "At Step " << i << "\n");
    // Check if we can schedule the node at this step because the resource
    // is not enough.
    if (!FDInfo->isFUAvailalbe(i, A->getFunUnit()))
      continue;

    // Compute the forces.
    double SelfForce = FDInfo->computeSelfForceAt(A, i);
    // Force update time frame
    A->scheduledTo(i);
    // Recover the time frame by force rebuild
    FDInfo->buildASAPStep(CurState); 
    FDInfo->buildALAPStep(CurState);

    // The follow function will invalid the time frame.
    DEBUG(dbgs() << " Self Force: " << SelfForce);
    double PredForce = FDInfo->computePredForceAt(A, i);
    DEBUG(dbgs() << " Pred Force: " << PredForce);
    double SuccForce = FDInfo->computeSuccForceAt(A, i);
    DEBUG(dbgs() << " Succ Force: " << SuccForce);
    double Force = SelfForce + PredForce + SuccForce;
    if (Force < BestStep.second)
      BestStep = std::make_pair(i, Force);

    DEBUG(dbgs() << '\n');
  }
  return BestStep.first;
}

void FDLScheduler::releaseMemory() {
  clear();
}

void FDLScheduler::clear() {
  CurState = 0;
  FDInfo->clear();
  MSInfo->clear();
}

void FDLScheduler::print(raw_ostream &O, const Module *M) const { }


FDLScheduler::SchedResult FDLScheduler::scheduleQueue(AtomQueueType &Queue) {
  while (!Queue.empty()) {
    // TODO: Short the list
    HWAOpInst *A = Queue.top();
    Queue.pop();

    bool isModuloConstrains = FDInfo->isModuloConstrains(A);

    DEBUG(A->print(dbgs()));
    DEBUG(dbgs() << " Active to schedule:-------------------\n");

    // TODO: Do check if this step is valid for others constrain.
    unsigned step = FDInfo->getASAPStep(A);
    if (FDInfo->getTimeFrame(A) != 1) {
      step = findBestStep(A);
      // If we can not schedule A.
      if (step == 0)
        return isModuloConstrains ? FDLScheduler::SchedFailII :
                                    FDLScheduler::SchedFailCricitalPath;

      A->scheduledTo(step);

      DEBUG(dbgs() << " After schedule:-------------------\n");
      FDInfo->recoverFDInfo(CurState);
      DEBUG(dbgs() << "\n\n\n");
      Queue.reheapify();
    } else {
      // Schedule to the best step.
      A->scheduledTo(step);
    }
  }
  return FDLScheduler::SchedSucc;
}

Pass *esyn::createFDLSchedulePass() {
  return new FDLScheduler();
}
