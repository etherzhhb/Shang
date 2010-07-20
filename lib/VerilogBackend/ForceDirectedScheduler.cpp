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
  unsigned StartStep, EndStep;

  /// @name PriorityQueue
  //{
  typedef PriorityQueue<HWAOpInst*, std::vector<HWAOpInst*>, fds_sort> AtomQueueType;
  
  template<class It>
  void fillQueue(AtomQueueType &Queue, It begin, It end);

  bool scheduleQueue(AtomQueueType &Queue);
  //}

  unsigned findBestStep(HWAOpInst *A);

  void clear();

  void FDListSchedule();
  void FDModuloSchedule();

  /// @name Common pass interface
  //{
  static char ID;
  FDLScheduler() : BasicBlockPass(&ID) {}
  bool runOnBasicBlock(BasicBlock &BB);
  void releaseMemory();
  void getAnalysisUsage(AnalysisUsage &AU) const;
  virtual void print(raw_ostream &O, const Module *M) const;
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

  StartStep = HI->getTotalCycle();
  Entry->scheduledTo(StartStep);

  if (MSInfo->isModuloSchedulable(*CurState))
    FDModuloSchedule();
  else
    FDListSchedule();
  
  HI->setTotalCycle(CurState->getExitRoot().getSlot() + 1);

  return false;
}

void FDLScheduler::FDModuloSchedule() {
  // Compute MII.
  unsigned RecMII = MSInfo->computeRecMII(*CurState);
  unsigned ResMII = MSInfo->computeResMII(*CurState);
  unsigned MII = std::max(RecMII, ResMII);

  // Set up Resource table
  FDInfo->clear();

  FDInfo->enableModuleFD(MII);
  EndStep = FDInfo->buildFDInfo(CurState, StartStep);

  //
  fds_sort s(FDInfo);
  AtomQueueType AQueue(s);

  typedef ModuloScheduleInfo::rec_iterator rec_iterator;
  typedef ModuloScheduleInfo::scc_vector scc_vector;
  // Schedule all SCCs.
  for (unsigned i = MII; i > 0; --i) {
    for (rec_iterator I = MSInfo->rec_begin(i), E = MSInfo->rec_end(i);
        I != E; ++I) {
      scc_vector &SCC = I->second;
      AQueue.clear();
      fillQueue(AQueue, SCC.begin(), SCC.end());
      scheduleQueue(AQueue);
    }
  }

  DEBUG(FDInfo->dumpTimeFrame(CurState));
  DEBUG(FDInfo->dumpDG(CurState));
}

void FDLScheduler::FDListSchedule() {
  FDInfo->clear();
  EndStep = FDInfo->buildFDInfo(CurState, StartStep);

  fds_sort s(FDInfo);
  AtomQueueType AQueue(s);

  fillQueue(AQueue, CurState->usetree_begin(), CurState->usetree_end());
  scheduleQueue(AQueue);
}


unsigned FDLScheduler::findBestStep(HWAOpInst *A) {
  std::pair<unsigned, double> BestStep = std::make_pair(0, 1e32);
  // For each possible step:
  for (unsigned i = FDInfo->getASAPStep(A), e = FDInfo->getALAPStep(A) + 1;
      i != e; ++i) {
    // Compute the forces.
    DEBUG(dbgs() << "At Step " << i);
    double SelfForce = FDInfo->computeSelfForceAt(A, i);
    // Force update time frame
    A->scheduledTo(i);
    // Recover the time frame by force rebuild
    FDInfo->buildASAPStep(Entry, StartStep); 
    FDInfo->buildALAPStep(Exit, EndStep);

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
}

void FDLScheduler::print(raw_ostream &O, const Module *M) const { }

bool FDLScheduler::scheduleQueue(AtomQueueType &Queue) {
  while (!Queue.empty()) {
    // TODO: Short the list
    HWAOpInst *A = Queue.top();
    Queue.pop();

    DEBUG(A->print(dbgs()));
    DEBUG(dbgs() << " Active to schedule:-------------------\n");

    // TODO: Do check if this step is valid for others constrain.
    unsigned step = FDInfo->getASAPStep(A);
    if (FDInfo->getTimeFrame(A) != 1) {
      step = findBestStep(A);
      A->scheduledTo(step);

      DEBUG(dbgs() << " After schedule:-------------------\n");
      FDInfo->buildFDInfo(CurState, StartStep);
      DEBUG(dbgs() << "\n\n\n");
      Queue.reheapify();
    } else {
      // Schedule to the best step.
      A->scheduledTo(step);
    }
  }
  return true;
}

Pass *esyn::createFDLSchedulePass() {
  return new FDLScheduler();
}
