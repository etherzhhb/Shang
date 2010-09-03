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
  bool operator() (const HWAOpFU *LHS, const HWAOpFU *RHS) const;
};

struct FDLScheduler : public BasicBlockPass {
  HWAtomInfo *HI;
  ResourceConfig *RC;
  ForceDirectedInfo *FDInfo;
  ModuloScheduleInfo *MSInfo;
  FSMState *CurState;
  HWAOpFU *Exit;

  unsigned MII;

  /// @name PriorityQueue
  //{
  typedef PriorityQueue<HWAOpFU*, std::vector<HWAOpFU*>, fds_sort> AtomQueueType;
  
  // Find the Node the do not have dependency in this Queue.
  template<class It>
  HWAtom *findFirstNode(It begin, It end);

  // Fill the priorityQueue, ignore FirstNode.
  template<class It>
  void fillQueue(AtomQueueType &Queue, It begin, It end, HWAtom *FirstNode = 0);

  typedef ModuloScheduleInfo::rec_iterator rec_iterator;
  typedef ModuloScheduleInfo::rec_vector rec_vector;

  // Return true when resource constraints preserved after citical path
  // scheduled
  bool scheduleCriticalPath();

  bool scheduleAtom(HWAtom *A);
  bool scheduleQueue(AtomQueueType &Queue);
  bool scheduleAtII();
  //}

  unsigned findBestStep(HWAtom *A);

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
  void print(raw_ostream &O, const Module *M) const;
  //}
};
} //end namespace

//===----------------------------------------------------------------------===//
bool fds_sort::operator()(const HWAOpFU* LHS, const HWAOpFU* RHS) const {
  HWFUnit *LFU = LHS->getFUnit(), *RFU = RHS->getFUnit();
  // Schedule the atom with less available function unit first.
  if (LFU->getTotalFUs() > RFU->getTotalFUs())
    return true;
  else if (LFU->getTotalFUs() < RFU->getTotalFUs())
    return false;

  // Schedule the low mobility nodes first.
  if (Info->getTimeFrame(LHS) > Info->getTimeFrame(RHS))
    return true; // Place RHS first.
  else if (Info->getTimeFrame(LHS) < Info->getTimeFrame(RHS))
    return false;

  //unsigned LHSLatency = FDS->getASAPStep(LHS);
  //unsigned RHSLatency = FDS->getASAPStep(RHS);
  //// Schedule as soon as possible?
  //if (LHSLatency < RHSLatency) return true;
  //if (LHSLatency > RHSLatency) return false;

  return LHS->getIdx() > RHS->getIdx();
}

//===----------------------------------------------------------------------===//
char FDLScheduler::ID = 0;

void FDLScheduler::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<HWAtomInfo>();
  AU.addRequired<ResourceConfig>();
  AU.addRequired<ForceDirectedInfo>();
  AU.addRequired<ModuloScheduleInfo>();
  AU.setPreservesAll();
}

template<class It>
void FDLScheduler::fillQueue(AtomQueueType &Queue, It begin, It end,
                             HWAtom *FirstNode) {
  for (It I = begin, E = end; I != E; ++I) {
    HWAtom *A = *I;

    // Do not push the FirstNode into the queue.
    if (A == FirstNode || A->isScheduled())
      continue;
    
    if (HWAOpFU *OI = dyn_cast<HWAOpFU>(A))    
      Queue.push(OI);
  }
  //
  Queue.reheapify();
}

template<class It>
HWAtom *FDLScheduler::findFirstNode(It begin, It end) {
  if (begin == end) return 0;

  std::pair<HWAtom*, unsigned> Ret =
    std::make_pair(*begin, FDInfo->getASAPStep(*begin));

  while (++begin != end) {
    HWAtom *A = *begin;

    unsigned ASAP = FDInfo->getASAPStep(A);
    if (ASAP < Ret.second)
      Ret = std::make_pair(A, ASAP);
  }

  return Ret.first;
}

bool FDLScheduler::runOnBasicBlock(BasicBlock &BB) {
  DEBUG(dbgs() << "==================== " << BB.getName() << '\n');
  HI = &getAnalysis<HWAtomInfo>();
  FDInfo = &getAnalysis<ForceDirectedInfo>();
  MSInfo = &getAnalysis<ModuloScheduleInfo>();

  CurState = HI->getStateFor(BB);

  if (MSInfo->isModuloSchedulable(*CurState)) {
    unsigned RecMII = MSInfo->computeRecMII(*CurState);
    unsigned ResMII = MSInfo->computeResMII(*CurState);
    MII = std::max(RecMII, ResMII);
    FDModuloSchedule();
  } else
    FDListSchedule();

  HI->setTotalCycle(CurState->getEndSlot() + 1);

  // Do not forget to schedule the delay atom;
  for (FSMState::iterator I = CurState->begin(), E = CurState->end();
       I != E; ++I) {
    HWAtom *A = *I;
    if (!A->isScheduled()) {
      A->scheduledTo(FDInfo->getASAPStep(A));
    }
  }
  
  return false;
}

void FDLScheduler::FDModuloSchedule() {
  unsigned StartStep = HI->getTotalCycle();
  CurState->scheduledTo(StartStep);

  // Ensure us can schedule the critical path.
  for (;;) {
    FDInfo->buildFDInfo();
    if (scheduleCriticalPath())
      break;
    DEBUG(FDInfo->dumpTimeFrame());
    // TODO: check if we could ever schedule these node without breaking the
    // resource constrain by check the DG.
    // If the resource average DG is bigger than the total available resource
    // we can never schedule the nodes without breaking the resource constrain.
    CurState->resetSchedule(StartStep);
    FDInfo->lengthenCriticalPath();
  }

  // Dirty Hack: Search the solution by increasing MII and critical path
  // alternatively.

  FDInfo->setMII(MII);
  for (;;) {
    FDInfo->buildFDInfo();
    if (scheduleCriticalPath())
      break;
    DEBUG(FDInfo->dumpTimeFrame());
    FDInfo->lengthenMII();
    CurState->resetSchedule(StartStep);
  }

  bool lastIncMII = true;
  for (;;) {
    bool SchedSucc = scheduleCriticalPath();
    assert(SchedSucc
      && "Why nodes critical path can not be schedule since DG is ok?");

    if (scheduleAtII()) {
      DEBUG(FDInfo->dumpTimeFrame());
      DEBUG(FDInfo->dumpDG());
      // Set up the initial interval.
      CurState->setII(FDInfo->getMII());
      return;
    } else if (lastIncMII) {
      FDInfo->lengthenCriticalPath();
      lastIncMII = false;
    } else {
      FDInfo->lengthenMII();
      MII = FDInfo->getMII();
      lastIncMII = true;
    }
    // Prepare for next schedule.
    CurState->resetSchedule(StartStep);
    FDInfo->buildFDInfo();
  }
}

bool FDLScheduler::scheduleAtII() {
  fds_sort s(FDInfo);
  AtomQueueType AQueue(s);

  for (unsigned i = FDInfo->getMII(); i > 0; --i) {
    for (rec_iterator I = MSInfo->rec_begin(i), E = MSInfo->rec_end(i);
        I != E; ++I) {
      rec_vector &Rec = I->second;
      HWAtom *LastAtom = Rec.back();
      DEBUG(dbgs() << "Recurrence at " << i << "--------------------\n");
      for (rec_vector::iterator SI = Rec.begin(), SE = Rec.end();
          SI != SE; ++SI) {
        HWAtom *A = *SI;
        HWEdge *Edge = LastAtom->getEdgeFrom(A);
        LastAtom = A;
        DEBUG(
          A->print(dbgs());
          dbgs() << " {" << FDInfo->getASAPStep(A) << ", "
                 << FDInfo->getALAPStep(A) << "}";
          if (Edge->isBackEdge())
            dbgs() << " Back-edge";
          if (A->isScheduled())
            dbgs() << " [Scheduled]\n";
        );

        // Only schedule backedge source at this phase.
        if (!Edge->isBackEdge())
          continue;

        if (A->isScheduled())
          continue;

        if (!scheduleAtom(A))
          return false;
        else // Only schedule one backedge.
          break;
      }
      DEBUG(FDInfo->dumpTimeFrame());
    }
  }

  // Schedule other nodes.
  AQueue.clear();
  fillQueue(AQueue, CurState->begin(), CurState->end());

  return scheduleQueue(AQueue);
}

void FDLScheduler::FDListSchedule() {
  unsigned StartStep = HI->getTotalCycle();
  for(;;) {
    CurState->resetSchedule(StartStep);
    FDInfo->buildFDInfo();

    fds_sort s(FDInfo);
    AtomQueueType AQueue(s);

    fillQueue(AQueue, CurState->begin(), CurState->end());

    if (!scheduleQueue(AQueue))
      FDInfo->lengthenCriticalPath();
    else // Break the loop if we schedule successful.
      break;
  }
  // Set the Initial Interval to the total slot, so we can generate the correct
  // control logic for loop if MS is disable.
  if (CurState->haveSelfLoop())
    CurState->setII(CurState->getTotalSlot());
  DEBUG(FDInfo->dumpTimeFrame());
}


unsigned FDLScheduler::findBestStep(HWAtom *A) {
  std::pair<unsigned, double> BestStep = std::make_pair(0, 1e32);
  DEBUG(dbgs() << "\tScan for best step:\n");
  // For each possible step:
  for (unsigned i = FDInfo->getASAPStep(A), e = FDInfo->getALAPStep(A) + 1;
      i != e; ++i) {
    DEBUG(dbgs() << "At Step " << i << "\n");

    // Temporary schedule A to i so we can get a more accurate pred and succ
    // force. Because the back edge constraint from A will be considered.
    A->scheduledTo(i);
    FDInfo->buildTimeFrame();
    // Compute the forces.
    double SelfForce = FDInfo->computeSelfForceAt(A, i);
    // The follow function will invalid the time frame.
    DEBUG(dbgs() << " Self Force: " << SelfForce);
    double PredForce = FDInfo->computePredForceAt(A, i);
    DEBUG(dbgs() << " Pred Force: " << PredForce);
    double SuccForce = FDInfo->computeSuccForceAt(A, i);
    DEBUG(dbgs() << " Succ Force: " << SuccForce);
    double Force = SelfForce + PredForce + SuccForce;
    DEBUG(dbgs() << " Force: " << Force);
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
  MSInfo->clear();
}

void FDLScheduler::print(raw_ostream &O, const Module *M) const { }

bool FDLScheduler::scheduleCriticalPath() {
  bool AnyScheduled = false;
  for (FSMState::iterator I = CurState->begin(), E = CurState->end();
      I != E; ++I) {
    HWAtom *A = *I;
  
    if (A->isScheduled() || FDInfo->getTimeFrame(A) != 1)
      continue;

    unsigned step = FDInfo->getASAPStep(A);
    DEBUG(A->print(dbgs()));
    DEBUG(dbgs() << " asap step: " << step << "\n");
    A->scheduledTo(step);
    AnyScheduled = true;
  }

  // Time frame may changed because of backedge constraints of scheduled nodes.
  if (AnyScheduled) {
    FDInfo->buildFDInfo();
    return FDInfo->isResourceConstraintPreserved();
  }
  // Else the time frame not changed.
  return true;
}

bool FDLScheduler::scheduleAtom(HWAtom *A) {
  assert(!A->isScheduled() && "A already scheduled!");
  DEBUG(A->print(dbgs()));
  unsigned step = FDInfo->getASAPStep(A);
  if (FDInfo->getTimeFrame(A) > 1) {
    step = findBestStep(A);
    DEBUG(dbgs() << "\n\nbest step: " << step << "\n");
    // If we can not schedule A.
    if (step == 0) {
      DEBUG(dbgs() << " Can not find avaliable step!\n\n");
      return false;
    }
  }

  A->scheduledTo(step);
  FDInfo->buildFDInfo();
  return scheduleCriticalPath();
}

bool FDLScheduler::scheduleQueue(AtomQueueType &Queue) {
  while (!Queue.empty()) {
    // TODO: Short the list
    HWAOpFU *A = Queue.top();
    Queue.pop();

    if (A->isScheduled())
      continue;

    DEBUG(dbgs() << " Schedule Node:-------------------\n");
    if (!scheduleAtom(A))
      return false;

    Queue.reheapify();
  }

  return true;
}

Pass *esyn::createFDLSchedulePass() {
  return new FDLScheduler();
}
