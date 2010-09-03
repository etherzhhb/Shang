//===- ForceDirectedListScheduling.cpp - ForceDirected Scheduler -*- C++ -*-===//
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
// This file implement the Force Direct List Schedulers
//
//===----------------------------------------------------------------------===//

#include "ForceDirectedScheduling.h"

#define DEBUG_TYPE "vbe-fdls"
#include "llvm/Support/Debug.h"

using namespace llvm;
using namespace esyn;

//===----------------------------------------------------------------------===//
bool
ForceDirectedListSchedulingBase::fds_sort::operator()(const HWAOpFU* LHS,
                                                      const HWAOpFU* RHS) const {
  HWFUnit *LFU = LHS->getFUnit(), *RFU = RHS->getFUnit();
  unsigned LTFU = LFU->getTotalFUs(), RTFU = RFU->getTotalFUs();
  // Schedule the atom with less available function unit first.
  if (LTFU > RTFU) return true;
  if (LTFU < RTFU) return false;

  unsigned LTF = Info.getTimeFrame(LHS), RTF = Info.getTimeFrame(RHS);
  // Schedule the low mobility nodes first.
  if (LTF > RTF) return true; // Place RHS first.
  if (LTF < RTF) return false;

  return LHS->getIdx() > RHS->getIdx();
}


//===----------------------------------------------------------------------===//

void ForceDirectedListScheduler::scheduleState() {
  unsigned StartStep = HI->getTotalCycle();
  for(;;) {
    CurState->resetSchedule(StartStep);
    FDInfo.buildFDInfo(true);

    fds_sort s(FDInfo);
    AtomQueueType AQueue(s);

    fillQueue(AQueue, CurState->begin(), CurState->end());

    if (!scheduleQueue(AQueue))
      FDInfo.lengthenCriticalPath();
    else // Break the loop if we schedule successful.
      break;
  }

  schedulePassiveAtoms();

  // Set the Initial Interval to the total slot, so we can generate the correct
  // control logic for loop if MS is disable.
  if (CurState->haveSelfLoop())
    CurState->setII(CurState->getTotalSlot());
  DEBUG(FDInfo.dumpTimeFrame());
}


//===----------------------------------------------------------------------===//

bool ForceDirectedModuloScheduler::scheduleAtII() {
  fds_sort s(FDInfo);
  AtomQueueType AQueue(s);

  // Schedule other nodes.
  AQueue.clear();
  fillQueue(AQueue, CurState->begin(), CurState->end());

  return scheduleQueue(AQueue);
}

void ForceDirectedModuloScheduler::scheduleState() {
  unsigned StartStep = HI->getTotalCycle();
  FDInfo.scheduleAtomTo(CurState, StartStep);

  // Ensure us can schedule the critical path.
  for (;;) {
    FDInfo.buildFDInfo(true);
    if (scheduleCriticalPath())
      break;
    DEBUG(FDInfo.dumpTimeFrame());
    // TODO: check if we could ever schedule these node without breaking the
    // resource constrain by check the DG.
    // If the resource average DG is bigger than the total available resource
    // we can never schedule the nodes without breaking the resource constrain.
    CurState->resetSchedule(StartStep);
    FDInfo.lengthenCriticalPath();
  }

  // Dirty Hack: Search the solution by increasing MII and critical path
  // alternatively.

  FDInfo.setMII(II);
  for (;;) {
    FDInfo.buildFDInfo(true);
    if (scheduleCriticalPath())
      break;
    DEBUG(FDInfo.dumpTimeFrame());
    FDInfo.lengthenMII();
    CurState->resetSchedule(StartStep);
  }

  bool lastIncMII = true;
  for (;;) {
    bool SchedSucc = scheduleCriticalPath();
    assert(SchedSucc
      && "Why nodes critical path can not be schedule since DG is ok?");

    if (scheduleAtII()) {
      DEBUG(FDInfo.dumpTimeFrame());
      DEBUG(FDInfo.dumpDG());
      // Set up the initial interval.
      CurState->setII(FDInfo.getMII());
      break;
    } else if (lastIncMII) {
      FDInfo.lengthenCriticalPath();
      lastIncMII = false;
    } else {
      FDInfo.lengthenMII();
      II = FDInfo.getMII();
      lastIncMII = true;
    }
    // Prepare for next schedule.
    CurState->resetSchedule(StartStep);
    FDInfo.buildFDInfo(true);
  }

  schedulePassiveAtoms();
}

//===----------------------------------------------------------------------===//

void ForceDirectedSchedulingBase::schedulePassiveAtoms() {
  for (FSMState::iterator I = CurState->begin(), E = CurState->end();
      I != E; ++I) {
    HWAtom *A = *I;
    if (A->isScheduled())
      continue;

    DEBUG(A->print(dbgs()));
    unsigned step = FDInfo.getASAPStep(A);
    FDInfo.scheduleAtomTo(A, step);
    FDInfo.buildFDInfo(false);
    bool res = scheduleCriticalPath();

    assert(res && "Why A can not schedule?");
  }
}

bool ForceDirectedSchedulingBase::scheduleCriticalPath() {
  for (FSMState::iterator I = CurState->begin(), E = CurState->end();
      I != E; ++I) {
    HWAtom *A = *I;
  
    if (A->isScheduled() || FDInfo.getTimeFrame(A) != 1)
      continue;

    unsigned step = FDInfo.getASAPStep(A);
    DEBUG(A->print(dbgs()));
    DEBUG(dbgs() << " asap step: " << step << "\n");
    FDInfo.scheduleAtomTo(A, step);
  }

  FDInfo.buildFDInfo(false);
  return FDInfo.isResourceConstraintPreserved();
}
//===----------------------------------------------------------------------===//

template<class It>
void ForceDirectedListSchedulingBase::fillQueue(AtomQueueType &Queue, It begin, It end,
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

unsigned ForceDirectedListSchedulingBase::findBestStep(HWAtom *A) {
  std::pair<unsigned, double> BestStep = std::make_pair(0, 1e32);
  DEBUG(dbgs() << "\tScan for best step:\n");
  // For each possible step:
  for (unsigned i = FDInfo.getASAPStep(A), e = FDInfo.getALAPStep(A) + 1;
      i != e; ++i) {
    DEBUG(dbgs() << "At Step " << i << "\n");

    // Temporary schedule A to i so we can get a more accurate pred and succ
    // force. Because the back edge constraint from A will be considered.
    FDInfo.scheduleAtomTo(A, i);
    FDInfo.buildTimeFrame();
    double Force = FDInfo.computeForce(A);
    DEBUG(dbgs() << " Force: " << Force);
    if (Force < BestStep.second)
      BestStep = std::make_pair(i, Force);

    DEBUG(dbgs() << '\n');
  }
  return BestStep.first;
}

bool ForceDirectedListSchedulingBase::scheduleAtom(HWAtom *A) {
  assert(!A->isScheduled() && "A already scheduled!");
  DEBUG(A->print(dbgs()));
  unsigned step = FDInfo.getASAPStep(A);
  if (FDInfo.getTimeFrame(A) > 1) {
    step = findBestStep(A);
    DEBUG(dbgs() << "\n\nbest step: " << step << "\n");
    // If we can not schedule A.
    if (step == 0) {
      DEBUG(dbgs() << " Can not find avaliable step!\n\n");
      return false;
    }
  }

  FDInfo.scheduleAtomTo(A, step);
  FDInfo.buildFDInfo(false);
  return scheduleCriticalPath();
}

bool ForceDirectedListSchedulingBase::scheduleQueue(AtomQueueType &Queue) {
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

//===----------------------------------------------------------------------===//

void ForceDirectedScheduler::scheduleState() {

  unsigned StartStep = HI->getTotalCycle();

  //for(;;) {
  //  CurState->resetSchedule(StartStep);
  //  FDInfo.buildFDInfo(true);

  //  fds_sort s(FDInfo);
  //  AtomQueueType AQueue(s);

  //  fillQueue(AQueue, CurState->begin(), CurState->end());

  //  if (!scheduleQueue(AQueue))
  //    FDInfo.lengthenCriticalPath();
  //  else // Break the loop if we schedule successful.
  //    break;
  //}

  schedulePassiveAtoms();

  // Set the Initial Interval to the total slot, so we can generate the correct
  // control logic for loop if MS is disable.
  if (CurState->haveSelfLoop())
    CurState->setII(CurState->getTotalSlot());
  DEBUG(FDInfo.dumpTimeFrame());
}

void ForceDirectedScheduler::findBestSink() {

}

void ForceDirectedScheduler::trySinkAtom(HWAtom *A) {
  unsigned ASAP = FDInfo.getASAPStep(A), ALAP = FDInfo.getALAPStep(A);
  
  //

}
