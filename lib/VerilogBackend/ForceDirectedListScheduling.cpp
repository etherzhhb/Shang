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
    State->resetSchedule(StartStep);
    buildFDInfo(true);

    fds_sort s(*this);
    AtomQueueType AQueue(s);

    fillQueue(AQueue, State->begin(), State->end());

    if (!scheduleQueue(AQueue))
      lengthenCriticalPath();
    else // Break the loop if we schedule successful.
      break;
  }

  schedulePassiveAtoms();

  // Set the Initial Interval to the total slot, so we can generate the correct
  // control logic for loop if MS is disable.
  if (State->haveSelfLoop())
    State->setII(State->getTotalSlot());
  DEBUG(dumpTimeFrame());
}


//===----------------------------------------------------------------------===//

bool ForceDirectedModuloScheduler::scheduleAtII() {
  fds_sort s(*this);
  AtomQueueType AQueue(s);

  // Schedule other nodes.
  AQueue.clear();
  fillQueue(AQueue, State->begin(), State->end());

  return scheduleQueue(AQueue);
}

void ForceDirectedModuloScheduler::scheduleState() {
  unsigned StartStep = HI->getTotalCycle();
  State->scheduledTo(StartStep);

  // Ensure us can schedule the critical path.
  for (;;) {
    buildFDInfo(true);
    if (scheduleCriticalPath())
      break;
    DEBUG(dumpTimeFrame());
    // TODO: check if we could ever schedule these node without breaking the
    // resource constrain by check the DG.
    // If the resource average DG is bigger than the total available resource
    // we can never schedule the nodes without breaking the resource constrain.
    State->resetSchedule(StartStep);
    lengthenCriticalPath();
  }

  // Dirty Hack: Search the solution by increasing MII and critical path
  // alternatively.

  setMII(II);
  for (;;) {
    buildFDInfo(true);
    if (scheduleCriticalPath())
      break;
    DEBUG(dumpTimeFrame());
    lengthenMII();
    State->resetSchedule(StartStep);
  }

  bool lastIncMII = true;
  for (;;) {
    bool SchedSucc = scheduleCriticalPath();
    assert(SchedSucc
      && "Why nodes critical path can not be schedule since DG is ok?");

    if (scheduleAtII()) {
      DEBUG(dumpTimeFrame());
      DEBUG(dumpDG());
      // Set up the initial interval.
      State->setII(getMII());
      break;
    } else if (lastIncMII) {
      lengthenCriticalPath();
      lastIncMII = false;
    } else {
      lengthenMII();
      II = getMII();
      lastIncMII = true;
    }
    // Prepare for next schedule.
    State->resetSchedule(StartStep);
    buildFDInfo(true);
  }

  schedulePassiveAtoms();
}

//===----------------------------------------------------------------------===//

void ForceDirectedSchedulingBase::schedulePassiveAtoms() {
  for (FSMState::iterator I = State->begin(), E = State->end();
      I != E; ++I) {
    HWAtom *A = *I;
    if (A->isScheduled())
      continue;

    DEBUG(A->print(dbgs()));
    unsigned step = getASAPStep(A);
    A->scheduledTo(step);
    buildFDInfo(false);
    updateSTF();
    bool res = scheduleCriticalPath();

    assert(res && "Why A can not schedule?");
  }
}

bool ForceDirectedSchedulingBase::scheduleCriticalPath() {
  for (FSMState::iterator I = State->begin(), E = State->end();
      I != E; ++I) {
    HWAtom *A = *I;
  
    if (A->isScheduled() || getTimeFrame(A) != 1)
      continue;

    unsigned step = getASAPStep(A);
    DEBUG(A->print(dbgs()));
    DEBUG(dbgs() << " asap step: " << step << "\n");
    A->scheduledTo(step);
  }
  // Do not need to update STF.
  buildFDInfo(false);
  return isResourceConstraintPreserved();
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
  for (unsigned i = getASAPStep(A), e = getALAPStep(A) + 1;
      i != e; ++i) {
    DEBUG(dbgs() << "At Step " << i << "\n");
    double Force = computeForce(A, i, i);
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
  unsigned step = getASAPStep(A);
  if (getTimeFrame(A) > 1) {
    step = findBestStep(A);
    DEBUG(dbgs() << "\n\nbest step: " << step << "\n");
    // If we can not schedule A.
    if (step == 0) {
      DEBUG(dbgs() << " Can not find avaliable step!\n\n");
      return false;
    }
  }

  A->scheduledTo(step);
  buildFDInfo(false);
  updateSTF();
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
  //  State->resetSchedule(StartStep);
  //  buildFDInfo(true);

  //  fds_sort s(FDInfo);
  //  AtomQueueType AQueue(s);

  //  fillQueue(AQueue, State->begin(), State->end());

  //  if (!scheduleQueue(AQueue))
  //    lengthenCriticalPath();
  //  else // Break the loop if we schedule successful.
  //    break;
  //}

  schedulePassiveAtoms();

  // Set the Initial Interval to the total slot, so we can generate the correct
  // control logic for loop if MS is disable.
  if (State->haveSelfLoop())
    State->setII(State->getTotalSlot());
  DEBUG(dumpTimeFrame());
}

void ForceDirectedScheduler::findBestSink() {

}

double ForceDirectedScheduler::trySinkAtom(HWAtom *A, TimeFrame &NewTimeFrame) {
  unsigned ASAP = getASAPStep(A), ALAP = getALAPStep(A);

  double ASAPForce = computeForce(A, ASAP, ASAP),
         ALAPForce = computeForce(A, ALAP, ALAP);

  double FMax = std::max(ASAPForce, ALAPForce),
         FMin = std::min(ASAPForce, ALAPForce);

  double FMinStar = FMin;

  if (ASAP + 1 < ALAP)
    FMinStar = std::min(FMinStar, 0.0);
  else
    assert(ASAP + 1 == ALAP && "Broken time frame!");
  
  double FGain = FMax - FMinStar;

  if (ASAPForce >= ALAPForce)
    NewTimeFrame = std::make_pair(ASAP, ALAP -1);
  else
    NewTimeFrame = std::make_pair(ASAP + 1, ALAP);
  
  return FGain;
}
