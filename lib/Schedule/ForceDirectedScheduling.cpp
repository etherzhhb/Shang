//===-- ForceDirectedScheduling.cpp - ForceDirected Scheduler ---*- C++ -*-===//
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
// This file implement the Force Direct Schedulers
//
//===----------------------------------------------------------------------===//

#include "SchedulingBase.h"

#define DEBUG_TYPE "vbe-fds"
#include "llvm/Support/Debug.h"

using namespace llvm;

//===----------------------------------------------------------------------===//
bool
FDListScheduler::fds_sort::operator()(const VSUnit* LHS,
                                      const VSUnit* RHS) const {
  // Schedule the sunit that taking non-trivial function unit first.
  FuncUnitId LHSID = LHS->getFUId(), RHSID = RHS->getFUId();
  if (!LHSID.isTrivial() && RHSID.isTrivial())
    return false;
  if (LHSID.isTrivial() && !RHSID.isTrivial())
    return true;
  if (!LHSID.isTrivial() && !RHSID.isTrivial()) {
    unsigned LTFUs = LHSID.getTotalFUs(), RTFUs = RHSID.getTotalFUs();
    // Schedule the schedule unit with less available function unit first.
    if (LTFUs > RTFUs) return true;
    if (LTFUs < RTFUs) return false;
  }

  unsigned LTF = Info.getTimeFrame(LHS), RTF = Info.getTimeFrame(RHS);
  // Schedule the low mobility nodes first.
  if (LTF > RTF) return true; // Place RHS first.
  if (LTF < RTF) return false;

  unsigned LALAP = Info.getALAPStep(LHS), RALAP = Info.getALAPStep(RHS);
  if (LALAP > RALAP) return true;
  if (LALAP < RALAP) return false;

  // 
  unsigned LASAP = Info.getASAPStep(LHS), RASAP = Info.getASAPStep(RHS);
  if (LASAP > RASAP) return true;
  if (LASAP < RASAP) return false;

  
  return LHS->getIdx() > RHS->getIdx();
}


//===----------------------------------------------------------------------===//

bool FDListScheduler::scheduleState() {
  buildFDepHD(true);
  if (!scheduleCriticalPath(false))
    return false;

  fds_sort s(*this);
  SUnitQueueType AQueue(s);

  fillQueue(AQueue, State.begin(), State.end());

  if (!scheduleQueue(AQueue)) {
    DEBUG(dumpDG());
    DEBUG(dbgs() << "Schedule fail! ExtraResReq: "
                 << getExtraResReq() << '\n');
    return false;
  }

  schedulePassiveSUnits();

  DEBUG(dumpTimeFrame());
  DEBUG(dumpDG());
  return true;
}

template<class It>
void FDListScheduler::fillQueue(SUnitQueueType &Queue, It begin, It end,
                                VSUnit *FirstNode) {
  for (It I = begin, E = end; I != E; ++I) {
    VSUnit *A = *I;

    // Do not push the FirstNode into the queue.
    if (A == FirstNode || A->isScheduled())
      continue;
    
    // Do not push the SUnit that taking trivial function unit.
    if (A->getFUId().isTrivial())
      continue;
    
    Queue.push(A);
  }
  //
  Queue.reheapify();
}

unsigned FDListScheduler::findBestStep(VSUnit *A) {
  unsigned ASAP = getASAPStep(A), ALAP = getALAPStep(A);
  // Time frame is 1, we do not have other choice.
  if (ASAP == ALAP) return ASAP;

  std::pair<unsigned, double> BestStep = std::make_pair(0, 1e32);
  DEBUG(dbgs() << "\tScan for best step:\n");
  // For each possible step:
  for (unsigned i = ASAP, e = ALAP + 1; i != e; ++i) {
    DEBUG(dbgs() << "At Step " << i << "\n");
    double Force = computeForce(A, i, i);
    DEBUG(dbgs() << " Force: " << Force);
    if (Force < BestStep.second)
      BestStep = std::make_pair(i, Force);

    DEBUG(dbgs() << '\n');
  }
  return BestStep.first;
}

bool FDListScheduler::scheduleSUnit(VSUnit *A) {
  assert(!A->isScheduled() && "A already scheduled!");
  DEBUG(A->print(dbgs()));
  unsigned step = findBestStep(A);
  DEBUG(dbgs() << "\n\nbest step: " << step << "\n");
  A->scheduledTo(step);
  // Update FDepHD.
  buildFDepHD(false);
  return (isResourceConstraintPreserved() && scheduleCriticalPath(false));
}

bool FDListScheduler::scheduleQueue(SUnitQueueType &Queue) {
  while (!Queue.empty()) {
    // TODO: Shor the list
    VSUnit *A = Queue.top();
    Queue.pop();

    if (A->isScheduled())
      continue;

    DEBUG(dbgs() << " Schedule Node:-------------------\n");
    if (!scheduleSUnit(A))
      return false;

    Queue.reheapify();
  }

  return true;
}

//===----------------------------------------------------------------------===//

bool FDScheduler::scheduleState() {
  buildFDepHD(true);
  if (!scheduleCriticalPath(false))
    return false;

  while (findBestSink()) {
    if (!scheduleCriticalPath(false)) {
      DEBUG(dumpDG());
      DEBUG(dbgs() << "Schedule fail! ExtraResReq: "
                   << getExtraResReq() << '\n');
      return false;
    }
  }

  schedulePassiveSUnits();

  DEBUG(dumpTimeFrame());
  DEBUG(dumpDG());
  return true;
}

bool FDScheduler::findBestSink() {
  TimeFrame BestSink;
  VSUnit *BestSinkSUnit = 0;
  double BestGain = -1.0;

  for (VSchedGraph::iterator I = State.begin(), E = State.end();
      I != E; ++I) {
    VSUnit *A = *I;
    if (A->isScheduled())
      continue;
    
    TimeFrame CurSink;

    double CurGain = trySinkSUnit(A, CurSink);
    if (CurGain > BestGain) {
      BestSinkSUnit = A;
      BestSink = CurSink;
      BestGain = CurGain;
    }
  }

  if (!BestSinkSUnit) return false;
  
  sinkSTF(BestSinkSUnit, BestSink.first, BestSink.second);
  if (getScheduleTimeFrame(BestSinkSUnit) == 1)
    BestSinkSUnit->scheduledTo(BestSink.first);
  buildFDepHD(false);
  updateSTF();
  return true;
}

double FDScheduler::trySinkSUnit(VSUnit *A, TimeFrame &NewTimeFrame) {
  // Build time frame to get the correct ASAP and ALAP.
  buildTimeFrame();

  unsigned ASAP = getASAPStep(A), ALAP = getALAPStep(A);

  double ASAPForce = computeForce(A, ASAP , ALAP - 1),
    ALAPForce = computeForce(A, ASAP + 1, ALAP);

  double FMax = std::max(ASAPForce, ALAPForce),
    FMin = std::min(ASAPForce, ALAPForce);

  double FMinStar = FMin;

  if (ASAP + 1 < ALAP)
    FMinStar = std::min(FMinStar, 0.0);
  else
    assert(ASAP + 1 == ALAP && "Broken time frame!");

  double FGain = FMax - FMinStar;

  // Discard the range with bigger force.
  if (ASAPForce > ALAPForce)
    NewTimeFrame = std::make_pair(ASAP + 1, ALAP);
  else
    NewTimeFrame = std::make_pair(ASAP, ALAP - 1);

  return FGain;
}

struct ims_sort {
  SchedulingBase &Info;
  ims_sort(SchedulingBase &s) : Info(s) {}
  bool operator() (const VSUnit *LHS, const VSUnit *RHS) const;
};

bool ims_sort::operator()(const VSUnit* LHS, const VSUnit* RHS) const {
  // Schedule the sunit that taking non-trivial function unit first.
  FuncUnitId LHSID = LHS->getFUId(), RHSID = RHS->getFUId();
  if (!LHSID.isTrivial() && RHSID.isTrivial())
    return false;
  if (LHSID.isTrivial() && !RHSID.isTrivial())
    return true;
  if (!LHSID.isTrivial() && !RHSID.isTrivial()) {
    unsigned LTFUs = LHSID.getTotalFUs(), RTFUs = RHSID.getTotalFUs();
    // Schedule the schedule unit with less available function unit first.
    if (LTFUs > RTFUs) return true;
    if (LTFUs < RTFUs) return false;
  }

  unsigned LASAP = Info.getASAPStep(LHS), RASAP = Info.getASAPStep(RHS);
  if (LASAP > RASAP) return true;
  if (LASAP < RASAP) return false;
  
  return LHS->getIdx() > RHS->getIdx();
}

bool IteractiveModuloScheduling::scheduleState() {
  ExcludeSlots.assign(State.getNumSUnits(), std::set<unsigned>());
  setCriticalPathLength(VSUnit::MaxSlot);

  fds_sort s(*this);

  while (!isAllSUnitScheduled()) {
    State.resetSchedule();
    buildTimeFrame();
    // Reset exclude slots and resource table.
    MRT.clear();

    typedef PriorityQueue<VSUnit*, std::vector<VSUnit*>, fds_sort> IMSQueueType;
    IMSQueueType ToSched(++State.begin(), State.end(), s);
    while (!ToSched.empty()) {
      VSUnit *A = ToSched.top();
      ToSched.pop();

      unsigned EarliestUntry = 0;
      for (unsigned i = getASAPStep(A), e = getALAPStep(A) + 1; i != e; ++i) {
        if (!A->getFUId().isTrivial() && isStepExcluded(A, i))
          continue;
        
        if (EarliestUntry == 0)
          EarliestUntry = i;
        
        if (!A->getFUId().isTrivial() && !isResAvailable(A->getFUId(), i))
          continue;

        // This is a available slot.
        A->scheduledTo(i);
        break;
      }

      if (EarliestUntry == 0) {
        increaseMII();
        break;
      } else if(!A->isScheduled()) {
        assert(!A->getFUId().isTrivial()
               && "SUnit can be schedule only because resource conflict!");
        VSUnit *Blocking = findBlockingSUnit(A->getFUId(), EarliestUntry);
        assert(Blocking && "No one blocking?");
        Blocking->resetSchedule();
        excludeStep(Blocking, EarliestUntry);
        // Resource table do not need to change.
        A->scheduledTo(EarliestUntry);
        ToSched.push(Blocking);
      }

      buildTimeFrame();
      ToSched.reheapify();
    }
  }
  DEBUG(buildTimeFrame());
  DEBUG(dumpTimeFrame());
  DEBUG(dumpDG());
  return true;
}

bool IteractiveModuloScheduling::isStepExcluded(VSUnit *A, unsigned step) {
  assert(getMII() && "IMS only work on Modulo scheduling!");
  assert(!A->getFUId().isTrivial() && "Unexpected trivial sunit!");

  unsigned ModuloStep = step % getMII();
  return ExcludeSlots[A->getIdx()].count(ModuloStep);
}

void IteractiveModuloScheduling::excludeStep(VSUnit *A, unsigned step) {
  assert(getMII() && "IMS only work on Modulo scheduling!");
  assert(!A->getFUId().isTrivial() && "Unexpected trivial sunit!");

  unsigned ModuloStep = step % getMII();
  ExcludeSlots[A->getIdx()].insert(ModuloStep);
}

VSUnit *IteractiveModuloScheduling::findBlockingSUnit(FuncUnitId FU, unsigned step) {
  for (VSchedGraph::iterator I = State.begin(), E = State.end(); I != E; ++I) {
    VSUnit *A = *I;
    if (A->getFUId().isTrivial() || !A->isScheduled() || A->getFUId() != FU)
      continue;
    if (A->getSlot() == step) return A; 
  }

  return 0;
}

bool IteractiveModuloScheduling::isResAvailable(FuncUnitId FU, unsigned step) {
  assert(getMII() && "IMS only work on Modulo scheduling!");
  // We will always have enough trivial resources.
  if (FU.isTrivial()) return true;

  unsigned ModuloStep = step % getMII();
  // Do all resource at step been reserve?
  if (MRT[FU][ModuloStep] >= FU.getTotalFUs())
    return false;

  ++MRT[FU][ModuloStep];
  
  return true;
}


bool IteractiveModuloScheduling::isAllSUnitScheduled() {
  for (VSchedGraph::iterator I = State.begin(), E = State.end();
      I != E; ++I) {
    VSUnit *A = *I;
    if (!A->isScheduled())
      return false;
  }

  return true;
}
