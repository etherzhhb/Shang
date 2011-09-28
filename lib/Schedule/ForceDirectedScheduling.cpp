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
bool SchedulingBase::fds_sort::operator()(const VSUnit* LHS,
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

bool IterativeModuloScheduling::scheduleState() {
  ExcludeSlots.assign(State.getNumSUnits(), std::set<unsigned>());
  setCriticalPathLength(VSUnit::MaxSlot);

  fds_sort s(*this);

  while (!isAllSUnitScheduled()) {
    State.resetSchedule();
    buildTimeFrame();
    // Reset exclude slots and resource table.
    resetRT();

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

        if (!A->getFUId().isTrivial() && !tryTakeResAtStep(A, i))
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
        VSUnit *Blocking = findBlockingSUnit(A, EarliestUntry);
        assert(Blocking && "No one blocking?");
        excludeStep(Blocking, EarliestUntry);

        unscheduleSU(Blocking);
        scheduleSU(A, EarliestUntry);

        ToSched.push(Blocking);
      }

      buildTimeFrame();
      ToSched.reheapify();
    }
  }
  DEBUG(buildTimeFrame());
  DEBUG(dumpTimeFrame());

  assert(isResourceConstraintPreserved()
         && "Resource constraints not preserved!");
  return true;
}

bool IterativeModuloScheduling::isStepExcluded(VSUnit *A, unsigned step) {
  assert(getMII() && "IMS only work on Modulo scheduling!");
  assert(!A->getFUId().isTrivial() && "Unexpected trivial sunit!");

  unsigned ModuloStep = computeStepKey(step);
  return ExcludeSlots[A->getIdx()].count(ModuloStep);
}

void IterativeModuloScheduling::excludeStep(VSUnit *A, unsigned step) {
  assert(getMII() && "IMS only work on Modulo scheduling!");
  assert(!A->getFUId().isTrivial() && "Unexpected trivial sunit!");

  unsigned ModuloStep = computeStepKey(step);
  ExcludeSlots[A->getIdx()].insert(ModuloStep);
}

VSUnit *IterativeModuloScheduling::findBlockingSUnit(VSUnit *U, unsigned step) {
  FuncUnitId FU = U->getFUId();
  unsigned Latency = U->getLatency();

  step = computeStepKey(step);

  for (VSchedGraph::iterator I = State.begin(), E = State.end(); I != E; ++I) {
    VSUnit *A = *I;
    if (A->getFUId().isTrivial() || !A->isScheduled() || A->getFUId() != FU)
      continue;

    unsigned CurStep = computeStepKey(A->getSlot());
    if (CurStep <= step && CurStep + Latency > step)
      return A;
  }

  return 0;
}

bool IterativeModuloScheduling::isAllSUnitScheduled() {
  for (VSchedGraph::iterator I = State.begin(), E = State.end(); I != E; ++I) {
    VSUnit *A = *I;
    if (!A->isScheduled())
      return false;
  }

  return true;
}

bool ASAPScheduler::scheduleState() {
  State.getEntryRoot()->scheduledTo(State.getStartSlot());

  for (VSchedGraph::iterator I = State.begin() + 1, E = State.end(); I != E;
       ++I) {
    VSUnit *A = *I;
    unsigned NewStep = 0;

    for (VSUnit::dep_iterator DI = A->dep_begin(), DE = A->dep_end();
         DI != DE; ++DI) {
      // Ignore the loop carried edges.
      if (DI.getEdge()->isLoopCarried()) continue;

      const VSUnit *Dep = *DI;

      assert(Dep->isScheduled() && "Dependence SU not scheduled!");
      unsigned Step = Dep->getDetailStep() + DI.getEdge()->getDetailLatency();

      NewStep = std::max(Step, NewStep);
    }

    NewStep = OpSlot::detailStepCeil(NewStep, A->hasDatapath()).getSlot();
    while (!tryTakeResAtStep(A, NewStep))
      ++NewStep;

    A->scheduledTo(NewStep);
  }

  return true;
}
