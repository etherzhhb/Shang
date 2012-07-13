//===-- ForceDirectedScheduling.cpp - ForceDirected Scheduler ---*- C++ -*-===//
//
// Copyright: 2011 by SYSU EDA Group. all rights reserved.
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
struct ims_sort {
  SchedulingBase &Info;
  ims_sort(SchedulingBase &s) : Info(s) {}
  bool operator() (const VSUnit *LHS, const VSUnit *RHS) const;
};

bool ims_sort::operator()(const VSUnit* LHS, const VSUnit* RHS) const {
  // Schedule the sunit that taking non-trivial function unit first.
  FuncUnitId LHSID = LHS->getFUId(), RHSID = RHS->getFUId();
  if (!LHSID.isTrivial() && RHSID.isTrivial()) return false;
  if (LHSID.isTrivial() && !RHSID.isTrivial()) return true;
  // Schedule the schedule unit with less available function unit first.
  if (!LHSID.isBound() && RHSID.isBound()) return true;
  if (LHSID.isBound() && !RHSID.isBound()) return false;

  unsigned LALAP = Info.getALAPStep(LHS), RALAP = Info.getALAPStep(RHS);
  if (LALAP > RALAP) return true;
  if (LALAP < RALAP) return false;

  unsigned LASAP = Info.getASAPStep(LHS), RASAP = Info.getASAPStep(RHS);
  if (LASAP > RASAP) return true;
  if (LASAP < RASAP) return false;

  return LHS->getIdx() > RHS->getIdx();
}

bool IterativeModuloScheduling::scheduleState() {
  State.resetSchedule(getMII());
  buildTimeFrame();
  // Reset exclude slots and resource table.
  resetRT();

  typedef PriorityQueue<VSUnit*, std::vector<VSUnit*>, ims_sort> IMSQueueType;
  IMSQueueType ToSched(State.sched_begin() + 1, State.sched_end(),
                       ims_sort(*this));
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

    // We had run out of slots
    if (EarliestUntry == 0) {
      return false;
    }

    // If a cannot be schedule but have some untry slot.
    if(!A->isScheduled()) {
      assert(!A->getFUId().isTrivial()
              && "SUnit can be schedule only because resource conflict!");
      // Unschedule all blocking schedule units.
      while (VSUnit *Blocking = findBlockingSUnit(A, EarliestUntry)) {
        assert(Blocking && "No one blocking?");
        excludeStep(Blocking, Blocking->getSlot());

        unscheduleSU(Blocking);
        ToSched.push(Blocking);
      }

      scheduleSU(A, EarliestUntry);
    }

    buildTimeFrame();
    ToSched.reheapify();
  }
  DEBUG(buildTimeFrame());
  DEBUG(dumpTimeFrame());

#ifndef NDEBUG
  verifyFUUsage();
#endif

  return true;
}

bool IterativeModuloScheduling::isStepExcluded(VSUnit *A, unsigned step) {
  assert(getMII() && "IMS only work on Modulo scheduling!");
  assert(!A->getFUId().isTrivial() && "Unexpected trivial sunit!");

  unsigned ModuloStep = computeStepKey(step);
  return ExcludeSlots[A].count(ModuloStep);
}

void IterativeModuloScheduling::excludeStep(VSUnit *A, unsigned step) {
  assert(getMII() && "IMS only work on Modulo scheduling!");
  assert(!A->getFUId().isTrivial() && "Unexpected trivial sunit!");

  unsigned ModuloStep = computeStepKey(step);
  ExcludeSlots[A].insert(ModuloStep);
}

VSUnit *IterativeModuloScheduling::findBlockingSUnit(VSUnit *U, unsigned step) {
  FuncUnitId FU = U->getFUId();
  unsigned Latency = U->getLatency();
  VFUs::FUTypes UTy = FU.getFUType();

  step = computeStepKey(step);

  typedef VSchedGraph::sched_iterator it;
  for (it I = State.sched_begin(), E = State.sched_end(); I != E; ++I) {
    VSUnit *A = *I;
    if (A->getFUId().isTrivial() || !A->isScheduled())
      continue;

    unsigned CurStep = A->getSlot();

    VFUs::FUTypes ATy = A->getFUType();
    // Blocked by PipelineStage/PipelineBreaker?
    if (UTy == VFUs::BRam && (ATy == VFUs::MemoryBus || ATy == VFUs::CalleeFN))
      --CurStep;
    else if ((UTy == VFUs::MemoryBus||UTy == VFUs::CalleeFN)&&ATy == VFUs::BRam)
      ++CurStep;
    else if (A->getFUId() != FU)
      continue;

    CurStep = computeStepKey(CurStep);

    if (CurStep <= step && CurStep + Latency > step)
      return A;
  }

  return 0;
}

bool IterativeModuloScheduling::isAllSUnitScheduled() {
  typedef VSchedGraph::sched_iterator it;
  for (it I = State.sched_begin(), E = State.sched_end(); I != E; ++I) {
    VSUnit *A = *I;
    if (!A->isScheduled())
      return false;
  }

  return true;
}

bool ASAPScheduler::scheduleState() {
  State.getEntryRoot()->scheduledTo(State.getStartSlot());
  BasicLinearOrderGenerator BLOG(*this);
  BLOG.addLinOrdEdge();

  typedef VSchedGraph::sched_iterator it;
  for (it I = State.sched_begin() + 1, E = State.sched_end(); I != E; ++I) {
    VSUnit *A = *I;
    assert(A->isControl() && "Unexpected datapath operation to schedule!");
    unsigned NewStep = 0;

    for (VSUnit::dep_iterator DI = A->dep_begin(), DE = A->dep_end();
         DI != DE; ++DI) {
      // Ignore the loop carried edges.
      if (DI.getEdge()->isLoopCarried()) continue;

      const VSUnit *Dep = *DI;

      assert(Dep->isScheduled() && "Dependence SU not scheduled!");
      unsigned Step = Dep->getSlot() + DI.getEdge()->getLatency();

      NewStep = std::max(Step, NewStep);
    }

    while (!tryTakeResAtStep(A, NewStep))
      ++NewStep;

    A->scheduledTo(NewStep);
  }

  return true;
}

struct alap_less {
  SchedulingBase &Info;
  alap_less(SchedulingBase &s) : Info(s) {}
  bool operator() (const VSUnit *LHS, const VSUnit *RHS) const {
    // Ascending order using ALAP.
    if (Info.getALAPStep(LHS) < Info.getALAPStep(RHS)) return true;
    if (Info.getALAPStep(LHS) > Info.getALAPStep(RHS)) return false;

    // Tie breaker 1: ASAP.
    if (Info.getASAPStep(LHS) < Info.getASAPStep(RHS)) return true;
    if (Info.getASAPStep(LHS) > Info.getASAPStep(RHS)) return false;

    // Tie breaker 2: Original topological order.
    return LHS->getIdx() < RHS->getIdx();
  }
};

void BasicLinearOrderGenerator::addLinOrdEdge() const {
  ConflictListTy ConflictList;

  typedef VSchedGraph::sched_iterator sched_it;
  for (sched_it I = S->sched_begin(), E = S->sched_end(); I != E; ++I) {
    VSUnit *U = *I;
    FuncUnitId Id = U->getFUId();

    // FIXME: Detect mutually exclusive predicate condition.
    if (!Id.isBound()) continue;

    if (Id.getFUType() == VFUs::MemoryBus || Id.getFUType() == VFUs::BRam)
      // These operations need to be handled specially.
      Id = FuncUnitId();

    ConflictList[Id].push_back(U);
  }

  S.buildTimeFrame();

  typedef ConflictListTy::iterator iterator;
  for (iterator I = ConflictList.begin(), E = ConflictList.end(); I != E; ++I) {
    std::vector<VSUnit*> &SUs = I->second;
    std::sort(SUs.begin(), SUs.end(), alap_less(S));

    // A trivial id means SUs contains the MemoryBus and BRam operations,
    // which need to be handle carefully.
    if (I->first == FuncUnitId())
      addLinOrdEdgeForMemOp(SUs);
    else
      addLinOrdEdge(I->second);
  }

  S->topologicalSortScheduleUnits();
}

void BasicLinearOrderGenerator::addLinOrdEdge(std::vector<VSUnit*> &SUs) const {
  VSUnit *LaterSU = SUs.back();
  SUs.pop_back();

  while (!SUs.empty()) {
    VSUnit *EalierSU = SUs.back();
    SUs.pop_back();

    // Build a dependence edge from EalierSU to LaterSU.
    // TODO: Add an new kind of edge: Constraint Edge, and there should be
    // hard constraint and soft constraint.
    LaterSU->addDep(VDCtrlDep::CreateDep(EalierSU, EalierSU->getLatency()));

    LaterSU = EalierSU;
  }
}

void BasicLinearOrderGenerator::addLinOrdEdgeForMemOp(std::vector<VSUnit*> &SUs) const {
  std::map<int, VSUnit*> LastSUs;

  while (!SUs.empty()) {
    VSUnit *U = SUs.back();
    SUs.pop_back();

    FuncUnitId Id = U->getFUId();
    int FUNum = Id.getFUNum();
    bool IsMemBus = (Id.getFUType() == VFUs::MemoryBus);
    if (IsMemBus) FUNum = - FUNum;

    typedef std::map<int, VSUnit*>::iterator it;
    for (it I = LastSUs.begin(), E = LastSUs.end(); I != E; ++I) {
      // No need to add dependency edges between the same kind of operations.
      // That is MemBus v.s. MemBus or BRam v.s. BRam
      if ((I->first > 0) == (FUNum > 0)) continue;

      I->second->addDep(VDCtrlDep::CreateDep(U, U->getLatency()));
    }

    VSUnit *&LaterSU = LastSUs[FUNum];
    if (LaterSU)
      LaterSU->addDep(VDCtrlDep::CreateDep(U, U->getLatency()));

    LaterSU = U;
  }
}
