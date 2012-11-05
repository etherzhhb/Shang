//===-- Schedulers.cpp - Implemenation of Schedulers ------------*- C++ -*-===//
//
//                      The Shang HLS frameowrk                               //
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implement the ASAP Scheduling and Iterative Modulo Scheduling
// algorithm.
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

typedef IterativeModuloScheduling::ScheduleResult ScheduleResult;
ScheduleResult IterativeModuloScheduling::scheduleLoop(){
  G.resetCPSchedule();
  buildTimeFrame();
  VSUnit *LoopOp = G.getLoopOp();
  assert(LoopOp && "Cannot find LoopOp in IMS scheduler!");
  // Schedule the LoopOp to the end of the first stage.
  unsigned LoopOpSlot = G.EntrySlot + getMII();
  if (getASAPStep(LoopOp) > LoopOpSlot)
    return IterativeModuloScheduling::MIITooSmall;

  LoopOp->scheduledTo(LoopOpSlot);

  // Reset exclude slots and resource table.
  resetRT();
  ExcludeSlots.clear();

  typedef PriorityQueue<VSUnit*, std::vector<VSUnit*>, ims_sort> IMSQueueType;
  IMSQueueType ToSched(cp_begin(&G) + 1, cp_end(&G), ims_sort(*this));
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
    if (EarliestUntry == 0)
      return ExcludeSlots[A].size() == getMII() ?
             IterativeModuloScheduling::MIITooSmall :
             IterativeModuloScheduling::Unknown;

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
  verifyFUUsage(cp_begin(&G), cp_end(&G));
#endif

  return IterativeModuloScheduling::Success;
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
  const MachineInstr *BlockingMI = getConflictedInst(U, step);
  return BlockingMI ? G.lookupSUnit(BlockingMI) : 0;
}

bool ASAPScheduler::scheduleState() {
  G.getEntryRoot()->scheduledTo(G.EntrySlot);
  buildTimeFrame();

  BasicLinearOrderGenerator::addLinOrdEdge(*this);

  for (iterator I = cp_begin(&G) + 1, E = cp_end(&G); I != E; ++I) {
    VSUnit *A = *I;
    assert(A->isControl() && "Unexpected datapath operation to schedule!");
    unsigned NewStep = 0;

    for (const_dep_it DI = dep_begin(A), DE = dep_end(A); DI != DE; ++DI) {
      // Ignore the loop carried edges.
      if (DI.isLoopCarried()) continue;

      const VSUnit *Dep = *DI;

      assert(Dep->isScheduled() && "Dependence SU not scheduled!");
      unsigned Step = Dep->getSlot() + DI.getLatency();

      NewStep = std::max(Step, NewStep);
    }

    while (!tryTakeResAtStep(A, NewStep)) {
      llvm_unreachable("Linear order generator should avoid this!");
      ++NewStep;
    }

    //scheduleSU(A, NewStep);
    A->scheduledTo(NewStep);
  }

  return true;
}
