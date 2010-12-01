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

//===----------------------------------------------------------------------===//
bool
ForceDirectedListScheduler::fds_sort::operator()(const VSUnit* LHS,
                                                 const VSUnit* RHS) const {
  //// Schedule HWOpFU first.
  //if (isa<VSUnit>(LHS) && !isa<VSUnit>(RHS))
  //  return false;
  //if (!isa<VSUnit>(LHS) && isa<VSUnit>(RHS))
  //  return true;
  //if (isa<VSUnit>(LHS) && isa<VSUnit>(RHS)) {
  //  HWFUnit *LFU = cast<VSUnit>(LHS)->getFUnit(),
  //          *RFU = cast<VSUnit>(RHS)->getFUnit();
  //  unsigned LTFU = LFU->getTotalFUs(), RTFU = RFU->getTotalFUs();
  //  // Schedule the atom with less available function unit first.
  //  if (LTFU > RTFU) return true;
  //  if (LTFU < RTFU) return false;
  //}

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

bool ForceDirectedListScheduler::scheduleState() {
  buildFDInfo(true);
  if (!scheduleCriticalPath(false))
    return false;

  fds_sort s(*this);
  SUnitQueueType AQueue(s);

  fillQueue(AQueue, State->begin(), State->end());

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

//===----------------------------------------------------------------------===//

void ForceDirectedSchedulingBase::schedulePassiveSUnits() {
  for (VSchedGraph::iterator I = State->begin(), E = State->end();
      I != E; ++I) {
    VSUnit *A = *I;
    if (A->isScheduled())
      continue;

    DEBUG(A->print(dbgs()));
    unsigned step = getASAPStep(A);
    A->scheduledTo(step);
    buildFDInfo(false);
    updateSTF();
  }
}

bool ForceDirectedSchedulingBase::scheduleCriticalPath(bool refreshFDInfo) {
  if (refreshFDInfo)
    buildFDInfo(true);

  for (VSchedGraph::iterator I = State->begin(), E = State->end();
      I != E; ++I) {
    VSUnit *A = *I;
  
    if (A->isScheduled() || getTimeFrame(A) != 1)
      continue;

    unsigned step = getASAPStep(A);
    DEBUG(A->print(dbgs()));
    DEBUG(dbgs() << " asap step: " << step << " in critical path.\n");
    A->scheduledTo(step);
  }
  return isResourceConstraintPreserved();
}
//===----------------------------------------------------------------------===//

template<class It>
void ForceDirectedListScheduler::fillQueue(SUnitQueueType &Queue, It begin, It end,
                        VSUnit *FirstNode) {
  for (It I = begin, E = end; I != E; ++I) {
    VSUnit *A = *I;

    // Do not push the FirstNode into the queue.
    if (A == FirstNode || A->isScheduled())
      continue;
    
    Queue.push(A);
  }
  //
  Queue.reheapify();
}

unsigned ForceDirectedListScheduler::findBestStep(VSUnit *A) {
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

bool ForceDirectedListScheduler::scheduleSUnit(VSUnit *A) {
  assert(!A->isScheduled() && "A already scheduled!");
  DEBUG(A->print(dbgs()));
  unsigned step = findBestStep(A);
  DEBUG(dbgs() << "\n\nbest step: " << step << "\n");
  A->scheduledTo(step);
  // Update FDInfo.
  buildFDInfo(false);
  return (isResourceConstraintPreserved() && scheduleCriticalPath(false));
}

bool ForceDirectedListScheduler::scheduleQueue(SUnitQueueType &Queue) {
  while (!Queue.empty()) {
    // TODO: Short the list
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

bool ForceDirectedScheduler::scheduleState() {
  buildFDInfo(true);
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

bool ForceDirectedScheduler::findBestSink() {
  TimeFrame BestSink;
  VSUnit *BestSinkSUnit = 0;
  double BestGain = -1.0;

  for (VSchedGraph::iterator I = State->begin(), E = State->end();
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
  buildFDInfo(false);
  updateSTF();
  return true;
}

double ForceDirectedScheduler::trySinkSUnit(VSUnit *A,
                                                TimeFrame &NewTimeFrame) {
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
  ForceDirectedSchedulingBase &Info;
  ims_sort(ForceDirectedSchedulingBase &s) : Info(s) {}
  bool operator() (const VSUnit *LHS, const VSUnit *RHS) const;
};

bool ims_sort::operator()(const VSUnit* LHS, const VSUnit* RHS) const {
  // Schedule HWOpFU first.

  //  HWFUnit *LFU = cast<VSUnit>(LHS)->getFUnit(),
  //          *RFU = cast<VSUnit>(RHS)->getFUnit();
  //  unsigned LTFU = LFU->getTotalFUs(), RTFU = RFU->getTotalFUs();
  //  // Schedule the atom with less available function unit first.
  //  if (LTFU > RTFU) return true;
  //  if (LTFU < RTFU) return false;
  // 
  unsigned LASAP = Info.getASAPStep(LHS), RASAP = Info.getASAPStep(RHS);
  if (LASAP > RASAP) return true;
  if (LASAP < RASAP) return false;
  
  return LHS->getIdx() > RHS->getIdx();
}

bool IteractiveModuloScheduling::scheduleState() {
  ExcludeSlots.clear();
  setCriticalPathLength(VSUnit::MaxSlot);

  fds_sort s(*this);

  while (!isAllSUnitScheduled()) {
    State->resetSchedule();
    buildTimeFrame();
    // Reset exclude slots and resource table.
    MRT.clear();

    typedef PriorityQueue<VSUnit*, std::vector<VSUnit*>, fds_sort> IMSQueueType;
    IMSQueueType ToSched(++State->begin(), State->end(), s);
    while (!ToSched.empty()) {
      VSUnit *A = ToSched.top();
      ToSched.pop();

      unsigned EarliestUntry = 0;
      for (unsigned i = getASAPStep(A), e = getALAPStep(A) + 1; i != e; ++i) {
        if (isStepExcluded(A, i))
          continue;
        
        if (EarliestUntry == 0)
          EarliestUntry = i;
        
        if (A && !isResAvailable(A->getFUType(), i))
          continue;

        // This is a available slot.
        A->scheduledTo(i);
        break;
      }

      if (EarliestUntry == 0) {
        increaseMII();
        break;
      } else if(!A->isScheduled()) {
        assert(A && "A can be schedule only because resource conflict!");
        VSUnit *Blocking = findBlockingSUnit(A->getFUType(), EarliestUntry);
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
  unsigned ModuloStep = step % getMII();
  return ExcludeSlots[A].count(ModuloStep);
}

void IteractiveModuloScheduling::excludeStep(VSUnit *A, unsigned step) {
  assert(getMII() && "IMS only work on Modulo scheduling!");
  unsigned ModuloStep = step % getMII();
  ExcludeSlots[A].insert(ModuloStep);
}

VSUnit *IteractiveModuloScheduling::findBlockingSUnit(unsigned FUClass,
                                                      unsigned step) {
  for (VSchedGraph::iterator I = State->begin(), E = State->end(); I != E; ++I) {
    VSUnit *A = *I;
    if (!A->isScheduled() || A->getFUType() != FUClass)
      continue;
    if (A->getSlot() == step) return A; 
  }

  return 0;
}

bool IteractiveModuloScheduling::isResAvailable(unsigned FUClass,
                                                unsigned step) {
  //assert(getMII() && "IMS only work on Modulo scheduling!");
  //// We will always have enough trivial resources.
  //if (FU->getResType() == HWResType::Trivial)
  //  return true;

  //unsigned ModuloStep = step % getMII();
  //// Do all resource at step been reserve?
  //if (MRT[FU][ModuloStep] >= FU->getTotalFUs())
  //  return false;

  //++MRT[FU][ModuloStep];
  
  return true;
}


bool IteractiveModuloScheduling::isAllSUnitScheduled() {
  for (VSchedGraph::iterator I = State->begin(), E = State->end();
      I != E; ++I) {
    VSUnit *A = *I;
    if (!A->isScheduled())
      return false;
  }

  return true;
}
