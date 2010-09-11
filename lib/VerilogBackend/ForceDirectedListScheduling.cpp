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
ForceDirectedListScheduler::fds_sort::operator()(const HWAtom* LHS,
                                                 const HWAtom* RHS) const {
  // Schedule HWOpFU first.
  if (isa<HWAOpFU>(LHS) && !isa<HWAOpFU>(RHS))
    return false;
  if (!isa<HWAOpFU>(LHS) && isa<HWAOpFU>(RHS))
    return true;
  if (isa<HWAOpFU>(LHS) && isa<HWAOpFU>(RHS)) {
    HWFUnit *LFU = cast<HWAOpFU>(LHS)->getFUnit(),
            *RFU = cast<HWAOpFU>(RHS)->getFUnit();
    unsigned LTFU = LFU->getTotalFUs(), RTFU = RFU->getTotalFUs();
    // Schedule the atom with less available function unit first.
    if (LTFU > RTFU) return true;
    if (LTFU < RTFU) return false;
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

bool ForceDirectedListScheduler::scheduleState() {
  buildFDInfo(true);
  if (!scheduleCriticalPath(false))
    return false;

  fds_sort s(*this);
  AtomQueueType AQueue(s);

  fillQueue(AQueue, State->begin(), State->end());

  if (!scheduleQueue(AQueue)) {
    DEBUG(dumpDG());
    DEBUG(dbgs() << "Schedule fail! ExtraResReq: "
                 << getExtraResReq() << '\n');
    return false;
  }

  schedulePassiveAtoms();

  DEBUG(dumpTimeFrame());
  DEBUG(dumpDG());
  return true;
}

//===----------------------------------------------------------------------===//

void ForceDirectedSchedulingBase::schedulePassiveAtoms() {
  for (FSMState::iterator I = State->begin(), E = State->end();
      I != E; ++I) {
    HWAtom *A = *I;
    if (A->isScheduled())
      continue;
    assert(!isa<HWAOpFU>(A) && "OpFU not schedule?");
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

  for (FSMState::iterator I = State->begin(), E = State->end();
      I != E; ++I) {
    HWAtom *A = *I;
  
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
void ForceDirectedListScheduler::fillQueue(AtomQueueType &Queue, It begin, It end,
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

unsigned ForceDirectedListScheduler::findBestStep(HWAtom *A) {
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

bool ForceDirectedListScheduler::scheduleAtom(HWAtom *A) {
  assert(!A->isScheduled() && "A already scheduled!");
  DEBUG(A->print(dbgs()));
  unsigned step = findBestStep(A);
  DEBUG(dbgs() << "\n\nbest step: " << step << "\n");
  A->scheduledTo(step);
  // Update FDInfo.
  buildFDInfo(false);
  return (isResourceConstraintPreserved() && scheduleCriticalPath(false));
}

bool ForceDirectedListScheduler::scheduleQueue(AtomQueueType &Queue) {
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

  schedulePassiveAtoms();

  DEBUG(dumpTimeFrame());
  DEBUG(dumpDG());
  return true;
}

bool ForceDirectedScheduler::findBestSink() {
  TimeFrame BestSink;
  HWAtom *BestSinkAtom = 0;
  double BestGain = -1.0;

  for (FSMState::iterator I = State->begin(), E = State->end();
      I != E; ++I) {
    HWAtom *A = *I;
    if (A->isScheduled())
      continue;
    
    TimeFrame CurSink;

    double CurGain = trySinkAtom(A, CurSink);
    if (CurGain > BestGain) {
      BestSinkAtom = A;
      BestSink = CurSink;
      BestGain = CurGain;
    }
  }

  if (!BestSinkAtom) return false;
  
  sinkSTF(BestSinkAtom, BestSink.first, BestSink.second);
  if (getScheduleTimeFrame(BestSinkAtom) == 1)
    BestSinkAtom->scheduledTo(BestSink.first);
  buildFDInfo(false);
  updateSTF();
  return true;
}

double ForceDirectedScheduler::trySinkAtom(HWAtom *A,
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
  bool operator() (const HWAtom *LHS, const HWAtom *RHS) const;
};

bool ims_sort::operator()(const HWAtom* LHS, const HWAtom* RHS) const {
  // Schedule HWOpFU first.
  if (isa<HWAOpFU>(LHS) && !isa<HWAOpFU>(RHS))
    return false;
  if (!isa<HWAOpFU>(LHS) && isa<HWAOpFU>(RHS))
    return true;
  if (isa<HWAOpFU>(LHS) && isa<HWAOpFU>(RHS)) {
    HWFUnit *LFU = cast<HWAOpFU>(LHS)->getFUnit(),
            *RFU = cast<HWAOpFU>(RHS)->getFUnit();
    unsigned LTFU = LFU->getTotalFUs(), RTFU = RFU->getTotalFUs();
    // Schedule the atom with less available function unit first.
    if (LTFU > RTFU) return true;
    if (LTFU < RTFU) return false;
  }
  // 
  unsigned LASAP = Info.getASAPStep(LHS), RASAP = Info.getASAPStep(RHS);
  if (LASAP > RASAP) return true;
  if (LASAP < RASAP) return false;
  
  return LHS->getIdx() > RHS->getIdx();
}

bool IMS::scheduleState() {
  State->scheduledTo(HI->getTotalCycle());
  buildASAPStep();
  MRT.clear();
  std::map<HWAtom*, std::set<unsigned> > ExcludeSlots;

  unsigned CurStep = State->getSlot();

  ims_sort s(*this);

  while (!isAllAtomScheduled()) {
    std::vector<HWAtom*> ToSched(++State->begin(), State->end());
    std::sort(ToSched.begin(), ToSched.end(), s);

    while (!ToSched.empty()) {
      HWAtom *A = ToSched.back();
      ToSched.pop_back();

      unsigned EarliestUnTry = 0;
      bool AllExcluded = true;
      for (unsigned i = getASAPStep(A), e = getASAPStep(A) + getMII();
           i != e; ++i) {
        if (ExcludeSlots[A].count(i))
          continue;
        if (EarliestUnTry == 0)        
          EarliestUnTry = i;
        HWAOpFU *OF = 0;
        if (OF = dyn_cast<HWAOpFU>(A)) {
          if (!isResAvailable(OF->getFUnit(), i))
            continue;
          else
            isResAvailable(OF->getFUnit(), i, true);
        }
        
        // This is a available slot.
        A->scheduledTo(i);
      }

      if (AllExcluded) {
        increaseMII();
        break;
      } else {
        HWAOpFU *OF = dyn_cast<HWAOpFU>(A);
        assert(OF && "A can be schedule only because resource conflict!");
        HWAOpFU *Blocking = findBlockingAtom(OF->getFUnit(), EarliestUnTry);
        assert(Blocking && "No one blocking?");
        Blocking->resetSchedule();
        ExcludeSlots[Blocking].insert(EarliestUnTry);
        // Resource table do not need to change.
        OF->scheduledTo(EarliestUnTry);
      }
    }
  }

  DEBUG(dumpTimeFrame());
  DEBUG(dumpDG());
  return true;
}


HWAOpFU *IMS::findBlockingAtom(HWFUnit *FU, unsigned step) {
  for (FSMState::iterator I = State->begin(), E = State->end(); I != E; ++I) {
    HWAOpFU *OF = dyn_cast<HWAOpFU>(*I);
    if (OF == 0 || !OF->isScheduled() || OF->getFUnit() != FU)
      continue;
    if (OF->getSlot() == step) return OF; 
  }

  return 0;
}

bool IMS::isResAvailable(HWFUnit *FU, unsigned step, bool take) {
  assert(getMII() && "IMS only work on Modulo scheduling!");
  // We will always have enough trivial resources.
  if (FU->getResType() == HWResType::Trivial)
    return true;

  unsigned ModuloStep = step % getMII();
  // Do all resource at step been reserve?
  if (MRT[FU][ModuloStep] >= FU->getTotalFUs())
    return false;

  if (take) ++MRT[FU][ModuloStep];
  
  return true;
}


bool IMS::isAllAtomScheduled() {
  for (FSMState::iterator I = State->begin(), E = State->end();
      I != E; ++I) {
    HWAtom *A = *I;
    if (!A->isScheduled())
      return false;
  }

  return true;
}
