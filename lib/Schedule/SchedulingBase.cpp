//===- ForceDirectedSchedulingBase.cpp - ForceDirected information analyze --*- C++ -*-===//
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
// This file implement the Force Direct information computation pass describe in
// Force-Directed Scheduling for the Behavioral Synthesis of ASIC's
//
//===----------------------------------------------------------------------===//

#include "SchedulingBase.h"
#include "ScheduleDOT.h"
#include "vtm/Passes.h"

#include "llvm/Support/CommandLine.h"

#define DEBUG_TYPE "vbe-fd-info"
#include "llvm/Support/Debug.h"

using namespace llvm;
using namespace llvm;

//===----------------------------------------------------------------------===//
void SchedulingBase::buildTimeFrame() {
  VSUnit *EntryRoot = State.getEntryRoot();
  assert(EntryRoot->isScheduled() && "Entry must be scheduled first!");

  // Reset the time frames
  typedef std::map<const VSUnit*, TimeFrame> TFMapTy;
  for (TFMapTy::iterator I = SUnitToTF.begin(), E = SUnitToTF.end();I != E;++I)
    I->second = std::make_pair(0, VSUnit::MaxSlot);

  // Build the time frames
  buildASAPStep();
  buildALAPStep();

  DEBUG(dumpTimeFrame());
}

void SchedulingBase::buildASAPStep() {
  VSUnit *Entry = State.getEntryRoot();
  SUnitToTF[Entry].first = Entry->getSlot();
  typedef VSchedGraph::sched_iterator it;
  it Start = State.sched_begin();

  bool changed = false;

  // Build the time frame iteratively.
  do {
    changed = false;
    for (it I = Start + 1, E = State.sched_end(); I != E; ++I) {
      VSUnit *A = *I;
      if (A->isScheduled()) {
        SUnitToTF[A].first = A->getSlot();
        continue;
      }

      DEBUG(dbgs() << "\n\nCalculating ASAP step for \n";
            A->dump(););

      unsigned NewStep = 0;
      for (VSUnit::dep_iterator DI = A->dep_begin(), DE = A->dep_end();
          DI != DE; ++DI) {
        const VSUnit *Dep = *DI;
        if (!DI.getEdge()->isLoopCarried() || MII) {
          unsigned DepASAP = Dep->isScheduled() ?
                             Dep->getSlot() : getASAPStep(Dep);
          int Step = DepASAP + DI.getEdge()->getLatency()
                     - (MII * DI.getEdge()->getItDst());
          DEBUG(dbgs() << "From ";
                if (DI.getEdge()->isLoopCarried())
                  dbgs() << "BackEdge ";
                Dep->print(dbgs());
                dbgs() << " Step " << Step << '\n');
          unsigned UStep = std::max(0, Step);
          NewStep = std::max(UStep, NewStep);
        }
      }

      DEBUG(dbgs() << "Update ASAP step to: " << NewStep << " for \n";
      A->dump();
      dbgs() << "\n\n";);

      unsigned &ASAPStep = SUnitToTF[A].first;
      if (ASAPStep != NewStep) {
        ASAPStep = NewStep;
        changed |= true;
      }
    }
  } while (changed);

  VSUnit *Exit = State.getExitRoot();
  CriticalPathEnd = std::max(CriticalPathEnd, getASAPStep(Exit));
}

void SchedulingBase::buildALAPStep() {
  VSUnit *Exit = State.getExitRoot();
  int LastSlot = CriticalPathEnd;
  SUnitToTF[Exit].second = LastSlot;

  bool changed = false;
  // Build the time frame iteratively.
  do {
    changed = false;
    for (int Idx = State.num_scheds()/*skip exitroot*/- 2; Idx >= 0; --Idx){
      VSUnit *A = State.getCtrlAt(Idx);
      if (A->isScheduled()) {
        SUnitToTF[A].second = A->getSlot();
        continue;
      }

      DEBUG(dbgs() << "\n\nCalculating ALAP step for \n";
            A->dump(););
      unsigned NewStep = VSUnit::MaxSlot;
      for (VSUnit::use_iterator UI = A->use_begin(), UE = A->use_end();
           UI != UE; ++UI) {
        const VSUnit *Use = *UI;
        VDEdge *UseEdge = Use->getEdgeFrom(A);

        if (!UseEdge->isLoopCarried() || MII) {
          unsigned UseALAP = Use->isScheduled() ?
                             Use->getSlot() : getALAPStep(Use);
          if (UseALAP == 0) {
            assert(UseEdge->isLoopCarried() && "Broken time frame!");
            UseALAP = VSUnit::MaxSlot;
          }
          unsigned Step = UseALAP - UseEdge->getLatency()
                          + (MII * UseEdge->getItDst());
          DEBUG(dbgs() << "From ";
                if (UseEdge->isLoopCarried())
                  dbgs() << "BackEdge ";
                Use->print(dbgs());
                dbgs() << " Step " << Step << '\n');
          NewStep = std::min(Step, NewStep);
        }
      }

      DEBUG(dbgs() << "Update ALAP step to: " << NewStep << " for \n";
            A->dump();
            dbgs() << "\n\n";);

      unsigned &ALAPStep = SUnitToTF[A].second;
      if (ALAPStep != NewStep) {
        assert(getASAPStep(A) <= NewStep && "Broken ALAP step!");
        ALAPStep = NewStep;
        changed = true;
      }
    }
  } while (changed);
}

void SchedulingBase::printTimeFrame(raw_ostream &OS) const {
  OS << "Time frame:\n";
  typedef VSchedGraph::sched_iterator it;
  for (it I = State.sched_begin(), E = State.sched_end(); I != E; ++I) {
    VSUnit *A = *I;
    A->print(OS);
    OS << " : {" << getASAPStep(A) << "," << getALAPStep(A)
      << "} " <<  getTimeFrame(A);

    for (VSUnit::dep_iterator DI = A->dep_begin(), DE = A->dep_end(); DI != DE;
        ++DI)
      OS << " [" << DI->getIdx() << "]"; 
    
    OS << '\n';
  }
}

unsigned SchedulingBase::computeResMII() {
  // FIXME: Compute the resource area cost
  std::map<FuncUnitId, unsigned> TotalResUsage;
  typedef VSchedGraph::sched_iterator it;
  for (it I = State.sched_begin(), E = State.sched_end(); I != E; ++I) {
    VSUnit *SU = *I;
    if (!SU->getFUId().isBound()) continue;

    ++TotalResUsage[SU->getFUId()];
  }

  unsigned MaxResII = 0;
  typedef std::map<FuncUnitId, unsigned>::iterator UsageIt;
  for (UsageIt I = TotalResUsage.begin(), E = TotalResUsage.end(); I != E; ++I){
    /*There is only 1 resource avaialbe for Prebound function unit kind*/
    const unsigned NumFUs = 1;
    MaxResII = std::max(MaxResII, I->second / NumFUs);
  }
  DEBUG(dbgs() << "ResMII: " << MaxResII << '\n');
  return MaxResII;
}

bool SchedulingBase::computeMII() {
  unsigned RecMII = computeRecMII();
  if (RecMII == 0) {
    MII = this->getCriticalPathLength();
    return false;
  }

  unsigned ResMII = computeResMII();
  MII = std::max(RecMII, ResMII);
  // Also adjust the critical path length.
  setCriticalPathLength(std::max(MII, getCriticalPathLength()));
  return true;
}

void SchedulingBase::dumpTimeFrame() const {
  printTimeFrame(dbgs());
}

bool SchedulingBase::tryTakeResAtStep(VSUnit *U, unsigned step) {
  FuncUnitId FU = U->getFUId();
  // We will always have enough trivial resources.
  if (FU.isTrivial()) return true;

  unsigned Latency = U->getLatency();

  // FIXME: Compute the area cost.
  if (FU.isBound()) {
    // Do all resource at step been reserve?
    for (unsigned i = step, e = step + Latency; i != e; ++i) {
      unsigned s = computeStepKey(i);
      /*There is only 1 resource avaialbe for Prebound function unit kind*/
      const unsigned NumFUs = 1;
      if (RT[FU][s] >= NumFUs)
        return false;
    }

    for (unsigned i = step, e = step + Latency; i != e; ++i) {
      unsigned s = computeStepKey(i);
      ++RT[FU][s];
    }
  }

  return true;
}

void SchedulingBase::scheduleSU(VSUnit *U, unsigned step) {
  U->scheduledTo(step);

  FuncUnitId FU = U->getFUId();
  // We will always have enough trivial resources.
  if (FU.isTrivial()) return;

  unsigned Latency = U->getLatency();
  for (unsigned i = step, e = step + Latency; i != e; ++i) {
    unsigned s = computeStepKey(i);
    ++RT[FU][s];
  }
}

void SchedulingBase::unscheduleSU(VSUnit *U) {
  unsigned step = U->getSlot();
  U->resetSchedule();

  FuncUnitId FU = U->getFUId();
  // We will always have enough trivial resources.
  if (FU.isTrivial()) return;

  unsigned Latency = U->getLatency();

  for (unsigned i = step, e = step + Latency; i != e; ++i) {
    unsigned s = computeStepKey(i);
    --RT[FU][s];
  }
}

bool SchedulingBase::isResourceConstraintPreserved() {
  ExtraResReq = 0.0;
  resetRT();

  typedef VSchedGraph::sched_iterator it;
  for (it I = State.sched_begin(), E = State.sched_end(); I != E; ++I) {
    VSUnit *A = *I;
    FuncUnitId FU = A->getFUId();
    // We only try to balance the post bind resource.
    // if (A->getFUId().isBinded()) continue;
    // Ignore the DG for trivial resources.
    if (!FU.isBound()) continue;

    bool available = false;

    // Check ifwe have enough function unit by try to fit them in the resource
    // table including ALAPStep.
    for (unsigned i = getASAPStep(A), e = getALAPStep(A) + 1; i != e; ++i) {
      if (tryTakeResAtStep(A, i)) {
        available = true;
        break;;
      }
    }

    if (!available)
      ExtraResReq += 1.0;
  }

  return ExtraResReq == 0.0;
}

unsigned SchedulingBase::computeStepKey(unsigned step) const {
  if (MII != 0) {
#ifndef NDEBUG
    unsigned StartSlot = State.getStartSlot();
    step = StartSlot + (step - StartSlot) % MII;
#else
    step = step % MII;
#endif
  }

  return step;
}

unsigned SchedulingBase::buildFDepHD(bool rstSTF) {
  if (rstSTF) State.resetSchedule(getMII());

  buildTimeFrame();

  return CriticalPathEnd;
}

//===----------------------------------------------------------------------===//
void SchedulingBase::schedulePassiveSUnits() {
  typedef VSchedGraph::sched_iterator it;
  for (it I = State.sched_begin(), E = State.sched_end(); I != E; ++I) {
    VSUnit *A = *I;
    if (A->isScheduled())
      continue;

    assert(A->getFUId().isTrivial()
      && "SUnit that taking non-trivial not scheduled?");

    DEBUG(A->print(dbgs()));
    unsigned step = getASAPStep(A);
    A->scheduledTo(step);
    buildFDepHD(false);
  }
}

bool SchedulingBase::allNodesSchedued() const {
  typedef VSchedGraph::sched_iterator it;
  for (it I = State.sched_begin(), E = State.sched_end(); I != E; ++I) {
    VSUnit *A = *I;
    if (!A->isScheduled()) return false;
  }

  return true;
}

bool SchedulingBase::scheduleCriticalPath(bool refreshFDepHD) {
  if (refreshFDepHD)
    buildFDepHD(true);

  typedef VSchedGraph::sched_iterator it;
  for (it I = State.sched_begin(), E = State.sched_end(); I != E; ++I) {
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

void SchedulingBase::viewGraph() {
  ViewGraph(this, State.getMachineBasicBlock()->getName());
}
