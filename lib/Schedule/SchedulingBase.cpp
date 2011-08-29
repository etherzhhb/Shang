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

static cl::opt<bool>
NoFDSchedule("disable-fd-schedule",
             cl::desc("vbe - Do not preform force-directed schedule"),
             cl::Hidden, cl::init(false));

//===----------------------------------------------------------------------===//
void SchedulingBase::buildTimeFrame() {
  VSUnit *EntryRoot = State.getEntryRoot();
  // Build the time frame
  assert(EntryRoot->isScheduled() && "Entry must be scheduled first!");
  buildASAPStep();
  buildALAPStep();

  DEBUG(dumpTimeFrame());
}

void SchedulingBase::buildASAPStep() {
  VSUnit *Entry = State.getEntryRoot();
  SUnitToTF[Entry->getIdx()].first = OpSlot(Entry->getSlot(), !Entry->hasDatapath());

  VSchedGraph::iterator Start = State.begin();

  bool changed = false;

  // Build the time frame iteratively.
  do {
    changed = false;
    for (VSchedGraph::iterator I = Start + 1, E = State.end(); I != E; ++I) {
      VSUnit *A = *I;
      if (A->isScheduled()) {
        SUnitToTF[A->getIdx()].first =OpSlot(A->getSlot(), !A->hasDatapath()) ;
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
                             Dep->getDetailSlot() : getASAPDetailStep(Dep);
          int Step = DepASAP + DI.getEdge()->getDetailLatency()
                     - (MII * DI.getEdge()->getItDst()) * 2;
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

      OpSlot NewSlot = OpSlot::detailSlotCeil(NewStep, A->hasDatapath());
      if (SUnitToTF[A->getIdx()].first != NewSlot) {
        SUnitToTF[A->getIdx()].first = NewSlot;
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
  SUnitToTF[Exit->getIdx()].second = OpSlot(LastSlot , !Exit->hasDatapath());

  VSchedGraph::reverse_iterator Start = State.rbegin();

  bool changed = false;
  // Build the time frame iteratively.
  do {
    changed = false;
    for (VSchedGraph::reverse_iterator I = Start + 1, E = State.rend();
         I != E; ++I) {
      VSUnit *A = *I;
      if (A->isScheduled()) {
        SUnitToTF[A->getIdx()].second = OpSlot(A->getSlot(), !A->hasDatapath());
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
                             Use->getDetailSlot() : getALAPDetailStep(Use);
          if (UseALAP == 0) {
            assert(UseEdge->isLoopCarried() && "Broken time frame!");
            UseALAP = VSUnit::MaxSlot;
          }
          unsigned Step = UseALAP - UseEdge->getDetailLatency()
                          + (MII * UseEdge->getItDst()) * 2;
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

      OpSlot NewSlot = OpSlot::detailSlotFloor(NewStep, A->hasDatapath());
      if (SUnitToTF[A->getIdx()].second != NewSlot) {
        SUnitToTF[A->getIdx()].second = NewSlot;
        changed = true;
      }
    }
  } while (changed);

#ifndef NDEBUG
  // Verify the time frames.
  for (VSchedGraph::iterator I = State.begin(), E = State.end(); I != E; ++I) {
    VSUnit *A = *I;
    assert(getALAPStep(A) >= getASAPStep(A)  && "Broken time frame!");
  }
#endif
}

void SchedulingBase::printTimeFrame(raw_ostream &OS) const {
  OS << "Time frame:\n";
  for (VSchedGraph::iterator I = State.begin(), E = State.end();
      I != E; ++I) {
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

void SchedulingBase::dumpTimeFrame() const {
  printTimeFrame(dbgs());
}

bool SchedulingBase::tryTakeResAtStep(VSUnit *U, unsigned step) {
  FuncUnitId FU = U->getFUId();
  // We will always have enough trivial resources.
  if (FU.isTrivial()) return true;

  unsigned Latency = U->getLatency();
  
  // Do all resource at step been reserve?
  for (unsigned i = step, e = step + Latency; i != e; ++i) {
    unsigned s = computeStepKey(i);
    if (RT[FU][s] >= FU.getTotalFUs())
      return false;
  }

  for (unsigned i = step, e = step + Latency; i != e; ++i) {
    unsigned s = computeStepKey(i);
    ++RT[FU][s];
  }

  return true;
}

bool SchedulingBase::isResourceConstraintPreserved() {
  ExtraResReq = 0.0;
  resetRT();

  for (VSchedGraph::iterator I = State.begin(), E = State.end(); I != E; ++I) {
    VSUnit *A = *I;
    FuncUnitId FU = A->getFUId();
    // We only try to balance the post bind resource.
    // if (A->getFUId().isBinded()) continue;
    // Ignore the DG for trivial resources.
    if (FU.isTrivial()) continue;

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
      ExtraResReq += 1.0 /  FU.getTotalFUs();
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
  if (rstSTF) {
    State.resetSchedule();
  }

  buildTimeFrame();

  return CriticalPathEnd;
}

//===----------------------------------------------------------------------===//
void SchedulingBase::schedulePassiveSUnits() {
  for (VSchedGraph::iterator I = State.begin(), E = State.end();
       I != E; ++I) {
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

bool SchedulingBase::scheduleCriticalPath(bool refreshFDepHD) {
  if (refreshFDepHD)
    buildFDepHD(true);

  for (VSchedGraph::iterator I = State.begin(), E = State.end();
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

void SchedulingBase::viewGraph() {
  ViewGraph(this, State.getMachineBasicBlock()->getName());
}
