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
void SchedulingBase::buildTimeFrame(const VSUnit *ClampedSUnit,
                                                 unsigned ClampedASAP,
                                                 unsigned ClampedALAP) {

  assert((ClampedSUnit == 0 
          || (ClampedASAP >= getSTFASAP(ClampedSUnit)
              && ClampedALAP <= getSTFALAP(ClampedSUnit)))
         && "Bad clamped value!");
  VSUnit *EntryRoot = State.getEntryRoot();
  // Build the time frame
  assert(EntryRoot->isScheduled() && "Entry must be scheduled first!");
  buildASAPStep(ClampedSUnit, ClampedASAP);
  buildALAPStep(ClampedSUnit, ClampedALAP);

  DEBUG(dumpTimeFrame());
}

void SchedulingBase::buildASAPStep(const VSUnit *ClampedSUnit, unsigned ClampedASAP) {
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

      unsigned NewStep = A == ClampedSUnit ? ClampedASAP : getSTFDetailASAP(A);
      for (VSUnit::dep_iterator DI = A->dep_begin(), DE = A->dep_end();
          DI != DE; ++DI) {
        const VSUnit *Dep = *DI;
        
        if (!DI.getEdge()->isLoopCarried() || MII) {
          unsigned DepASAP = Dep->isScheduled() ?
                             Dep->getDetailSlot() : getASAPDetailStep(Dep);
          int Step = DepASAP + (DI.getEdge()->getLatency()
                     - MII * DI.getEdge()->getItDst()) * 2;
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
      if (SUnitToTF[A->getIdx()].first != OpSlot(NewStep/2, !A->hasDatapath())) {
        SUnitToTF[A->getIdx()].first =OpSlot(NewStep/2, !A->hasDatapath());
        changed |= true;
      }
    }
  } while (changed);

  VSUnit *Exit = State.getExitRoot();
  CriticalPathEnd = std::max(CriticalPathEnd, getASAPStep(Exit));
}

void SchedulingBase::buildALAPStep(const VSUnit *ClampedSUnit,
                                                unsigned ClampedALAP) {
  VSUnit *Exit = State.getExitRoot();
  int LastSlot = Exit == ClampedSUnit ? ClampedALAP : CriticalPathEnd;
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
      unsigned NewStep = A == ClampedSUnit ? ClampedALAP : getSTFDetailALAP(A);
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
          
        unsigned Step = UseALAP - (UseEdge->getLatency()
                          - MII * UseEdge->getItDst()) * 2;
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
      if (SUnitToTF[A->getIdx()].second != OpSlot(NewStep/2, !A->hasDatapath())) {
        SUnitToTF[A->getIdx()].second = OpSlot(NewStep/2, !A->hasDatapath());
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

void SchedulingBase::buildDGraph() {
  DGraph.clear();
  for (VSchedGraph::iterator I = State.begin(), E = State.end(); I != E; ++I) {
    VSUnit *A = *I;
    // We only try to balance the post bind resource.
    // if (A->getFUId().isBinded()) continue;
    // Ignore the DG for trivial resources.
    if (A->getFUId().isTrivial()) continue;
    
    unsigned TimeFrame = getTimeFrame(A);
    unsigned ASAPStep = getASAPStep(A), ALAPStep = getALAPStep(A);

    double Prob = 1.0 / (double) TimeFrame;
    // Including ALAPStep.
    for (unsigned i = ASAPStep, e = ALAPStep + 1; i != e; ++i)
      accDGraphAt(i, A->getFUId(), Prob);    
  }
  DEBUG(printDG(dbgs()));
}


bool SchedulingBase::isResourceConstraintPreserved() {
  ExtraResReq = 0.0;
  // No resource in use.
  if (DGraph.empty()) return true;

  // For each function unit.
  for (DGType::const_iterator I = DGraph.begin(), E = DGraph.end();
       I != E; ++I) {
    FuncUnitId ID = I->first;
    DEBUG(dbgs() << "FU " << ID << " Average Usage: ");
    double TotalDG = 0;
    unsigned AvailableSteps = 0;
    for (DGStepMapType::const_iterator SI = I->second.begin(),
         SE = I->second.end(); SI != SE; ++SI) {
      ++AvailableSteps;
      TotalDG += SI->second;
    }
    double AverageDG = TotalDG / AvailableSteps;
    // The upper bound of error: Only 0.5 extra functional unit need.
    double e = 0.5 / AvailableSteps;
    DEBUG(dbgs() << AverageDG << '\n');
    
    // Accumulate the extra require function unit amount.
    if (AverageDG > ID.getTotalFUs() + e)
      ExtraResReq += (AverageDG - ID.getTotalFUs()) /  ID.getTotalFUs();
  }
  return ExtraResReq == 0.0;
}

void SchedulingBase::printDG(raw_ostream &OS) const {  
  // For each step
  // For each FU.
  for (DGType::const_iterator I = DGraph.begin(), E = DGraph.end();
       I != E; ++I) {
    OS << "FU " << I->first << ":\n";
    for (DGStepMapType::const_iterator SI = I->second.begin(),
         SE = I->second.end(); SI != SE; ++SI) {
      OS << "@ " << SI->first << ": " << SI->second << '\n';
    }
    OS << '\n';
  }
}

double SchedulingBase::getDGraphAt(unsigned step, FuncUnitId FUClass) const {
  // Modulo DG for modulo schedule.
  DGType::const_iterator at = DGraph.find(FUClass);
  if (at != DGraph.end()) {
    DGStepMapType::const_iterator SI = at->second.find(computeStepKey(step));
    if (SI != at->second.end())
      return SI->second;
  }

  return 0.0;
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

void SchedulingBase::accDGraphAt(unsigned step, FuncUnitId FUClass, double d) {
  // Modulo DG for modulo schedule.
  DGraph[FUClass][computeStepKey(step)] += d;
}

// Including end.
double SchedulingBase::getRangeDG(FuncUnitId FUClass, unsigned start, unsigned end) {
  double range = end - start + 1;
  double ret = 0.0;
  for (unsigned i = start, e = end + 1; i != e; ++i)
    ret += getDGraphAt(i, FUClass);

  ret /= range;
  return ret;
}

double SchedulingBase::computeForce(const VSUnit *A, unsigned ASAP, unsigned ALAP) {
  buildTimeFrame(A, ASAP, ALAP);
  buildDGraph();
  // Compute the forces.
  double SelfForce = computeSelfForce(A, ASAP, ALAP);
  // The follow function will invalid the time frame.
  DEBUG(dbgs() << " Self Force: " << SelfForce);
  double OtherForce = computeOtherForce(A);
  DEBUG(dbgs() << " Other Force: " << OtherForce);
  double Force = SelfForce + OtherForce;
  DEBUG(dbgs() << " Force: " << Force);
  return Force;
}

double SchedulingBase::computeSelfForce(const VSUnit *A,
                                 unsigned start,
                                 unsigned end) {
  FuncUnitId FU = A->getFUId();

  // The trivial function unit do not contribute any force.
  if (FU.isTrivial()) return 0.0;
  
  // FIXME: How should handle the pre-bind MachineInstruction.
  // if (NoFDSchedule && !A->isBinded() && MII) return 0.0;

  double Force = getRangeDG(FU, start, end) - getAvgDG(A);

  // FIXME: Make the atoms taking expensive function unit have bigger force.
  return Force / FU.getTotalFUs();
}

double SchedulingBase::computeRangeForce(const VSUnit *A,
                                  unsigned int start,
                                  unsigned int end) {
  // FIXME: How should handle the pre-bind MachineInstruction.
  // if (NoFDSchedule && !A->isBinded() && MII) return 0.0;

  FuncUnitId FU = A->getFUId();

  // The trivial function unit do not contribute any force.
  if (FU.isTrivial()) return 0.0;

  double Force = getRangeDG(FU, start, end) - getAvgDG(A);
  // FIXME: Make the atoms taking expensive function unit have bigger force.
  return Force / FU.getTotalFUs();
}

double SchedulingBase::computeOtherForce(const VSUnit *A) {
  double ret = 0.0;

  for (VSchedGraph::iterator I = State.begin(), E = State.end(); I != E; ++I) {
    if (A == *I) continue;

    // The trivial function unit do not contribute any force.
    if (A->getFUId().isTrivial()) continue;

    ret += computeRangeForce(A, getASAPStep(A), getALAPStep(A));
  }
  return ret;
}

void SchedulingBase::buildAvgDG() {
  for (VSchedGraph::iterator I = State.begin(), E = State.end();
       I != E; ++I) { 
    VSUnit *A = *I;
    // We only care about the no trivial resource.
    if (A->getFUId().isTrivial()) continue;
    // We only care about the utilization of post bind resource.
    // if (A->getFUId().isBinded()) continue;
    
    double res = 0.0;
    for (unsigned i = getASAPStep(A), e = getALAPStep(A) + 1; i != e; ++i)
      res += getDGraphAt(i, A->getFUId());

    res /= (double) getTimeFrame(A);
    AvgDG[A->getIdx()] = res;
  }
}

unsigned SchedulingBase::buildFDepHD(bool rstSTF) {
  if (rstSTF) {
    resetSTF();
    State.resetSchedule();
  }
  buildTimeFrame();

  // FIXME: This is not necessary in ILP scheduler after we can compute ASAP and
  // ALAP with function unit information.
  buildDGraph();
  buildAvgDG();

  return CriticalPathEnd;
}

void SchedulingBase::dumpDG() const {
  printDG(dbgs());
}

void SchedulingBase::resetSTF() {
  for (VSchedGraph::iterator I = State.begin(), E = State.end();
       I != E; ++I) {
    VSUnit *SU = *I;
    SUnitToSTF[SU->getIdx()] = std::make_pair(OpSlot(0,true),
                                              OpSlot(VSUnit::MaxSlot,true));
  }
}

void SchedulingBase::sinkSTF(const VSUnit *A, unsigned ASAP, unsigned ALAP) {
  assert(ASAP <= ALAP && "Bad time frame to sink!");
  assert(ASAP >= getSTFASAP(A) && ALAP <= getSTFALAP(A) && "Can not Sink!");
  SUnitToSTF[A->getIdx()] = std::make_pair(OpSlot(ASAP, !A->hasDatapath()),
                                           OpSlot(ALAP, !A->hasDatapath()));
  // We may need to reduce critical path.
  if (A == State.getExitRoot()) {
    assert(CriticalPathEnd >= ALAP && "Can not sink ExitRoot!");
    CriticalPathEnd = ALAP;
  }
}

void SchedulingBase::updateSTF() {
  for (VSchedGraph::iterator I = State.begin(), E = State.end();
      I != E; ++I) {
    VSUnit *A = *I;
    // Only update the scheduled time frame.
    if (!isSTFScheduled(A))
      continue;

    sinkSTF(A, getASAPStep(A), getALAPStep(A));
  }
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
      updateSTF();
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
