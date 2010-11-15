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

#include "ForceDirectedScheduling.h"
#include "HWAtomPasses.h"

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
void ForceDirectedSchedulingBase::buildTimeFrame(const HWAtom *ClampedAtom,
                                                 unsigned ClampedASAP,
                                                 unsigned ClampedALAP) {

  assert((ClampedAtom == 0 
          || (ClampedASAP >= getSTFASAP(ClampedAtom)
              && ClampedALAP <= getSTFALAP(ClampedAtom)))
         && "Bad clamped value!");
  AtomToTF.clear();
  HWAtom *EntryRoot = State->getEntryRoot();
  // Build the time frame
  assert(EntryRoot->isScheduled() && "Entry must be scheduled first!");
  buildASAPStep(ClampedAtom, ClampedASAP);
  buildALAPStep(ClampedAtom, ClampedALAP);

  DEBUG(dumpTimeFrame());
}

void ForceDirectedSchedulingBase::buildASAPStep(const HWAtom *ClampedAtom,
                                                unsigned ClampedASAP) {
  HWAtom *Entry = State->getEntryRoot();
  AtomToTF[Entry].first = Entry->getSlot();

  FSMState::iterator Start = State->begin();

  bool changed = false;

  // Build the time frame iteratively.
  do {
    changed = false;
    for (FSMState::iterator I = ++Start, E = State->end(); I != E; ++I) {
      HWAtom *A = *I;
      if (A->isScheduled()) {
        AtomToTF[A].first = A->getSlot();
        continue;
      }

      DEBUG(dbgs() << "\n\nCalculating ASAP step for \n";
            A->dump(););

      unsigned NewStep = A == ClampedAtom ? ClampedASAP : getSTFASAP(A);

      for (HWAtom::dep_iterator DI = A->dep_begin(), DE = A->dep_end();
          DI != DE; ++DI) {
        const HWAtom *Dep = *DI;
        
        if (!DI.getEdge()->isBackEdge() || MII) {
          unsigned DepASAP = Dep->isScheduled() ?
                             Dep->getSlot() : getASAPStep(Dep);
          int Step = DepASAP + DI.getEdge()->getLatency()
                     - MII * DI.getEdge()->getItDst();
          DEBUG(dbgs() << "From ";
                if (DI.getEdge()->isBackEdge())
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
      if (AtomToTF[A].first != NewStep) {
        AtomToTF[A].first = NewStep;
        changed |= true;
      }
    }
  } while (changed);

  HWAtom *Exit = State->getExitRoot();
  CriticalPathEnd = std::max(CriticalPathEnd, getASAPStep(Exit));
}

void ForceDirectedSchedulingBase::buildALAPStep(const HWAtom *ClampedAtom,
                                                unsigned ClampedALAP) {
  HWAtom *Exit = State->getExitRoot();
  AtomToTF[Exit].second = Exit == ClampedAtom ? ClampedALAP : CriticalPathEnd;

  FSMState::reverse_iterator Start = State->rbegin();

  bool changed = false;

  // Build the time frame iteratively.
  do {
    changed = false;
    for (FSMState::reverse_iterator I = ++Start, E = State->rend();
         I != E; ++I) {
      HWAtom *A = *I;
      if (A->isScheduled()) {
        AtomToTF[A].second = A->getSlot();
        continue;
      }

      DEBUG(dbgs() << "\n\nCalculating ALAP step for \n";
            A->dump(););

      unsigned NewStep = A == ClampedAtom ? ClampedALAP : getSTFALAP(A);
      for (HWAtom::use_iterator UI = A->use_begin(), UE = A->use_end();
           UI != UE; ++UI) {
        const HWAtom *Use = *UI;
        HWEdge *UseEdge = Use->getEdgeFrom(A);

        if (!UseEdge->isBackEdge() || MII) {
          unsigned UseALAP = Use->isScheduled() ?
                             Use->getSlot() : getALAPStep(Use);
          if (UseALAP == 0) {
            assert(UseEdge->isBackEdge() && "Broken time frame!");
            UseALAP = HWAtom::MaxSlot;
          }
          
          unsigned Step = UseALAP - UseEdge->getLatency()
                          + MII * UseEdge->getItDst();
          DEBUG(dbgs() << "From ";
                if (UseEdge->isBackEdge())
                  dbgs() << "BackEdge ";
                Use->print(dbgs());
                dbgs() << " Step " << Step << '\n');
          NewStep = std::min(Step, NewStep);
        }
      }

      DEBUG(dbgs() << "Update ALAP step to: " << NewStep << " for \n";
            A->dump();
            dbgs() << "\n\n";);
      if (AtomToTF[A].second != NewStep) {
        AtomToTF[A].second = NewStep;
        changed = true;
      }
    }
  } while (changed);

#ifndef NDEBUG
  // Verify the time frames.
  for (FSMState::iterator I = State->begin(), E = State->end(); I != E; ++I) {
    HWAtom *A = *I;
    assert(getALAPStep(A) >= getASAPStep(A)  && "Broken time frame!");
  }
#endif
}

void ForceDirectedSchedulingBase::printTimeFrame(raw_ostream &OS) const {
  OS << "Time frame:\n";
  for (FSMState::iterator I = State->begin(), E = State->end();
      I != E; ++I) {
    HWAtom *A = *I;
    A->print(OS);
    OS << " : {" << getASAPStep(A) << "," << getALAPStep(A)
      << "} " <<  getTimeFrame(A);

    for (HWAtom::dep_iterator DI = A->dep_begin(), DE = A->dep_end(); DI != DE;
        ++DI)
      OS << " [" << DI->getIdx() << "]"; 
    
    OS << '\n';
  }
}

void ForceDirectedSchedulingBase::dumpTimeFrame() const {
  printTimeFrame(dbgs());
}

void ForceDirectedSchedulingBase::buildDGraph() {
  DGraph.clear();
  for (FSMState::iterator I = State->begin(), E = State->end(); I != E; ++I){
    // We only try to balance the post bind resource.
    // Ignore the DG for trivial resources.
    // if (A->isTrivial()) continue;
    HWAtom *A = *I;
    unsigned TimeFrame = getTimeFrame(A);
    unsigned ASAPStep = getASAPStep(A), ALAPStep = getALAPStep(A);

    double Prob = 1.0 / (double) TimeFrame;
    // Including ALAPStep.
    for (unsigned i = ASAPStep, e = ALAPStep + 1; i != e; ++i)
      accDGraphAt(i, A->getFUType(), Prob);    
  }
  DEBUG(printDG(dbgs()));
}


bool ForceDirectedSchedulingBase::isResourceConstraintPreserved() {
  ExtraResReq = 0.0;
  // No resource in use.
  if (DGraph.empty()) return true;

  for (DGType::const_iterator I = DGraph.begin(), E = DGraph.end();
       I != E; ++I) {
    // FIXME: print out the Resource class name.
    DEBUG(dbgs() << "FU " << I->first << " Average Usage: ");
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
    // FIXME: Get the totalFU of a specific resource class.
    //if (AverageDG > I->first->getTotalFUs() + e)
    //  ExtraResReq += (AverageDG - I->first->getTotalFUs())
    //                  /  I->first->getTotalFUs();
  }
  return ExtraResReq == 0.0;
}

void ForceDirectedSchedulingBase::printDG(raw_ostream &OS) const {  
  // For each step
  // For each FU.
  for (DGType::const_iterator I = DGraph.begin(), E = DGraph.end();
       I != E; ++I) {
    // FIXME: print out the Resource class name.
    OS << "FU " << I->first << ":\n";
    for (DGStepMapType::const_iterator SI = I->second.begin(),
         SE = I->second.end(); SI != SE; ++SI) {
      OS << "@ " << SI->first << ": " << SI->second << '\n';
    }
    OS << '\n';
  }
}

double ForceDirectedSchedulingBase::getDGraphAt(unsigned step,
                                                unsigned FUClass) const {
  // Modulo DG for modulo schedule.
  DGType::const_iterator at = DGraph.find(FUClass);
  if (at != DGraph.end()) {
    DGStepMapType::const_iterator SI = at->second.find(computeStepKey(step));
    if (SI != at->second.end())
      return SI->second;
  }

  return 0.0;
}

unsigned ForceDirectedSchedulingBase::computeStepKey(unsigned step) const {
  if (MII != 0) {
#ifndef NDEBUG
    unsigned StartSlot = State->getStartSlot();
    step = StartSlot + (step - StartSlot) % MII;
#else
    step = step % MII;
#endif
  }

  return step;
}

void ForceDirectedSchedulingBase::accDGraphAt(unsigned step, unsigned FUClass,
                                              double d) {
  // Modulo DG for modulo schedule.
  DGraph[FUClass][computeStepKey(step)] += d;
}

// Including end.
double ForceDirectedSchedulingBase::getRangeDG(unsigned FUClass,
                                               unsigned start, unsigned end) {
  double range = end - start + 1;
  double ret = 0.0;
  for (unsigned i = start, e = end + 1; i != e; ++i)
    ret += getDGraphAt(i, FUClass);

  ret /= range;
  return ret;
}

double ForceDirectedSchedulingBase::computeForce(const HWAtom *A,
                                                 unsigned ASAP, unsigned ALAP) {
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

double ForceDirectedSchedulingBase::computeSelfForce(const HWAtom *A,
                                                     unsigned start,
                                                     unsigned end) {
  // FIXME: How should handle the pre-bind MachineInstruction.
  // if (NoFDSchedule && !A->isBinded() && MII) return 0.0;

  unsigned FUClass = A->getFUType();
  double Force = getRangeDG(FUClass, start, end) - getAvgDG(A);

  // FIXME: Make the atoms taking expensive function unit have bigger force.
  return Force; // / FU->getTotalFUs();
}

double ForceDirectedSchedulingBase::computeRangeForce(const HWAtom *A,
                                                      unsigned int start,
                                                      unsigned int end) {
  // FIXME: How should handle the pre-bind MachineInstruction.
  // if (NoFDSchedule && !A->isBinded() && MII) return 0.0;

  unsigned FUClass = A->getFUType();
  double Force = getRangeDG(FUClass, start, end) - getAvgDG(A);
  // FIXME: Make the atoms taking expensive function unit have bigger force.
  return Force; // / FU->getTotalFUs();
}

double ForceDirectedSchedulingBase::computeOtherForce(const HWAtom *A) {
  double ret = 0.0;

  for (FSMState::iterator I = State->begin(), E = State->end(); I != E; ++I) {
    if (A == *I) continue;
    
    ret += computeRangeForce(A, getASAPStep(A), getALAPStep(A));
  }
  return ret;
}

void ForceDirectedSchedulingBase::buildAvgDG() {
  AvgDG.clear();
  for (FSMState::iterator I = State->begin(), E = State->end();
       I != E; ++I) {
    // We only care about the utilization of post bind resource. 
    HWAtom *A = *I;
    double res = 0.0;
    for (unsigned i = getASAPStep(A), e = getALAPStep(A) + 1; i != e; ++i)
      res += getDGraphAt(i, A->getFUType());

    res /= (double) getTimeFrame(A);
    AvgDG[A] = res;
  }
}

unsigned ForceDirectedSchedulingBase::buildFDInfo(bool rstSTF) {
  if (rstSTF) {
    resetSTF();
    State->resetSchedule();
  }
  buildTimeFrame();

  buildDGraph();
  buildAvgDG();

  return CriticalPathEnd;
}

void ForceDirectedSchedulingBase::dumpDG() const {
  printDG(dbgs());
}

void ForceDirectedSchedulingBase::resetSTF() {
  AtomToSTF.clear();
  for (FSMState::iterator I = State->begin(), E = State->end();
       I != E; ++I)
    AtomToSTF.insert(std::make_pair(*I, std::make_pair(0, HWAtom::MaxSlot)));
}

void ForceDirectedSchedulingBase::sinkSTF(const HWAtom *A,
                                          unsigned ASAP, unsigned ALAP) {
  assert(ASAP <= ALAP && "Bad time frame to sink!");
  assert(ASAP >= getSTFASAP(A) && ALAP <= getSTFALAP(A) && "Can not Sink!");
  AtomToSTF[A] = std::make_pair(ASAP, ALAP);
  // We may need to reduce critical path.
  if (A == State->getExitRoot()) {
    assert(CriticalPathEnd >= ALAP && "Can not sink ExitRoot!");
    CriticalPathEnd = ALAP;
  }
}

void ForceDirectedSchedulingBase::updateSTF() {
  for (FSMState::iterator I = State->begin(), E = State->end();
      I != E; ++I) {
    HWAtom *A = *I;
    // Only update the scheduled time frame.
    if (!isSTFScheduled(A))
      continue;

    sinkSTF(A, getASAPStep(A), getALAPStep(A));
  }
}
