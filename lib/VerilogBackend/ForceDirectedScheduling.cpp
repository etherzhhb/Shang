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

#define DEBUG_TYPE "vbe-fd-info"
#include "llvm/Support/Debug.h"

using namespace llvm;
using namespace esyn;

static cl::opt<bool>
NoFDSchedule("disable-fd-schedule",
             cl::desc("vbe - Do not preform force-directed schedule"),
             cl::Hidden, cl::init(false));

//===----------------------------------------------------------------------===//
void ForceDirectedSchedulingBase::buildTimeFrame() {
  AtomToTF.clear();
  // Build the time frame
  assert(State->isScheduled() && "Entry must be scheduled first!");
  unsigned FirstStep = State->getSlot();
  buildASAPStep(State, FirstStep);
  buildALAPStep(State->getExitRoot(), CriticalPathEnd);

  DEBUG(dumpTimeFrame());
}

void ForceDirectedSchedulingBase::buildASAPStep(const HWAtom *Root, unsigned step) {
  AtomToTF[Root].first = step;

  FSMState::iterator Start = std::find(State->begin(), State->end(),
                                       Root);

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

      unsigned NewStep = getSTFASAP(A);

      for (HWAtom::dep_iterator DI = A->dep_begin(), DE = A->dep_end();
          DI != DE; ++DI) {
        const HWAtom *Dep = *DI;
        if (!DI.getEdge()->isBackEdge() || MII) {
          unsigned DepASAP = Dep->isScheduled() ?
                             Dep->getSlot() : getASAPStep(Dep);
          int Step = DepASAP + Dep->getLatency()
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

  HWAOpFU *Exit = State->getExitRoot();
  CriticalPathEnd = std::max(CriticalPathEnd, getASAPStep(Exit));
}

void ForceDirectedSchedulingBase::buildALAPStep(const HWAtom *Root, unsigned step) {
  AtomToTF[Root].second = step;

  FSMState::reverse_iterator Start = std::find(State->rbegin(), State->rend(),
                                               Root);

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

      unsigned NewStep = getSTFALAP(A);
   
      for (HWAtom::use_iterator UI = A->use_begin(), UE = A->use_end();
           UI != UE; ++UI) {
        HWEdge *UseEdge = (*UI)->getEdgeFrom(A);
        const HWAtom *Use = *UI;

        if (!UseEdge->isBackEdge() || MII) {
          unsigned UseALAP = Use->isScheduled() ?
                             Use->getSlot() : getALAPStep(Use);
          if (UseALAP == 0) {
            assert(UseEdge->isBackEdge() && "Broken time frame!");
            UseALAP = HWAtom::MaxSlot;
          }
          
          unsigned Step = UseALAP - A->getLatency()
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

      assert(getALAPStep(A) >= getASAPStep(A)
             && "Broken time frame!");
    }
  } while (changed);
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
    if (HWAOpFU *OpInst = dyn_cast<HWAOpFU>(*I)) {
      // Ignore the DG for trivial resources.
      if (OpInst->isTrivial()) continue;

      unsigned TimeFrame = getTimeFrame(OpInst);
      unsigned ASAPStep = getASAPStep(OpInst), ALAPStep = getALAPStep(OpInst);

      HWFUnit *FU = OpInst->getFUnit();
      double Prob = 1.0 / (double) TimeFrame;
      // Including ALAPStep.
      for (unsigned i = ASAPStep, e = ALAPStep + 1; i != e; ++i)
        accDGraphAt(i, FU, Prob);
    }
  }
  DEBUG(printDG(dbgs()));
}


bool ForceDirectedSchedulingBase::isResourceConstraintPreserved() {
  ExtraRresRequirement = 0.0;
  // No resource in use.
  if (DGraph.empty()) return true;

  for (DGType::const_iterator I = DGraph.begin(), E = DGraph.end();
       I != E; ++I) {
    DEBUG(dbgs() << "FU " << *I->first << " Average Usage: ");
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
    if (AverageDG > I->first->getTotalFUs() + e)
      ExtraRresRequirement += (AverageDG - I->first->getTotalFUs())
                              /  I->first->getTotalFUs();
  }
  return ExtraRresRequirement == 0.0;
}

void ForceDirectedSchedulingBase::printDG(raw_ostream &OS) const {  
  // For each step
  // For each FU.
  for (DGType::const_iterator I = DGraph.begin(), E = DGraph.end();
       I != E; ++I) {
    OS << "FU " << *I->first << ":\n";
    for (DGStepMapType::const_iterator SI = I->second.begin(),
         SE = I->second.end(); SI != SE; ++SI) {
      OS << "@ " << SI->first << ": " << SI->second << '\n';
    }
    OS << '\n';
  }
}

double ForceDirectedSchedulingBase::getDGraphAt(unsigned step, HWFUnit *FU) const {
  // Modulo DG for modulo schedule.
  DGType::const_iterator at = DGraph.find(FU);
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
    unsigned StartSlot = State->getSlot();
    step = StartSlot + (step - StartSlot) % MII;
#else
    step = step % MII;
#endif
  }

  return step;
}

void ForceDirectedSchedulingBase::accDGraphAt(unsigned step, HWFUnit *FU, double d) {
  // Modulo DG for modulo schedule.
  DGraph[FU][computeStepKey(step)] += d;
}

// Including end.
double ForceDirectedSchedulingBase::getRangeDG(HWFUnit  *FU, unsigned start, unsigned end) {
  double range = end - start + 1;
  double ret = 0.0;
  for (unsigned i = start, e = end + 1; i != e; ++i)
    ret += getDGraphAt(i, FU);

  ret /= range;
  return ret;
}

double ForceDirectedSchedulingBase::computeForce(const HWAtom *A,
                                       unsigned ASAP, unsigned ALAP) {
  sinkSTF(A, ASAP, ALAP);
  buildTimeFrame();
  // Compute the forces.
  double SelfForce = computeSelfForce(A);
  // The follow function will invalid the time frame.
  DEBUG(dbgs() << " Self Force: " << SelfForce);
  double PredForce = computePredForce(A);
  DEBUG(dbgs() << " Pred Force: " << PredForce);
  double SuccForce = computeSuccForce(A);
  DEBUG(dbgs() << " Succ Force: " << SuccForce);
  double Force = SelfForce + PredForce + SuccForce;
  DEBUG(dbgs() << " Force: " << Force);
  return Force;
}

double ForceDirectedSchedulingBase::computeSelfForce(const HWAtom *A) {
  const HWAOpFU *OpInst = dyn_cast<HWAOpFU>(A);
  if (!OpInst) return 0.0;

  if (NoFDSchedule && !OpInst->isBinded()) return 0.0;

  HWFUnit *FU = OpInst->getFUnit();
  double Force = getRangeDG(FU, getSTFASAP(A), getSTFASAP(A))
                 - getAvgDG(OpInst);

  // Make the atoms taking expensive function unit have bigger force.
  return Force / FU->getTotalFUs();
}

double ForceDirectedSchedulingBase::computeRangeForce(const HWAtom *A,
                                            unsigned int start,
                                            unsigned int end) {
  const HWAOpFU *OpInst = dyn_cast<HWAOpFU>(A);
  if (!OpInst) return 0.0;

  if (NoFDSchedule && !OpInst->isBinded()) return 0.0;

  HWFUnit *FU = OpInst->getFUnit();
  double Force = getRangeDG(FU, start, end) - getAvgDG(OpInst);
  return Force / FU->getTotalFUs();
}

double ForceDirectedSchedulingBase::computeSuccForce(const HWAtom *A) {
  double ret = 0.0;

  for (const_usetree_iterator I = const_usetree_iterator::begin(A),
       E = const_usetree_iterator::end(A); I != E; ++I)
    if (const HWAOpFU *OI = dyn_cast<HWAOpFU>(*I))
      ret += computeRangeForce(OI, getASAPStep(OI), getALAPStep(OI));

  return ret;
}

double ForceDirectedSchedulingBase::computePredForce(const HWAtom *A) {
  // Adjust the time frame.
  //buildALAPStep(A, step);

  double ret = 0;

  for (const_deptree_iterator I = const_deptree_iterator::begin(A),
       E = const_deptree_iterator::end(A); I != E; ++I)
    if (const HWAOpFU *OI = dyn_cast<HWAOpFU>(*I))
      ret += computeRangeForce(OI, getASAPStep(OI), getALAPStep(OI));

  return ret;
}

void ForceDirectedSchedulingBase::buildAvgDG() {
  AvgDG.clear();
  for (FSMState::iterator I = State->begin(), E = State->end();
       I != E; ++I)
    // We only care about the utilization of post bind resource. 
    if (HWAOpFU *A = dyn_cast<HWAOpFU>(*I)) {
      double res = 0.0;
      for (unsigned i = getASAPStep(A), e = getALAPStep(A) + 1; i != e; ++i)
        res += getDGraphAt(i, A->getFUnit());

      res /= (double) getTimeFrame(A);
      AvgDG[A] = res;
    }
}

unsigned ForceDirectedSchedulingBase::buildFDInfo(bool rstSTF) {
  if (rstSTF) {
    resetSTF();
    unsigned StartStep = HI->getTotalCycle();
    State->resetSchedule(StartStep);
  }
  buildTimeFrame();
  if (rstSTF) updateSTF();

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

void ForceDirectedSchedulingBase::sinkSTF(const HWAtom *A, unsigned ASAP, unsigned ALAP) {
  assert(ASAP <= ALAP && "Bad time frame to sink!");
  AtomToSTF[A] = std::make_pair(ASAP, ALAP);
}

void ForceDirectedSchedulingBase::updateSTF() {
  for (FSMState::iterator I = State->begin(), E = State->end();
      I != E; ++I) {
    HWAtom *A = *I;
    sinkSTF(A, getASAPStep(A), getALAPStep(A));
  }
}
