//===- ForceDirectedInfo.cpp - ForceDirected information analyze --*- C++ -*-===//
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

#include "ForceDirectedInfo.h"
#include "ModuloScheduleInfo.h"
#include "HWAtomPasses.h"

#define DEBUG_TYPE "vbe-fd-info"
#include "llvm/Support/Debug.h"

using namespace llvm;
using namespace esyn;

//===----------------------------------------------------------------------===//
char ForceDirectedInfo::ID = 0;

RegisterPass<ForceDirectedInfo> X("vbe-fd-info",
                           "vbe - Compute necessary information for force"
                           " directed scheduling passes");

void ForceDirectedInfo::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequiredTransitive<HWAtomInfo>();
  AU.addRequiredTransitive<ResourceConfig>();
  AU.setPreservesAll();
}


void ForceDirectedInfo::buildASAPStep(const FSMState *EntryRoot,
                                      unsigned step) {
  AtomToTF[EntryRoot].first = step;

  for (FSMState::iterator I = State->begin(), E = State->end();
       I != E; ++I) {
    HWAtom *A = *I;
    if (A->isScheduled()) {
      AtomToTF[A].first = A->getSlot();
      continue;
    }

    unsigned NewStep = 0;

    for (HWAtom::dep_iterator DI = A->dep_begin(), DE = A->dep_end();
        DI != DE; ++DI) {
      const HWAtom *Dep = *DI;
      if (!DI.getEdge()->isBackEdge() || (Dep->isScheduled() && MII)) {
        unsigned Step = getASAPStep(Dep) + Dep->getLatency()
                        - MII * DI.getEdge()->getItDst();
        DEBUG(dbgs() << "From ";
              if (DI.getEdge()->isBackEdge())
                dbgs() << "BackEdge ";
              Dep->print(dbgs());
              dbgs() << " Step " << Step << '\n');
        NewStep = std::max(Step, NewStep);
      }
    }

    DEBUG(dbgs() << "Update ASAP step to: " << NewStep << " for \n";
    A->dump();
    dbgs() << "\n\n";);
    AtomToTF[A].first = NewStep;
  }

  HWAOpInst *Exit = State->getExitRoot();
  CriticalPathEnd = std::max(CriticalPathEnd, getASAPStep(Exit));
}

void ForceDirectedInfo::buildALAPStep(const HWAOpInst *ExitRoot,
                                      unsigned step) {
  AtomToTF[ExitRoot].second = step;

  for (FSMState::reverse_iterator I = State->rbegin(), E = State->rend();
       I != E; ++I) {
    HWAtom *A = *I;
    if (A->isScheduled()) {
      AtomToTF[A].second = A->getSlot();
      continue;
    }

    unsigned NewStep = AtomToTF[A].second;
 
    for (HWAtom::use_iterator UI = A->use_begin(), UE = A->use_end();
         UI != UE; ++UI) {
      HWEdge *UseEdge = (*UI)->getEdgeFrom(A);
      const HWAtom *Use = *UI;

      if (!UseEdge->isBackEdge()
          || (Use->isScheduled() && MII)) {
        unsigned Step = getALAPStep(Use) - A->getLatency()
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
    AtomToTF[A].second = NewStep;

    assert(getALAPStep(A) >= getASAPStep(A)
      && "Broken time frame!");
  }
}


void ForceDirectedInfo::printTimeFrame(raw_ostream &OS) const {
  OS << "Time frame:\n";
  for (FSMState::iterator I = State->begin(), E = State->end();
      I != E; ++I) {
    HWAtom *A = *I;
    A->print(OS);
    OS << " : {" << getASAPStep(A) << "," << getALAPStep(A)
      << "} " <<  getTimeFrame(A) << "\n";
  }
}

void ForceDirectedInfo::dumpTimeFrame() const {
  printTimeFrame(dbgs());
}

bool ForceDirectedInfo::isFUAvailalbe(unsigned step, HWFUnit FU) const {
  unsigned key = computeStepKey(step, FU.getFUnitID());
  unsigned usage = const_cast<ForceDirectedInfo*>(this)->ResUsage[key];
  return usage < FU.getTotalFUs();
}

void ForceDirectedInfo::presevesFUForAtom(HWAtom *A) {
  assert(A->isScheduled() && "Can only preseves FU for Scheduled Atom!");
  if (HWAOpInst *OI = dyn_cast<HWAOpInst>(A)) {
    if (OI->isTrivial())
      return;
    // Increase the resource usage for scheduled atom.
    unsigned StepKey = computeStepKey(OI->getSlot(),
                                      OI->getFunUnitID());
    ++ResUsage[StepKey];
  }
}

void ForceDirectedInfo::buildDGraph() {
  DGraph.clear();
  for (FSMState::iterator I = State->begin(), E = State->end(); I != E; ++I){
    // We only try to balance the post bind resource.
    if (HWAOpInst *OpInst = dyn_cast<HWAOpInst>(*I)) {
      // Ignore the DG for trivial resources.
      if (OpInst->isTrivial()) continue;

      unsigned TimeFrame = getTimeFrame(OpInst);
      unsigned ASAPStep = getASAPStep(OpInst), ALAPStep = getALAPStep(OpInst);

      HWFUnit FU= OpInst->getFunUnit();
      double Prob = 1.0 / (double) TimeFrame;
      // Including ALAPStep.
      for (unsigned i = ASAPStep, e = ALAPStep + 1; i != e; ++i)
        accDGraphAt(i, FU.getFUnitID(), Prob);
    }
  }
  DEBUG(printDG(dbgs()));
}

void ForceDirectedInfo::printDG(raw_ostream &OS) const {  
  // For each step
  for (DGType::const_iterator I = DGraph.begin(), E = DGraph.end(); I != E; ++I) {
    unsigned Key = I->first;
    double V = I->second;

    unsigned Step, FUID;
    decompseStepKey(Key, Step, FUID);
    OS << '[' << FUID << "] @ " << Step << ": " << V << '\n';
  }
}

unsigned
ForceDirectedInfo::computeStepKey(unsigned step, HWFUnitID FUID) const {
  if (MII != 0) {
#ifndef NDEBUG
    unsigned StartSlot = State->getSlot();
    step = StartSlot + (step - StartSlot) % MII;
#else
    step = step % MII;
#endif
  }

  union {
    struct {
      unsigned Step     : 16;
      unsigned ResData  : 16;
    } S;
    unsigned Data;
  } U;

  U.S.Step = step;
  U.S.ResData = FUID.getRawData();
  return U.Data;
}

void ForceDirectedInfo::decompseStepKey(unsigned key,
                                        unsigned &step, unsigned &FUID) {
  union {
    struct {
      unsigned Step     : 16;
      unsigned ResData  : 16;
    } S;
    unsigned Data;
  } U;

  U.Data = key;

  step = U.S.Step;
  FUID = U.S.ResData;
}

double ForceDirectedInfo::getDGraphAt(unsigned step, HWFUnitID FUID) const {
  // Modulo DG for modulo schedule.
  DGType::const_iterator at = DGraph.find(computeStepKey(step , FUID));
  
  if (at != DGraph.end()) return at->second;  

  return 0.0;
}

void ForceDirectedInfo::accDGraphAt(unsigned step, HWFUnitID FUID, double d) {
  // Modulo DG for modulo schedule.
  DGraph[computeStepKey(step , FUID)] += d;
}

double ForceDirectedInfo::getRangeDG(HWFUnitID FUID, unsigned start,
                                     unsigned end/*included*/) {
  double range = end - start + 1;
  double ret = 0.0;
  for (unsigned i = start, e = end + 1; i != e; ++i)
    ret += getDGraphAt(i, FUID);

  ret /= range;
  return ret;
}

double ForceDirectedInfo::computeSelfForceAt(const HWAOpInst *OpInst,
                                             unsigned step) {
 double Force = getDGraphAt(step, OpInst->getFunUnitID()) - getAvgDG(OpInst);
 HWFUnit FU = OpInst->getFunUnit();
 return Force / FU.getTotalFUs();
}

double ForceDirectedInfo::computeSuccForceAt(const HWAOpInst *OpInst,
                                             unsigned step) {
  double ret = 0.0;

  for (const_usetree_iterator I = const_usetree_iterator::begin(OpInst),
      E = const_usetree_iterator::end(OpInst); I != E; ++I) {
    if (*I == OpInst)
      continue;
    
    // Ignore backedge.
    if (I->getIdx() < OpInst->getIdx())
      continue;

    if (const HWAOpInst *P = dyn_cast<HWAOpInst>(*I)) {
      HWFUnit FU = P->getFunUnit();
      double Force = getRangeDG(P->getFunUnitID(), getASAPStep(P), getALAPStep(P))
                     - getAvgDG(P);
      ret += Force / FU.getTotalFUs();
    }
  }

  return ret;
}

double ForceDirectedInfo::computePredForceAt(const HWAOpInst *OpInst,
                                             unsigned step) {
  double ret = 0;

  for (const_deptree_iterator I = const_deptree_iterator::begin(OpInst),
      E = const_deptree_iterator::end(OpInst); I != E; ++I) {
    if (*I == OpInst)
      continue;

    // Ignore backedge.
    if (I->getIdx() > OpInst->getIdx())
      continue;

    if (const HWAOpInst *P = dyn_cast<HWAOpInst>(*I)) {
      HWFUnit FU = P->getFunUnit();
      double Force = getRangeDG(P->getFunUnitID(), getASAPStep(P), getALAPStep(P))
                     - getAvgDG(P);
      ret += Force / FU.getTotalFUs();
    }
  }

  return ret;
}

void ForceDirectedInfo::buildAvgDG() {
  for (FSMState::iterator I = State->begin(), E = State->end();
       I != E; ++I)
    // We only care about the utilization of post bind resource. 
    if (HWAOpInst *A = dyn_cast<HWAOpInst>(*I)) {
      double res = 0.0;
      for (unsigned i = getASAPStep(A), e = getALAPStep(A) + 1; i != e; ++i)
        res += getDGraphAt(i, A->getFunUnitID());

      res /= (double) getTimeFrame(A);
      AvgDG[A] = res;
    }
}

void ForceDirectedInfo::reset() {
  AtomToTF.clear();
  DGraph.clear();
  ResUsage.clear();
  AvgDG.clear();
}

void ForceDirectedInfo::releaseMemory() {
  reset();
  MII = 0;
  CriticalPathEnd = 0;
}

bool ForceDirectedInfo::runOnBasicBlock(BasicBlock &BB) {
  HWAtomInfo &HI = getAnalysis<HWAtomInfo>();
  State = HI.getStateFor(BB);
  return false;
}

void ForceDirectedInfo::initALAPStep() {
  for (FSMState::iterator I = State->begin(), E = State->end();
      I != E; ++I) {
    HWAtom *A = *I;
    //
    // Set up the II constrain.
    if (MII)
      AtomToTF[A].second = getASAPStep(A) + MII - A->getLatency();
    else
      AtomToTF[A].second = HWAtom::MaxSlot;
  }
}

unsigned ForceDirectedInfo::buildFDInfo() {
  // Build the time frame
  assert(State->isScheduled() && "Entry must be scheduled first!");
  unsigned FirstStep = State->getSlot();
  buildASAPStep(State, FirstStep);
  initALAPStep();
  buildALAPStep(State->getExitRoot(), CriticalPathEnd);

  DEBUG(dumpTimeFrame());

  buildDGraph();
  buildAvgDG();

  return CriticalPathEnd;
}

void ForceDirectedInfo::dumpDG() const {
  printDG(dbgs());
}
