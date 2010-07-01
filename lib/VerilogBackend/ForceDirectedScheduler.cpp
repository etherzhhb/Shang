//===- ForceDirectedScheduler.cpp - The ForceDirected Scheduler  -*- C++ -*-===//
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
// This file implement the Force Direct Scheduler pass describe in
// Force-Directed Scheduling for the Behavioral Synthesis of ASIC's
//
//===----------------------------------------------------------------------===//

#include "vbe/SchedulerBase.h"
#include "HWAtomInfo.h"
#include "HWAtomPasses.h"


#define DEBUG_TYPE "vbe-fd-schedule"
#include "llvm/Support/Debug.h"

using namespace llvm;
using namespace esyn;

namespace {

struct FDLScheduler : public BasicBlockPass, public Scheduler {

  HWAtomInfo *HI;
  ResourceConfig *RC;

  // Time Frame {asap step, alap step }
  typedef std::pair<unsigned, unsigned> TimeFrame;
  // Mapping hardware atoms to time frames.
  typedef std::map<HWAOpInst*, TimeFrame> TimeFrameMapType;

  /// @name TimeFrame
  //{
  TimeFrameMapType AtomToTF;
  void buildTimeFrame();

  void buildASAPStep(HWAtom *A, unsigned step);
  unsigned getASAPStep(HWAOpInst *A) const {
    return const_cast<FDLScheduler*>(this)->AtomToTF[A].first;
  }


  void buildALAPStep(HWAtom *A, unsigned step);
  unsigned getALAPStep(HWAOpInst *A) const { 
    return const_cast<FDLScheduler*>(this)->AtomToTF[A].second;
  }

  unsigned getTimeFrame(HWAOpInst *A) const {
    return getALAPStep(A) - getASAPStep(A) + 1;
  }

  void printTimeFrame(raw_ostream &OS) const ;
  //}

  /// @name Distribuition Graphs
  //{
  // The Key of DG, { step, resource type }
  typedef std::map<unsigned, double> DGType;

  DGType DGraph;
  void buildDGraph();
  double getDGraphAt(unsigned step, enum HWResource::ResTypes ResType) const;
  void accDGraphAt(unsigned step, enum HWResource::ResTypes ResType, double d);
  void printDG(raw_ostream &OS) const ;
  //}

  std::map<HWAOpInst*, double> AvgDG;
  void buildAvgDG();
  double getAvgDG(HWAOpInst *A);
  double getRangeDG(HWAOpInst *A, unsigned start, unsigned end/*included*/);

  double computeSelfForceAt(HWAOpInst *OpInst, double step);
  double computeSuccForceAt(HWAOpInst *OpInst, double step);
  double computePredForceAt(HWAOpInst *OpInst, double step);

  void reset();
  void clear();

  /// @name Common pass interface
  //{
  static char ID;
  FDLScheduler() : BasicBlockPass(&ID), Scheduler(), HI(0), RC(0) {}
  bool runOnBasicBlock(BasicBlock &BB);
  void releaseMemory();
  void getAnalysisUsage(AnalysisUsage &AU) const;
  virtual void print(raw_ostream &O, const Module *M) const;
  //}
};
} //end namespace

char FDLScheduler::ID = 0;

void FDLScheduler::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<HWAtomInfo>();
  AU.addRequired<ResourceConfig>();
  AU.setPreservesAll();
}

bool FDLScheduler::runOnBasicBlock(BasicBlock &BB) {
  DEBUG(dbgs() << "==================== " << BB.getName() << '\n');
  HI = &getAnalysis<HWAtomInfo>();
  CurStage = &(HI->getStateFor(BB));
  
  createAtomList();

  HWAVRoot &EntryRoot = CurStage->getEntryRoot();
  EntryRoot.scheduledTo(HI->getTotalCycle());

  for (usetree_iterator I = CurStage->usetree_begin(),
      E = CurStage->usetree_end(); I != E; ++I)
    if (HWAOpInst *OpInst = dyn_cast<HWAOpInst>(*I))
      AtomToTF[OpInst] = std::make_pair(0, UINT32_MAX);

  buildTimeFrame();

  while (!isListEmpty()) {
    buildDGraph();
    buildAvgDG();

    // TODO: Short the list
    HWAOpInst *A = *list_begin();
    DEBUG(A->print(dbgs()));
    DEBUG(dbgs() << " Active to schedule:-------------------\n");
    // Remember the best Step.
    std::pair<unsigned, double> bestStep = std::make_pair(0, 1e8);
    // For each possible step:
    for (unsigned i = getASAPStep(A), e = getALAPStep(A) + 1; i != e; ++i) {
      DEBUG(dbgs() << "At Step " << i);
      double SelfForce = computeSelfForceAt(A, i);
      DEBUG(dbgs() << " Self Force: " << SelfForce);

      // Update time frame
      buildASAPStep(A, i);
      // Make schedule to i.
      buildALAPStep(A, i + A->getLatency());

      double PredForce = computePredForceAt(A, i);
      DEBUG(dbgs() << " Pred Force: " << PredForce);

      double SuccForce = computeSuccForceAt(A, i);
      DEBUG(dbgs() << " Succ Force: " << SuccForce);

      double Force = SelfForce + PredForce + SuccForce;

      if (Force < bestStep.second)
        bestStep = std::make_pair(i, Force);

      DEBUG(dbgs() << '\n');
    }
    
    A->scheduledTo(bestStep.first);
    // Dirty hack, Recover the time frame.
    // Update time frame
    buildASAPStep(A, A->getSlot());
    // Make schedule to i.
    buildALAPStep(A, A->getSlot() + A->getLatency());

    removeFromList(list_begin());

    DEBUG(dbgs() << " After schedule:-------------------\n");
    DEBUG(printTimeFrame(dbgs()));
    DEBUG(dbgs() << "\n\n\n");
  }
  
  HI->setTotalCycle(CurStage->getExitRoot().getSlot() + 1);

  return false;
}

void FDLScheduler::buildAvgDG() {
  for (usetree_iterator I = CurStage->usetree_begin(),
      E = CurStage->usetree_end(); I != E; ++I)
    if (HWAOpInst *A = dyn_cast<HWAOpInst>(*I)) {
      double res = 0.0;
      for (unsigned i = getASAPStep(A), e = getALAPStep(A) + 1; i != e; ++i)
        res += getDGraphAt(i, A->getResClass());

      res /= (double) getTimeFrame(A);
      AvgDG[A] = res;
    }
}

double FDLScheduler::getAvgDG(HWAOpInst *A) {
  return AvgDG[A];
}


double FDLScheduler::getRangeDG(HWAOpInst *A,
                                unsigned start, unsigned end/*included*/) {
  double range = end - start + 1;
  double ret = 0.0;
  for (unsigned i = start, e = end + 1; i != e; ++i)
    ret += getDGraphAt(i, A->getResClass());

  ret /= range;
  return ret;
}

double FDLScheduler::computeSelfForceAt(HWAOpInst *OpInst, double step) {
  return getDGraphAt(step, OpInst->getResClass()) - getAvgDG(OpInst);
}

double FDLScheduler::computeSuccForceAt(HWAOpInst *OpInst, double step) {
  // All successor will start when this operation finish
  unsigned newStep = step + OpInst->getLatency();
  
  double ret = 0;

  for (usetree_iterator I = usetree_iterator::begin(OpInst),
      E = usetree_iterator::end(OpInst); I != E; ++I) {
    if (*I == OpInst)
      continue;
    
    if (HWAOpInst *U = dyn_cast<HWAOpInst>(*I)) {
      double UF = getRangeDG(U, getASAPStep(U), getALAPStep(U)) - getAvgDG(U);
      ret += UF;
      // Dirty hack, invalid the time frame of U
      AtomToTF[OpInst] = std::make_pair(0, UINT32_MAX);
    }
  }
  return ret;
}


double FDLScheduler::computePredForceAt(HWAOpInst *OpInst, double step) {
  // All successor will start when this operation finish
  unsigned newStep = step + OpInst->getLatency();

  double ret = 0;

  for (deptree_iterator I = deptree_iterator::begin(OpInst),
      E = deptree_iterator::end(OpInst); I != E; ++I) {
    if (*I == OpInst)
      continue;

    if (HWAOpInst *U = dyn_cast<HWAOpInst>(*I)) {
      double UF = getRangeDG(U, getASAPStep(U), getALAPStep(U)) - getAvgDG(U);
      ret += UF;
      // Dirty hack, invalid the time frame of U
      AtomToTF[OpInst] = std::make_pair(0, UINT32_MAX);
    }
  }
  
  return ret;
}

void FDLScheduler::releaseMemory() {
  clear();
  clearSchedulerBase();
}

double FDLScheduler::getDGraphAt(unsigned step,
                                 enum HWResource::ResTypes ResType) const {
  unsigned key = (step << 4) | (0xf & ResType);
  DGType::const_iterator at = DGraph.find(key);
  
  if (at != DGraph.end()) return at->second;  

  return 0.0;
}

void FDLScheduler::accDGraphAt(unsigned step, enum HWResource::ResTypes ResType,
                               double d) {
  unsigned key = (step << 4) | (0xf & ResType);
  DGraph[key] += d;
}

void FDLScheduler::buildDGraph() {
  DGraph.clear();
  for (usetree_iterator I = CurStage->usetree_begin(),
      E = CurStage->usetree_end(); I != E; ++I){
    if (HWAOpInst *OpInst = dyn_cast<HWAOpInst>(*I)) {
      if (OpInst->getResClass() == HWResource::Trivial)
        continue;
      
      double Prob = 1.0 / (double) getTimeFrame(OpInst);
      // Including ALAPStep.
      for (unsigned i = getASAPStep(OpInst), e = getALAPStep(OpInst) + 1;
          i != e; ++i)
        accDGraphAt(i, OpInst->getResClass(), Prob);
    }
  }
  DEBUG(printDG(dbgs()));
}

void FDLScheduler::printDG(raw_ostream &OS) const {
  // For each step
  for (unsigned ri = HWResource::FirstResourceType,
      re = HWResource::LastResourceType; ri != re; ++ri) {
    OS << "DG for resource: " << ri <<'\n';
    for (unsigned i = CurStage->getEntryRoot().getSlot(),
        e = getASAPStep(&CurStage->getExitRoot()) + 1; i != e; ++i)
      OS.indent(2) << "At step " << i << " : "
        << getDGraphAt(i, (enum HWResource::ResTypes)ri) << '\n';

    OS << '\n';
  }
}

void FDLScheduler::buildASAPStep(HWAtom *A, unsigned step) {
  if (HWAOpInst *OI = dyn_cast<HWAOpInst>(A)) {
    unsigned ExistStep = getASAPStep(OI);

    // Schedule A to this step.
    if (A->isScheduled())
      step = A->getSlot();  

    // A is not ready at this step.
    if (ExistStep >= step)
      return;

    AtomToTF[OI].first = step;
  }

  // The successor will be schedule when A is finish
  step += A->getLatency(); 

  for (HWAtom::use_iterator I = A->use_begin(), E = A->use_end(); I != E; ++I)
    buildASAPStep(*I, step);
}

void FDLScheduler::buildALAPStep(HWAtom *A, unsigned step) {
  // The step is the finish time, translate it to start time;
  step -= A->getLatency(); 

  if (HWAOpInst *OI = dyn_cast<HWAOpInst>(A)) {
    unsigned ExistStep = getALAPStep(OI);

    // Schedule A to this step.
    if (A->isScheduled())
      step = A->getSlot();  

    // A is not ready at this step or alreay schedule to this step.
    if (ExistStep <= step)
      return;

    AtomToTF[OI].second = step;
  }

  for (HWAtom::dep_iterator I = A->dep_begin(), E = A->dep_end(); I != E; ++I)
    buildALAPStep(*I, step);
}

void FDLScheduler::buildTimeFrame() {
  HWAVRoot &Entry = CurStage->getEntryRoot();
  // Build the time frame
  buildASAPStep(&Entry, Entry.getSlot()); 
  HWAOpInst &Exit = CurStage->getExitRoot();
  unsigned ExitStep = getASAPStep(&Exit);
  buildALAPStep(&Exit, ExitStep);
  DEBUG(printTimeFrame(dbgs()));
}

void FDLScheduler::clear() {
  AtomToTF.clear();
  DGraph.clear();
}

void FDLScheduler::printTimeFrame(raw_ostream &OS) const {
  OS << "Time frame:\n";
  for (usetree_iterator I = CurStage->usetree_begin(),
      E = CurStage->usetree_end(); I != E; ++I) {
    if (HWAOpInst *A = dyn_cast<HWAOpInst>(*I)) {
      A->print(OS);
      OS << " : {" << getASAPStep(A) << "," << getALAPStep(A)
         << "} " <<  getTimeFrame(A) << "\n";
    }
  }
  
}

void FDLScheduler::print(raw_ostream &O, const Module *M) const { }

Pass *esyn::createFDLSchedulePass() {
  return new FDLScheduler();
}
