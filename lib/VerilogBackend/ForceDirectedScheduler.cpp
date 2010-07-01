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
  typedef std::map<HWAtom*, TimeFrame> TimeFrameMapType;

  /// @name TimeFrame
  //{
  TimeFrameMapType AtomToTF;
  void buildTimeFrame();

  unsigned computeASAPStep(HWAtom *A);
  unsigned getASAPStep(HWAtom *A) const {
    return const_cast<FDLScheduler*>(this)->AtomToTF[A].first;
  }
  void setASAPStep(HWAtom *A, unsigned step);

  unsigned computeALAPStep(HWAtom *A);
  unsigned getALAPStep(HWAtom *A) const { 
    return const_cast<FDLScheduler*>(this)->AtomToTF[A].second;
  }
  void setALAPStep(HWAtom *A, unsigned step);

  unsigned getTimeFrame(HWAtom *A) const {
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

  while (!isListEmpty()) {
    AtomToTF.clear();
    DGraph.clear();
  
    buildTimeFrame();

    buildDGraph();

    // TODO: Short the list
    HWAtom *A = *list_begin();
    if (HWAOpInst *OpInst = dyn_cast<HWAOpInst>(A)) {
    }
    //
    break;
  }
  
  HWAOpInst &Exit = CurStage->getExitRoot();
  Exit.scheduledTo(getALAPStep(&Exit));
  HI->setTotalCycle(CurStage->getExitRoot().getSlot() + 1);

  return false;
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
  for (usetree_iterator I = CurStage->usetree_begin(),
      E = CurStage->usetree_end(); I != E; ++I){
    if (HWAOpInst *OpInst = dyn_cast<HWAOpInst>(*I)) {
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
    for (unsigned i = getALAPStep(&CurStage->getEntryRoot()),
        e = getALAPStep(&CurStage->getExitRoot()) + 1; i != e; ++i) {
          OS.indent(2) << "At step " << i << " : " <<
            getDGraphAt(i, (enum HWResource::ResTypes)ri) << '\n';
    }
    OS << '\n';
  }
}

void FDLScheduler::buildTimeFrame() {
  HWAVRoot &Entry = CurStage->getEntryRoot();
  // Build the time frame
  setASAPStep(&Entry, 1); 
  HWAOpInst &Exit = CurStage->getExitRoot();
  unsigned ExitStep = getASAPStep(&Exit);
  setALAPStep(&Exit, ExitStep);
  DEBUG(printTimeFrame(dbgs()));
  ////Schedule to asap step.
  //for (usetree_iterator I = usetree_iterator::begin(&Entry),
  //    E = usetree_iterator::end(&Entry); I != E; ++I) {
  //  HWAtom *A = *I;
  //  A->scheduledTo(HI->getTotalCycle() + AtomToTF[A].first - 1);
  //}
  //HI->setTotalCycle(CurStage->getExitRoot().getSlot() + 1);
}

unsigned FDLScheduler::computeASAPStep(HWAtom *A) {
  unsigned ret = 0;
  for (HWAtom::dep_iterator I = A->dep_begin(), E = A->dep_end();
      I != E; ++I) {
    HWAtom *Dep = *I;
    unsigned AsapStep = getASAPStep(Dep);
    if (AsapStep == 0) // The node not visit yet
      return 0;

    AsapStep += Dep->getLatency();
    // The node will be schedule as soon as the last dependence scheduled
    if(AsapStep > ret)
      ret = AsapStep;
  }

  return ret;
}


void FDLScheduler::setASAPStep(HWAtom *A, unsigned step) {
  //DEBUG(A->dump());
  //DEBUG(dbgs() << "set to asap " << step << '\n');
  // TODO: Consider the scheduled node
  AtomToTF[A].first = step;;
  
  for (HWAtom::use_iterator I = A->use_begin(), E = A->use_end();
      I != E; ++I) {
    HWAtom *U = *I;
    // U was already been schedule to a asap step.
    if (getASAPStep(U) != 0)
      continue;
    
    unsigned dep_step = computeASAPStep(U);
    // Some dependencies of U not finish
    if (dep_step == 0)
      continue;

    // All depends of U finish, schedule it to a step.
    setASAPStep(U, dep_step);
  }
}

unsigned FDLScheduler::computeALAPStep(HWAtom *A) {
  unsigned ret = UINT32_MAX;
  for (HWAtom::use_iterator I = A->use_begin(), E = A->use_end();
    I != E; ++I) {
      HWAtom *Dep = *I;
      unsigned AlapStep = getALAPStep(Dep);
      if (AlapStep == 0) // The node not visit yet
        return 0;
      // The Node will be schedule only when the first use need it.
      else if(AlapStep < ret)
        ret = AlapStep;
  }
  // Now ret is the step that A should be finish, place it to the
  // right place so that i can be finish at this step
  ret -= A->getLatency();
  
  return ret;
}

void FDLScheduler::setALAPStep(HWAtom *A, unsigned step) {
  //DEBUG(A->dump());
  //DEBUG(dbgs() << "set to alap " << step << '\n');
  // TODO: Consider the scheduled node
  AtomToTF[A].second = step;;

  for (HWAtom::dep_iterator I = A->dep_begin(), E = A->dep_end();
    I != E; ++I) {
      HWAtom *U = *I;
      // U was already been schedule to a asap step.
      if (getALAPStep(U) != 0)
        continue;

      unsigned use_step = computeALAPStep(U);
      // Some dependencies of U not finish
      if (use_step == 0)
        continue;

      // All depends of U finish, schedule it to a step.
      setALAPStep(U, use_step);
  }
}

void FDLScheduler::clear() {
  AtomToTF.clear();
  DGraph.clear();
}

void FDLScheduler::printTimeFrame(raw_ostream &OS) const {
  for (usetree_iterator I = CurStage->usetree_begin(),
    E = CurStage->usetree_end(); I != E; ++I) {
    (*I)->print(OS);
    OS << " : {" << getASAPStep(*I) << "," << getALAPStep(*I)
       << "} " <<  getTimeFrame(*I) << "\n";
  }
  
}

void FDLScheduler::print(raw_ostream &O, const Module *M) const { }

Pass *esyn::createFDLSchedulePass() {
  return new FDLScheduler();
}
