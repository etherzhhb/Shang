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

#include "llvm/ADT/PriorityQueue.h"

#define DEBUG_TYPE "vbe-fd-schedule"
#include "llvm/Support/Debug.h"

using namespace llvm;
using namespace esyn;

namespace {
struct FDLScheduler;

struct fds_sort {
  FDLScheduler *FDS;
  fds_sort(FDLScheduler *s) : FDS(s) {}
  bool operator() (const HWAOpInst* LHS, const HWAOpInst* RHS) const;
};

struct FDLScheduler : public BasicBlockPass, public Scheduler {
  HWAtomInfo *HI;
  ResourceConfig *RC;

  // Time Frame {asap step, alap step }
  typedef std::pair<unsigned, unsigned> TimeFrame;
  // Mapping hardware atoms to time frames.
  typedef std::map<const HWAOpInst*, TimeFrame> TimeFrameMapType;

  /// @name TimeFrame
  //{
  TimeFrameMapType AtomToTF;
  void buildTimeFrame();

  void buildASAPStep(const HWAtom *A, unsigned step);
  unsigned getASAPStep(const HWAOpInst *A) const {
    return const_cast<FDLScheduler*>(this)->AtomToTF[A].first;
  }


  void buildALAPStep(const HWAtom *A, unsigned step);
  unsigned getALAPStep(const HWAOpInst *A) const { 
    return const_cast<FDLScheduler*>(this)->AtomToTF[A].second;
  }

  unsigned getTimeFrame(const HWAOpInst *A) const {
    return getALAPStep(A) - getASAPStep(A) + 1;
  }

  void printTimeFrame(raw_ostream &OS) const;
  void dumpTimeFrame() const;
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

  /// @name Force computation
  //{
  std::map<const HWAPostBind*, double> AvgDG;
  void buildAvgDG();
  double getAvgDG(const HWAPostBind *A) {  return AvgDG[A]; }
  double getRangeDG(const HWAPostBind *A, unsigned start, unsigned end/*included*/);

  double computeSelfForceAt(const HWAOpInst *OpInst, unsigned step);
  /// This function will invalid the asap step of all node in
  /// successor tree
  double computeSuccForceAt(const HWAOpInst *OpInst, unsigned step);
  /// This function will invalid the alap step of all node in
  /// predecessor tree
  double computePredForceAt(const HWAOpInst *OpInst, unsigned step);

  unsigned findBestStep(const HWAOpInst *A);
  //}

  /// @name PriorityQueue
  //{
  PriorityQueue<HWAOpInst*, std::vector<HWAOpInst*>, fds_sort> PQueue;
  void fillPQueue();
  //}

  void reset();
  void clear();

  /// @name Common pass interface
  //{
  static char ID;
  FDLScheduler()
    : BasicBlockPass(&ID), Scheduler(), HI(0), RC(0), PQueue(fds_sort(this)) {}
  bool runOnBasicBlock(BasicBlock &BB);
  void releaseMemory();
  void getAnalysisUsage(AnalysisUsage &AU) const;
  virtual void print(raw_ostream &O, const Module *M) const;
  //}
};
} //end namespace

//===----------------------------------------------------------------------===//
bool fds_sort::operator()(const HWAOpInst* LHS, const HWAOpInst* RHS) const {
  unsigned LHSTF = FDS->getTimeFrame(LHS), RHSTF = FDS->getTimeFrame(RHS);
  // Schedule the low mobility nodes first.
  if (LHSTF < RHSTF)
    return true;
  
  return false;
}

//===----------------------------------------------------------------------===//
char FDLScheduler::ID = 0;

void FDLScheduler::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<HWAtomInfo>();
  AU.addRequired<ResourceConfig>();
  AU.setPreservesAll();
}

void FDLScheduler::fillPQueue() {
  for (usetree_iterator I = CurStage->usetree_begin(), E = CurStage->usetree_end();
      I != E; ++I)
    if (HWAOpInst *A = dyn_cast<HWAOpInst>(*I))    
      PQueue.push(A);
}

bool FDLScheduler::runOnBasicBlock(BasicBlock &BB) {
  DEBUG(dbgs() << "==================== " << BB.getName() << '\n');
  HI = &getAnalysis<HWAtomInfo>();
  CurStage = &(HI->getStateFor(BB));


  HWAVRoot &EntryRoot = CurStage->getEntryRoot();
  EntryRoot.scheduledTo(HI->getTotalCycle());

  for (usetree_iterator I = CurStage->usetree_begin(),
      E = CurStage->usetree_end(); I != E; ++I)
    if (const HWAOpInst *OpInst = dyn_cast<HWAOpInst>(*I))
      AtomToTF[OpInst] = std::make_pair(0, UINT32_MAX);

  buildTimeFrame();
  buildDGraph();
  buildAvgDG();

  fillPQueue();

  while (!PQueue.empty()) {
    // TODO: Short the list
    HWAOpInst *A = PQueue.top();
    PQueue.pop();

    DEBUG(A->print(dbgs()));
    DEBUG(dbgs() << " Active to schedule:-------------------\n");
    
    // TODO: Do check if this step is valid for others constrain.
    unsigned step = getASAPStep(A);
    if (getTimeFrame(A) != 1) {
      TimeFrameMapType TmpMap = AtomToTF;
      step = findBestStep(A);
      AtomToTF = TmpMap;
      AtomToTF[A] = std::make_pair(0, UINT32_MAX);
      // Recover the time frame by force rebuild
      buildASAPStep(A, step);
      buildALAPStep(A, step + A->getLatency());
      PQueue.reheapify();
    }
    
    // Schedule to the best step.
    A->scheduledTo(step);

    DEBUG(dbgs() << " After schedule:-------------------\n");
    DEBUG(dumpTimeFrame());
    // Rebuild DG.
    buildDGraph();
    buildAvgDG();
    
    DEBUG(dbgs() << "\n\n\n");
  }

  HI->setTotalCycle(CurStage->getExitRoot().getSlot() + 1);

  return false;
}


unsigned FDLScheduler::findBestStep(const HWAOpInst *A) {
  // Remember the best Step and lowest force.
  unsigned Frame = getTimeFrame(A), FrameStart = getASAPStep(A);
  SmallVector<double, 8> Forces(Frame);
  // For each possible step:
  for (unsigned i = 0, e = Frame; i != e; ++i) {
    unsigned ASAPStep = FrameStart + i, ALAPStep = FrameStart + Frame - i - 1; 
    // Compute the forces.
    DEBUG(dbgs() << "At Step " << ASAPStep);
    double SelfForce = computeSelfForceAt(A, ASAPStep);
    // Force update time frame
    AtomToTF[A] = std::make_pair(0, UINT32_MAX);
    buildASAPStep(A, ASAPStep);
    buildALAPStep(A, ALAPStep + A->getLatency());

    // The follow function will invalid the time frame.
    DEBUG(dbgs() << " Self Force: " << SelfForce);
    double PredForce = computePredForceAt(A, ALAPStep);
    DEBUG(dbgs() << " Pred Force: " << PredForce);
    double SuccForce = computeSuccForceAt(A, ASAPStep);
    DEBUG(dbgs() << " Succ Force: " << SuccForce);
    //double Force = SelfForce + PredForce + SuccForce;
    Forces[i] += SelfForce + SuccForce;
    Forces[Frame - i - 1] += PredForce;

    DEBUG(dbgs() << '\n');
  }

  DEBUG(
    for (unsigned i = 0, e = Frame; i != e; ++i)
      dbgs() << "Force at : " << Forces[i] << '\n';
  ); 

  SmallVector<double, 8>::iterator MimForceAt
    = std::min_element(Forces.begin(), Forces.end());

  return FrameStart + (MimForceAt - Forces.begin());
}

void FDLScheduler::buildAvgDG() {
  for (usetree_iterator I = CurStage->usetree_begin(),
      E = CurStage->usetree_end(); I != E; ++I)
    // We only care about the utilization of prebind resource. 
    if (HWAPostBind *A = dyn_cast<HWAPostBind>(*I)) {
      double res = 0.0;
      for (unsigned i = getASAPStep(A), e = getALAPStep(A) + 1; i != e; ++i)
        res += getDGraphAt(i, A->getResClass());

      res /= (double) getTimeFrame(A);
      AvgDG[A] = res;
    }
}

double FDLScheduler::getRangeDG(const HWAPostBind *A,
                                unsigned start, unsigned end/*included*/) {
  double range = end - start + 1;
  double ret = 0.0;
  for (unsigned i = start, e = end + 1; i != e; ++i)
    ret += getDGraphAt(i, A->getResClass());

  ret /= range;
  return ret;
}

double FDLScheduler::computeSelfForceAt(const HWAOpInst *OpInst, unsigned step) {
  if (const HWAPostBind *A = dyn_cast<HWAPostBind>(OpInst))  
    return getDGraphAt(step, A->getResClass()) - getAvgDG(A);

  // The force about the pre-bind resoure dose not matter.
  return 0.0;
}

double FDLScheduler::computeSuccForceAt(const HWAOpInst *OpInst, unsigned step) {
  double ret = 0;

  for (const_usetree_iterator I = const_usetree_iterator::begin(OpInst),
      E = const_usetree_iterator::end(OpInst); I != E; ++I) {
    if (*I == OpInst)
      continue;
    
    if (const HWAOpInst *U = dyn_cast<HWAOpInst>(*I)) {
      if (const HWAPostBind *P = dyn_cast<HWAPostBind>(U))
        ret += getRangeDG(P, getASAPStep(P), getALAPStep(P)) - getAvgDG(P);
    }
  }

  return ret;
}

double FDLScheduler::computePredForceAt(const HWAOpInst *OpInst, unsigned step) {
  double ret = 0;

  for (const_deptree_iterator I = const_deptree_iterator::begin(OpInst),
      E = const_deptree_iterator::end(OpInst); I != E; ++I) {
    if (*I == OpInst)
      continue;

    if (const HWAOpInst *U = dyn_cast<HWAOpInst>(*I)) {
      if (const HWAPostBind *P = dyn_cast<HWAPostBind>(U))
        ret += getRangeDG(P, getASAPStep(P), getALAPStep(P)) - getAvgDG(P);
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
    // We only try to balance the post bind resource.
    if (HWAPostBind *OpInst = dyn_cast<HWAPostBind>(*I)) {
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

void FDLScheduler::buildASAPStep(const HWAtom *A, unsigned step) {
  if (const HWAOpInst *OI = dyn_cast<HWAOpInst>(A)) {
    unsigned ExistStep = getASAPStep(OI);

    // Schedule A to this step.
    if (A->isScheduled())
      step = A->getSlot();  

    // A is not ready at this step.
    if (ExistStep > step)
      return;

    AtomToTF[OI].first = step;
  }

  // The successor will be schedule when A is finish
  step += A->getLatency(); 

  for (HWAtom::const_use_iterator I = A->use_begin(), E = A->use_end();
      I != E; ++I)
    buildASAPStep(*I, step);
}

void FDLScheduler::buildALAPStep(const HWAtom *A, unsigned step) {
  // The step is the finish time, translate it to start time;
  step -= A->getLatency(); 

  if (const HWAOpInst *OI = dyn_cast<HWAOpInst>(A)) {
    unsigned ExistStep = getALAPStep(OI);

    // Schedule A to this step.
    if (A->isScheduled())
      step = A->getSlot();  

    // A is not ready at this step or alreay schedule to this step.
    if (ExistStep < step)
      return;

    AtomToTF[OI].second = step;
  }

  for (HWAtom::const_dep_iterator I = A->dep_begin(), E = A->dep_end();
      I != E; ++I)
    buildALAPStep(*I, step);
}

void FDLScheduler::buildTimeFrame() {
  HWAVRoot &Entry = CurStage->getEntryRoot();
  // Build the time frame
  buildASAPStep(&Entry, Entry.getSlot()); 
  HWAOpInst &Exit = CurStage->getExitRoot();
  unsigned ExitStep = getASAPStep(&Exit);
  buildALAPStep(&Exit, ExitStep);
  DEBUG(dumpTimeFrame());
}

void FDLScheduler::clear() {
  AtomToTF.clear();
  DGraph.clear();
  AvgDG.clear();
  PQueue.clear();
}

void FDLScheduler::printTimeFrame(raw_ostream &OS) const {
  OS << "Time frame:\n";
  for (usetree_iterator I = CurStage->usetree_begin(),
      E = CurStage->usetree_end(); I != E; ++I) {
    if (const HWAOpInst *A = dyn_cast<HWAOpInst>(*I)) {
      A->print(OS);
      OS << " : {" << getASAPStep(A) << "," << getALAPStep(A)
         << "} " <<  getTimeFrame(A) << "\n";
    }
  }
}

void FDLScheduler::dumpTimeFrame() const {
  printTimeFrame(dbgs());
}

void FDLScheduler::print(raw_ostream &O, const Module *M) const { }


Pass *esyn::createFDLSchedulePass() {
  return new FDLScheduler();
}
