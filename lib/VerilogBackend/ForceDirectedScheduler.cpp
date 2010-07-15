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

#define DEBUG_TYPE "vbe-fd-sched"
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

struct FDLScheduler : public BasicBlockPass {
  HWAtomInfo *HI;
  ResourceConfig *RC;

  FSMState *CurStage;

  // Time Frame {asap step, alap step }
  typedef std::pair<unsigned, unsigned> TimeFrame;
  // Mapping hardware atoms to time frames.
  typedef std::map<const HWAtom*, TimeFrame> TimeFrameMapType;

  /// @name TimeFrame
  //{
  TimeFrameMapType AtomToTF;
  unsigned CriticalPathLength;
  void buildASAPStep();
  unsigned getASAPStep(const HWAOpInst *A) const {
    return const_cast<FDLScheduler*>(this)->AtomToTF[A].first;
  }


  void buildALAPStep();
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

  unsigned findBestStep(HWAOpInst *A);
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
    : BasicBlockPass(&ID), HI(0), RC(0), PQueue(fds_sort(this)) {}
  bool runOnBasicBlock(BasicBlock &BB);
  void releaseMemory();
  void getAnalysisUsage(AnalysisUsage &AU) const;
  virtual void print(raw_ostream &O, const Module *M) const;
  //}
};
} //end namespace

//===----------------------------------------------------------------------===//
bool fds_sort::operator()(const HWAOpInst* LHS, const HWAOpInst* RHS) const {
  // Schedule the low mobility nodes first.
  if (FDS->getTimeFrame(LHS) > FDS->getTimeFrame(RHS))
    return true; // Place RHS first.
  
  //unsigned LHSLatency = FDS->getASAPStep(LHS);
  //unsigned RHSLatency = FDS->getASAPStep(RHS);
  //// Schedule as soon as possible?
  //if (LHSLatency < RHSLatency) return true;
  //if (LHSLatency > RHSLatency) return false;

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


  // Build the time frame
  HWAVRoot &Entry = CurStage->getEntryRoot();
  Entry.scheduledTo(HI->getTotalCycle());

  buildASAPStep(); 
  HWAOpInst &Exit = CurStage->getExitRoot();
  CriticalPathLength = getASAPStep(&Exit);
  buildALAPStep();
  DEBUG(dumpTimeFrame());

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
      step = findBestStep(A);
      A->scheduledTo(step);
      // Recover the time frame by force rebuild
      buildASAPStep(); 
      buildALAPStep();

      DEBUG(dbgs() << " After schedule:-------------------\n");
      DEBUG(dumpTimeFrame());
      DEBUG(dbgs() << "\n\n\n");
      // Rebuild DG.
      buildDGraph();
      buildAvgDG();
      PQueue.reheapify();
    } else {
      // Schedule to the best step.
      A->scheduledTo(step);
    }
  }

  HI->setTotalCycle(CurStage->getExitRoot().getSlot() + 1);

  return false;
}


unsigned FDLScheduler::findBestStep(HWAOpInst *A) {
  std::pair<unsigned, double> BestStep = std::make_pair(0, 1e32);
  // For each possible step:
  for (unsigned i = getASAPStep(A), e = getALAPStep(A) + 1; i != e; ++i) {
    // Compute the forces.
    DEBUG(dbgs() << "At Step " << i);
    double SelfForce = computeSelfForceAt(A, i);
    // Force update time frame
    A->scheduledTo(i);
    // Recover the time frame by force rebuild
    buildASAPStep(); 
    buildALAPStep();

    // The follow function will invalid the time frame.
    DEBUG(dbgs() << " Self Force: " << SelfForce);
    double PredForce = computePredForceAt(A, i);
    DEBUG(dbgs() << " Pred Force: " << PredForce);
    double SuccForce = computeSuccForceAt(A, i);
    DEBUG(dbgs() << " Succ Force: " << SuccForce);
    double Force = SelfForce + PredForce + SuccForce;
    if (Force < BestStep.second)
      BestStep = std::make_pair(i, Force);

    DEBUG(dbgs() << '\n');
  }
  return BestStep.first;
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
        e = getALAPStep(&CurStage->getExitRoot()) + 1; i != e; ++i)
      OS.indent(2) << "At step " << i << " : "
        << getDGraphAt(i, (enum HWResource::ResTypes)ri) << '\n';

    OS << '\n';
  }
}

void FDLScheduler::buildASAPStep() {
  const HWAVRoot *Root = &CurStage->getEntryRoot();

  typedef HWAtom::const_use_iterator ChildIt;
  SmallVector<std::pair<const HWAtom*, ChildIt>, 32> WorkStack;
  DenseMap<const HWAtom*, unsigned> VisitCount;
  //
  AtomToTF[Root].first = Root->getSlot();
  WorkStack.push_back(std::make_pair(Root, Root->use_begin()));
  //
  while (!WorkStack.empty()) {
    const HWAtom *Node = WorkStack.back().first;
    ChildIt It = WorkStack.back().second;

    if (It == Node->use_end())
      WorkStack.pop_back();
    else {
      const HWAtom *ChildNode = *It;
      ++WorkStack.back().second;
      unsigned VC = ++VisitCount[ChildNode];

      unsigned NewStep = AtomToTF[Node].first + Node->getLatency();
      if (ChildNode->isScheduled())
        NewStep = ChildNode->getSlot();

      if (VC == 1 || AtomToTF[ChildNode].first < NewStep)
        AtomToTF[ChildNode].first = NewStep;
      
      // Only move forwork when we visit the node from all its deps.
      if (VC == ChildNode->getNumDeps())
        WorkStack.push_back(std::make_pair(ChildNode, ChildNode->use_begin()));
    }
  }
}

void FDLScheduler::buildALAPStep() {
  const HWAOpInst *Root = &CurStage->getExitRoot();

  typedef HWAtom::const_dep_iterator ChildIt;
  SmallVector<std::pair<const HWAtom*, ChildIt>, 32> WorkStack;
  DenseMap<const HWAtom*, unsigned> VisitCount;
  //
  AtomToTF[Root].second = CriticalPathLength;
  WorkStack.push_back(std::make_pair(Root, Root->dep_begin()));
  //
  while (!WorkStack.empty()) {
    const HWAtom *Node = WorkStack.back().first;
    ChildIt It = WorkStack.back().second;

    if (It == Node->dep_end())
      WorkStack.pop_back();
    else {
      const HWAtom *ChildNode = *It;
      ++WorkStack.back().second;
      unsigned VC = ++VisitCount[ChildNode];

      unsigned NewStep = AtomToTF[Node].second - ChildNode->getLatency();
      if (ChildNode->isScheduled())
        NewStep = ChildNode->getSlot();

      if (VC == 1 || AtomToTF[ChildNode].second > NewStep)
        AtomToTF[ChildNode].second = NewStep;

      DEBUG(dbgs() << "Visit " << "\n");
      DEBUG(ChildNode->dump());
      DEBUG(dbgs() << "VC: " << VC << " total use: "
                   << ChildNode->getNumUses() << '\n');

      // Only move forwork when we visit the node from all its deps.
      if (VC == ChildNode->getNumUses())
        WorkStack.push_back(std::make_pair(ChildNode, ChildNode->dep_begin()));
    }
  }
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
