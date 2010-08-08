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


void ForceDirectedInfo::buildASAPStep(const HWAVRoot *EntryRoot,
                                      unsigned step) {
  typedef HWAtom::const_use_iterator ChildIt;
  SmallVector<std::pair<const HWAtom*, ChildIt>, 32> WorkStack;
  DenseMap<const HWAtom*, unsigned> VisitCount;
  //
  AtomToTF[EntryRoot].first = step;
  WorkStack.push_back(std::make_pair(EntryRoot, EntryRoot->use_begin()));
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

      unsigned NewStep = ChildNode->getSlot();
      if (!ChildNode->isScheduled()) {
        int SNewStep = AtomToTF[Node].first + Node->getLatency() -
          // delta Node -> ChildNode
          MII * ChildNode->getDepEdge(Node)->getItDst();
        NewStep = std::max(0, SNewStep);
      }

      if (VC == 1 || AtomToTF[ChildNode].first < NewStep)
        AtomToTF[ChildNode].first = NewStep;

      // Only move forward when we visit the node from all its deps.
      if (VC == ChildNode->getNumDeps())
        WorkStack.push_back(std::make_pair(ChildNode, ChildNode->use_begin()));
    }
  }
}

void ForceDirectedInfo::buildALAPStep(const HWAOpInst *ExitRoot,
                                      unsigned step) {
  typedef HWAtom::const_dep_iterator ChildIt;
  SmallVector<std::pair<const HWAtom*, ChildIt>, 32> WorkStack;
  DenseMap<const HWAtom*, unsigned> VisitCount;
  //
  AtomToTF[ExitRoot].second = step;
  WorkStack.push_back(std::make_pair(ExitRoot, ExitRoot->dep_begin()));
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

      unsigned NewStep = ChildNode->getSlot();

      if (!ChildNode->isScheduled()) {
        // Latency is ChildNode->Node.
        NewStep = AtomToTF[Node].second - ChildNode->getLatency()
                  + MII * It.getEdge()->getItDst();// delta ChildNode -> Node.
        // Consider the SCC ALAP constrain.
        NewStep = std::min(NewStep, getSCCALAPStep(ChildNode));
      }

      if (VC == 1 || AtomToTF[ChildNode].second > NewStep)
        AtomToTF[ChildNode].second = NewStep;

      DEBUG(dbgs() << "Visit " << "\n");
      DEBUG(ChildNode->dump());
      DEBUG(dbgs() << "VC: " << VC << " total use: "
        << ChildNode->getNumUses() << '\n');

      // Only move forward when we visit the node from all its deps.
      if (VC == ChildNode->getNumUses()) {
        assert(getALAPStep(ChildNode) >= getASAPStep(ChildNode)
               && "Broken time frame!");
        WorkStack.push_back(std::make_pair(ChildNode, ChildNode->dep_begin()));
      }
    }
  }
}


void ForceDirectedInfo::printTimeFrame(FSMState *State, raw_ostream &OS) const {
  OS << "Time frame:\n";
  for (usetree_iterator I = State->usetree_begin(),
      E = State->usetree_end(); I != E; ++I) {
    HWAtom *A = *I;
    A->print(OS);
    OS << " : {" << getASAPStep(A) << "," << getALAPStep(A)
      << "} " <<  getTimeFrame(A) << "\n";
  }
}

void ForceDirectedInfo::dumpTimeFrame(FSMState *State) const {
  printTimeFrame(State, dbgs());
}

bool ForceDirectedInfo::isFUAvailalbe(unsigned step, HWFUnit FU) const {
  unsigned key = computeStepKey(step, FU.getFUnitID());
  unsigned usage = const_cast<ForceDirectedInfo*>(this)->ResUsage[key];
  return usage < FU.getTotalFUs();
}

void ForceDirectedInfo::buildDGraph(FSMState *State) {
  DGraph.clear();
  for (usetree_iterator I = State->usetree_begin(),
    E = State->usetree_end(); I != E; ++I){
      // We only try to balance the post bind resource.
      if (HWAOpInst *OpInst = dyn_cast<HWAOpInst>(*I)) {
        if (OpInst->getResClass() == HWResource::Trivial)
          continue;

        unsigned TimeFrame = getTimeFrame(OpInst);
        unsigned ASAPStep = getASAPStep(OpInst);
        // Remember the Function unit usage of the scheduled instruction.
        if (TimeFrame == 1)
          ++ResUsage[computeStepKey(ASAPStep, OpInst->getFunUnitID())];



        double Prob = 1.0 / (double) TimeFrame;
        // Including ALAPStep.
        for (unsigned i = ASAPStep, e = getALAPStep(OpInst) + 1;
            i != e; ++i)
          accDGraphAt(i, OpInst->getFunUnitID(), Prob);
      }
  }
  DEBUG(printDG(State, dbgs()));
}

void ForceDirectedInfo::printDG(FSMState *State, raw_ostream &OS) const {
  unsigned StartStep = State->getEntryRoot().getSlot();
  unsigned EndStep = MII > 0 ? (StartStep + MII - 1)
                                  : getALAPStep(&State->getExitRoot());
  // For each step
  for (unsigned ri = HWResource::FirstResourceType,
    re = HWResource::LastResourceType; ri != re; ++ri) {
      OS << "DG for resource: " << ri <<'\n';
      for (unsigned i = StartStep,
        e = EndStep + 1; i != e; ++i)
        OS.indent(2) << "At step " << i << " : "
        << getDGraphAt(i, (enum HWResource::ResTypes)ri) << '\n';

      OS << '\n';
  }
}

unsigned
ForceDirectedInfo::computeStepKey(unsigned step, HWFUnitID FUID) const {
  if (MII != 0)
    step = step % MII;
  
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

double ForceDirectedInfo::getRangeDG(const HWAPostBind *A,
                                unsigned start, unsigned end/*included*/) {
  double range = end - start + 1;
  double ret = 0.0;
  for (unsigned i = start, e = end + 1; i != e; ++i)
    ret += getDGraphAt(i, A->getFunUnitID());

  ret /= range;
  return ret;
}

double ForceDirectedInfo::computeSelfForceAt(const HWAOpInst *OpInst,
                                             unsigned step) {
  if (const HWAPostBind *A = dyn_cast<HWAPostBind>(OpInst))  
    return getDGraphAt(step, A->getFunUnitID()) - getAvgDG(A);

  // The force about the pre-bind resoure dose not matter.
  return 0.0;
}

double ForceDirectedInfo::computeSuccForceAt(const HWAOpInst *OpInst,
                                             unsigned step) {
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

double ForceDirectedInfo::computePredForceAt(const HWAOpInst *OpInst,
                                             unsigned step) {
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

void ForceDirectedInfo::buildAvgDG(FSMState *State) {
  for (usetree_iterator I = State->usetree_begin(),
      E = State->usetree_end(); I != E; ++I)
    // We only care about the utilization of post bind resource. 
    if (HWAPostBind *A = dyn_cast<HWAPostBind>(*I)) {
      double res = 0.0;
      for (unsigned i = getASAPStep(A), e = getALAPStep(A) + 1; i != e; ++i)
        res += getDGraphAt(i, A->getFunUnitID());

      res /= (double) getTimeFrame(A);
      AvgDG[A] = res;
    }
}

void ForceDirectedInfo::clear() {
  AtomToTF.clear();
  AtomToSCCALAP.clear();
  DGraph.clear();
  ResUsage.clear();
  AvgDG.clear();
  MII = 0;
  CriticalPathEnd = 0;
}

void ForceDirectedInfo::releaseMemory() {
  clear();
}

bool ForceDirectedInfo::runOnFunction(Function &F) {
  return false;
}

unsigned ForceDirectedInfo::buildFDInfo(FSMState *State, unsigned StartStep,
                                        unsigned EndStep) {
  // Build the time frame
  HWAVRoot *Entry = &State->getEntryRoot();
  buildASAPStep(Entry, StartStep); 
  
  HWAOpInst *Exit = &State->getExitRoot();
  if (EndStep == 0)
    EndStep = getASAPStep(Exit);
  CriticalPathEnd = EndStep;

  // Dirty Hack: Just place all SCC at the first II.
  // TODO: Move the whole SCC around and find the best place.
  if (MII)
    buildSCCASAPStep(State, CriticalPathEnd, MII,
                     Entry->use_begin(), Entry->use_end());

  buildALAPStep(Exit, CriticalPathEnd);

  DEBUG(dumpTimeFrame(State));

  buildDGraph(State);
  buildAvgDG(State);

  return CriticalPathEnd;
}

void ForceDirectedInfo::recoverFDInfo(FSMState *State) {
  // Build the time frame
  buildASAPStep(State); 
  buildALAPStep(State);

  DEBUG(dumpTimeFrame(State));

  buildDGraph(State);
  buildAvgDG(State);
}

void ForceDirectedInfo::dumpDG(FSMState *State) const {
  printDG(State, dbgs());
}
