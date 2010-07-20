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


void ForceDirectedInfo::buildASAPStep(const HWAtom *Root, unsigned step) {
  typedef HWAtom::const_use_iterator ChildIt;
  SmallVector<std::pair<const HWAtom*, ChildIt>, 32> WorkStack;
  DenseMap<const HWAtom*, unsigned> VisitCount;
  //
  AtomToTF[Root].first = step;
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

      unsigned NewStep = ChildNode->getSlot();
      if (!ChildNode->isScheduled()) {
        int SNewStep = AtomToTF[Node].first + Node->getLatency() -
          // delta Node -> ChildNode
          Modulo * ChildNode->getDepEdge(Node)->getItDst();
        NewStep = std::max(0, SNewStep);
      }

      if (VC == 1 || AtomToTF[ChildNode].first < NewStep)
        AtomToTF[ChildNode].first = NewStep;

      // Only move forwork when we visit the node from all its deps.
      if (VC == ChildNode->getNumDeps())
        WorkStack.push_back(std::make_pair(ChildNode, ChildNode->use_begin()));
    }
  }
}

void ForceDirectedInfo::buildALAPStep(const HWAtom *Root, unsigned step) {
  typedef HWAtom::const_dep_iterator ChildIt;
  SmallVector<std::pair<const HWAtom*, ChildIt>, 32> WorkStack;
  DenseMap<const HWAtom*, unsigned> VisitCount;
  //
  AtomToTF[Root].second = step;
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

      unsigned NewStep = ChildNode->getSlot();

      if (!ChildNode->isScheduled())
        // Latency is ChildNode->Node.
        NewStep = AtomToTF[Node].second - ChildNode->getLatency()
                  + Modulo * It.getEdge()->getItDst();// delta ChildNode -> Node.    
      
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


void ForceDirectedInfo::printTimeFrame(FSMState *State, raw_ostream &OS) const {
  OS << "Time frame:\n";
  for (usetree_iterator I = State->usetree_begin(),
    E = State->usetree_end(); I != E; ++I) {
      if (const HWAOpInst *A = dyn_cast<HWAOpInst>(*I)) {
        A->print(OS);
        OS << " : {" << getASAPStep(A) << "," << getALAPStep(A)
          << "} " <<  getTimeFrame(A) << "\n";
      }
  }
}

void ForceDirectedInfo::dumpTimeFrame(FSMState *State) const {
  printTimeFrame(State, dbgs());
}

void ForceDirectedInfo::buildDGraph(FSMState *State) {
  DGraph.clear();
  for (usetree_iterator I = State->usetree_begin(),
    E = State->usetree_end(); I != E; ++I){
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
  DEBUG(printDG(State, dbgs()));
}

void ForceDirectedInfo::printDG(FSMState *State, raw_ostream &OS) const {
  // For each step
  for (unsigned ri = HWResource::FirstResourceType,
    re = HWResource::LastResourceType; ri != re; ++ri) {
      OS << "DG for resource: " << ri <<'\n';
      for (unsigned i = State->getEntryRoot().getSlot(),
        e = getALAPStep(&State->getExitRoot()) + 1; i != e; ++i)
        OS.indent(2) << "At step " << i << " : "
        << getDGraphAt(i, (enum HWResource::ResTypes)ri) << '\n';

      OS << '\n';
  }
}

double ForceDirectedInfo::getDGraphAt(unsigned step,
                                 enum HWResource::ResTypes ResType) const {
  unsigned key = (step << 4) | (0xf & ResType);
  DGType::const_iterator at = DGraph.find(key);
  
  if (at != DGraph.end()) return at->second;  

  return 0.0;
}

void ForceDirectedInfo::accDGraphAt(unsigned step,
                                    enum HWResource::ResTypes ResType,
                                    double d) {
  unsigned key = (step << 4) | (0xf & ResType);
  DGraph[key] += d;
}

double ForceDirectedInfo::getRangeDG(const HWAPostBind *A,
                                unsigned start, unsigned end/*included*/) {
  double range = end - start + 1;
  double ret = 0.0;
  for (unsigned i = start, e = end + 1; i != e; ++i)
    ret += getDGraphAt(i, A->getResClass());

  ret /= range;
  return ret;
}

double ForceDirectedInfo::computeSelfForceAt(const HWAOpInst *OpInst,
                                             unsigned step) {
  if (const HWAPostBind *A = dyn_cast<HWAPostBind>(OpInst))  
    return getDGraphAt(step, A->getResClass()) - getAvgDG(A);

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
    // We only care about the utilization of prebind resource. 
    if (HWAPostBind *A = dyn_cast<HWAPostBind>(*I)) {
      double res = 0.0;
      for (unsigned i = getASAPStep(A), e = getALAPStep(A) + 1; i != e; ++i)
        res += getDGraphAt(i, A->getResClass());

      res /= (double) getTimeFrame(A);
      AvgDG[A] = res;
    }
}


void ForceDirectedInfo::clear() {
  AtomToTF.clear();
  DGraph.clear();
  AvgDG.clear();
  Modulo = 0;
}

void ForceDirectedInfo::releaseMemory() {
  clear();
}

bool ForceDirectedInfo::runOnFunction(Function &F) {
  return false;
}

unsigned ForceDirectedInfo::buildFDInfo(FSMState *State, unsigned StartStep,
                                        unsigned MII, unsigned EndStep) {
  Modulo = MII;
  // Build the time frame
  HWAtom *Entry = &State->getEntryRoot();
  buildASAPStep(Entry, StartStep); 
  
  HWAOpInst *Exit = &State->getExitRoot();
  // Compute the EndStep if it is not given.
  if (EndStep == 0)
    EndStep = getASAPStep(Exit);
  buildALAPStep(Exit, EndStep);

  DEBUG(dumpTimeFrame(State));

  buildDGraph(State);
  buildAvgDG(State);

  return EndStep;
}