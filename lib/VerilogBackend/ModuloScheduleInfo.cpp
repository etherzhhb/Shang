//===- ModuloScheduleInfo.cpp - ModuleSchedule information analyze -*- C++ -*-===//
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
// This file implement the ModuleSchedule information computation pass describe
// in
// Josep, L. (1996). Swing Modulo Scheduling: A Lifetime-Sensitive Approach.
//
//===----------------------------------------------------------------------===//

#include "ModuloScheduleInfo.h"
#include "HWAtomPasses.h"

#include "llvm/Analysis/LoopInfo.h"
#include "llvm/ADT/SCCIterator.h"
#include "llvm/Support/CommandLine.h"
#define DEBUG_TYPE "vbe-ms-info"
#include "llvm/Support/Debug.h"

using namespace llvm;
using namespace esyn;


static cl::opt<bool>
NoModuloSchedule("disable-modulo-schedule",
          cl::desc("vbe - Do not preform modulo schedule"),
          cl::Hidden, cl::init(false));

//===----------------------------------------------------------------------===//
typedef GraphTraits<Inverse<esyn::HWAtom*> > HWAtomSccGT;
typedef GraphTraits<Inverse<const esyn::HWAtom*> > ConstHWAtomSccGT;

typedef scc_iterator<HWAtom*, HWAtomSccGT> dep_scc_iterator;
typedef scc_iterator<const HWAtom*, ConstHWAtomSccGT> const_dep_scc_iterator;

//static Self findSccEdge(NodeType *Src, NodeType *Dst) {
//  return std::find(Self(Dst->edge_begin()),
//    Self(Dst->edge_end()),
//    Src);
//}
//===----------------------------------------------------------------------===//
char ModuloScheduleInfo::ID = 0;

RegisterPass<ModuloScheduleInfo> X("vbe-ms-info",
                                  "vbe - Compute necessary information for modulo"
                                  " schedule scheduling passes");

void ModuloScheduleInfo::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequiredTransitive<HWAtomInfo>();
  AU.addRequiredTransitive<ResourceConfig>();
  AU.addRequired<LoopInfo>();
  AU.setPreservesAll();
}

bool ModuloScheduleInfo::runOnFunction(Function &F) {
  HI = &getAnalysis<HWAtomInfo>();
  LI = &getAnalysis<LoopInfo>();
  RC = &getAnalysis<ResourceConfig>();
  return false;
}

bool ModuloScheduleInfo::isModuloSchedulable(FSMState &State) const {
  // Are we disable modulo schedule?
  if (NoModuloSchedule) return false;
  
  BasicBlock *BB = State.getBasicBlock();
  Loop *L = LI->getLoopFor(BB);
  // States that not in loops are not MSable.
  if (!L) return false;

  // Only schedule single block loops.
  if (L->getBlocks().size() > 1)
    return false;
  
  return true;
}

unsigned ModuloScheduleInfo::computeRecII(scc_vector &Scc) {
  assert(Scc.size() > 1 && "No self loop expect in DDG!");
  unsigned totalLatency = 0;
  unsigned totalItDist = 0;
  HWAtom *LastAtom = 0;
  // Make a circle.
  Scc.push_back(Scc.front());
  for (scc_vector::const_iterator I = Scc.begin(), E = Scc.end();
    I != E; ++I) {
      HWAtom *CurAtom = *I;
      totalLatency += CurAtom->getLatency();

      if (LastAtom) {
        HWAtomSccGT::ChildIteratorType EI =
          HWAtomSccGT::ChildIteratorType::findSccEdge(LastAtom, CurAtom);
        assert(EI != HWAtomSccGT::child_end(LastAtom) && "Edge not found!");
        if (HWMemDep *MemEdge = dyn_cast<HWMemDep>(EI.getEdge())) {
          unsigned ItDst = MemEdge->getItDst();
          if (ItDst > 0) {
            totalItDist = ItDst;
          }
        }
      }
      // Update last atom we had visited.
      LastAtom = CurAtom;
  }
  // The latency of the first node had been count twice.
  totalLatency -= Scc.front()->getLatency();
  // Recover the Scc vector.
  Scc.pop_back();
  assert(totalItDist != 0 && "No cross iteration dependence?");
  return ceil((double)totalLatency / totalItDist);
}


unsigned ModuloScheduleInfo::computeRecMII(FSMState &State) {
  HWAtom *Root = &State.getExitRoot();
  unsigned MaxRecII = 1;
  for (dep_scc_iterator SCCI = dep_scc_iterator::begin(Root),
      SCCE = dep_scc_iterator::end(Root); SCCI != SCCE; ++SCCI) {
    scc_vector &Atoms = *SCCI;
    if (Atoms.size() == 1) {
      assert(!SCCI.hasLoop() && "No self loop expect in DDG!");
      TrivialNodes.push_back(Atoms[0]);
      continue;
    }

    DEBUG(dbgs() << "SCC found:\n");
    DEBUG(
      for (scc_vector::const_iterator I = Atoms.begin(), E = Atoms.end();
          I != E; ++I)
        (*I)->getValue().dump();
    );
    unsigned RecII = computeRecII(Atoms);
    // Update maxrecii.
    MaxRecII = std::max(RecII, MaxRecII);
    RecList.insert(std::make_pair(RecII, Atoms));
    DEBUG(dbgs() << "RecII: " << RecII << '\n');
  }

  DEBUG(dbgs() << "RecMII: " << MaxRecII << '\n');
  return MaxRecII;
}

unsigned ModuloScheduleInfo::computeResMII(FSMState &State) const {
  std::map<HWResource::ResTypes, unsigned> TotalResUsage;
  for (usetree_iterator I = State.usetree_begin(), E = State.usetree_end();
    I != E; ++I)
    if (HWAOpInst *A = dyn_cast<HWAOpInst>(*I))
      ++TotalResUsage[A->getResClass()];

  unsigned MaxResII = 0;
  typedef std::map<HWResource::ResTypes, unsigned>::iterator UsageIt;
  for (UsageIt I = TotalResUsage.begin(), E = TotalResUsage.end(); I != E; ++I){
    if (I->first != HWResource::Trivial)
      MaxResII = std::max(MaxResII,
                          I->second / RC->getResource(I->first)->getTotalRes());
  }
  DEBUG(dbgs() << "ResMII: " << MaxResII << '\n');
  return MaxResII;
}

ModuloScheduleInfo::~ModuloScheduleInfo() {
  clear();
}

void ModuloScheduleInfo::clear() {
  RecList.clear();
}
