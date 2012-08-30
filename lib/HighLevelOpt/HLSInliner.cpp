//===--- HSLInliner.cpp ---Perform HLS specific function inlining ---------===//
//
// Copyright: 2011 by SYSU EDA Group. all rights reserved.
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
//
//===----------------------------------------------------------------------===//

#include "vtm/DesignMetrics.h"
#include "vtm/FUInfo.h"
#include "vtm/Passes.h"

#include "llvm/Analysis/CallGraph.h"
#include "llvm/Analysis/InlineCost.h"
#include "llvm/Support/CallSite.h"
#include "llvm/Transforms/IPO/InlinerPass.h"
#include "llvm/ADT/SmallPtrSet.h"
#include "llvm/Target/TargetData.h"
#include "llvm/Support/raw_ostream.h"
#define DEBUG_TYPE "vtm-inliner"
#include "llvm/Support/Debug.h"

using namespace llvm;

namespace {

// AlwaysInliner only inlines functions that are mark as "always inline".
class HLSInliner : public Inliner {
  InlineCostAnalyzer CA;
  TargetData *TD;
  CallGraph *CG;
  DenseMap<const Function*, DesignMetrics::DesignCost> CachedCost;
public:
  // Use extremely low threshold.
  HLSInliner() : Inliner(ID), TD(0) {
    initializeHLSInlinerPass(*PassRegistry::getPassRegistry());
  }

  static char ID; // Pass identification, replacement for typeid

  DesignMetrics::DesignCost lookupOrComputeCost(Function *F) {
    DesignMetrics::DesignCost &Cost = CachedCost[F];

    if (Cost) return Cost;

    DesignMetrics Metrics(TD);
    Metrics.visit(*F);

    // The cost increment after the function is inlined.
    // Ignore the 2 steps for submodule launching and returing.
    Cost = Metrics.getCost(2);
    DEBUG(dbgs() << "Inline cost of function: " << F->getName() << ':'
                 << Cost << '\n' << "Number of CallSites: " << F->getNumUses()
                 << '\n');

    return Cost;
  }

  InlineCost getInlineCost(CallSite CS) {
    Function *F = CS.getCalledFunction(), *ParentF = CS.getCalledFunction();
    if (!F || F->isDeclaration() ||  F->hasFnAttr(Attribute::NoInline))
      return InlineCost::getNever();

    CallGraphNode *CGN = (*CG)[F];
    // Only try to inline the leaves in call graph.
    if (!CGN->empty()) return InlineCost::getNever();

    unsigned NumUses = 0;
    typedef Instruction::use_iterator use_iterator;
    for (use_iterator I = F->use_begin(), E = F->use_end(); I != E; ++I) {
      CallSite CS(*I);

      if (CS.getInstruction() && CS.getCalledFunction() == ParentF) ++NumUses;
    }

    DEBUG(dbgs() << "Function: " << F->getName() << '\n');
    DesignMetrics::DesignCost Cost = lookupOrComputeCost(F);
    uint64_t Threshold = VFUs::MulCost[63] * 4,
             IncreasedCost = Cost.getCostInc(NumUses);

    DEBUG(dbgs() << "Cost: " << Cost << ' '
                 << "Increased cost: " << IncreasedCost << ' '
                 << "Threshold: " << Threshold);
    // FIXME: Read the threshold from the constraints script.
    if (IncreasedCost < Threshold) {
      DEBUG(dbgs() << "...going to inline function\n");
      return InlineCost::getAlways();
    }

    DEBUG(dbgs() << "...not inline function\n");
    return InlineCost::getNever();
  }

  bool doInitialization(CallGraph &CG);

  void releaseMemory() { CachedCost.clear(); }

  void getAnalysisUsage(AnalysisUsage &Info) const {
    Info.addRequired<TargetData>();
    Inliner::getAnalysisUsage(Info);
  }
};
}

char HLSInliner::ID = 0;
INITIALIZE_PASS_BEGIN(HLSInliner, "hls-inline",
                "Inliner HLS specific algorithm", false, false)
INITIALIZE_AG_DEPENDENCY(CallGraph)
INITIALIZE_PASS_END(HLSInliner, "hls-inline",
                "Inliner HLS specific algorithm", false, false)

Pass *llvm::createHLSInlinerPass() {
  return new HLSInliner();
}

// doInitialization - Initializes the vector of functions that have not
// been annotated with the "always inline" attribute.
bool HLSInliner::doInitialization(CallGraph &CG) {
  this->CG = &CG;
  TD = getAnalysisIfAvailable<TargetData>();
  assert(TD && "Cannot initialize target data!");
  return false;
}
