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
  DenseMap<const Function*, uint64_t> CachedCost;
public:
  // Use extremely low threshold.
  HLSInliner() : Inliner(ID), TD(0) {
    initializeHLSInlinerPass(*PassRegistry::getPassRegistry());
  }

  static char ID; // Pass identification, replacement for typeid

  uint64_t lookupOrComputeCost(Function *F) {
    uint64_t &Cost = CachedCost[F];

    if (Cost) return Cost;

    DesignMetrics Metrics(TD);
    Metrics.visit(*F);

    // The cost increment after the function is inlined.
    Cost = Metrics.getResourceCost();
    DEBUG(dbgs() << "Inline cost of function: " << F->getName() << ':'
                 << Cost << '\n' << "Number of CallSites: " << F->getNumUses()
                 << '\n');

    Cost *= F->getNumUses() - 1;
    // Make sure we have a no-zero cost.
    Cost = std::max(UINT64_C(1), Cost);

    return Cost;
  }

  InlineCost getInlineCost(CallSite CS) {
    Function *F = CS.getCalledFunction();
    if (!F || F->isDeclaration() ||  F->hasFnAttr(Attribute::NoInline))
      return InlineCost::getNever();

    CallGraphNode *CGN = (*CG)[F];
    // Only try to inline the leaves in call graph.
    if (!CGN->empty()) return InlineCost::getNever();

    DEBUG(dbgs() << "Function: " << F->getName() << '\n');
    uint64_t IncreasedCost = lookupOrComputeCost(F);
    DEBUG(dbgs() << "Increased cost: " << IncreasedCost << ' '
                 << "Threshold: " << VFUs::MulCost[63] * 8 << '\n');
    // FIXME: Read the threshold from the constraints script.
    if (IncreasedCost < VFUs::MulCost[63] * 3) {
      DEBUG(dbgs() << "...going to inline function\n");
      return InlineCost::getAlways();
    }

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
