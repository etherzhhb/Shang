//===--- HSLInliner.cpp ---Perform HLS specific function inlining ---------===//
//
//                      The Shang HLS frameowrk                               //
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
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
  DenseMap<const Function*, DesignMetrics::DesignCost> CachedCost;
public:
  // Use extremely low threshold.
  HLSInliner() : Inliner(ID) {
    initializeHLSInlinerPass(*PassRegistry::getPassRegistry());
  }

  static char ID; // Pass identification, replacement for typeid

  DesignMetrics::DesignCost lookupOrComputeCost(Function *F) {
    DesignMetrics::DesignCost &Cost = CachedCost[F];

    if (Cost) return Cost;

    DesignMetrics Metrics(&getAnalysis<TargetData>());
    Metrics.visit(*F);

    Cost = Metrics.getCost();
    DEBUG(dbgs() << "Inline cost of function: " << F->getName() << ':'
                 << Cost << '\n' << "Number of CallSites: " << F->getNumUses()
                 << '\n');

    return Cost;
  }

  InlineCost getInlineCost(CallSite CS) {
    Function *F = CS.getCalledFunction(), *CallerF = CS.getCaller();
    if (!F || F->isDeclaration() ||  F->hasFnAttr(Attribute::NoInline))
      return InlineCost::getNever();

    unsigned NumUses = 0;
    typedef Instruction::use_iterator use_iterator;
    for (use_iterator I = F->use_begin(), E = F->use_end(); I != E; ++I) {
      CallSite CurCS(*I);

      if (!CurCS.getInstruction() || !CurCS.getCaller()) continue;

      Function *CurCaller = CurCS.getCaller();

      // Don't try to inline F if all its caller function are not visited, but
      // please not that CurCaller will never be visited if it do not have any
      // user.
      if (!CachedCost.count(CurCaller) && CurCaller->getNumUses() != 0)
        return InlineCost::getNever();

      if (CurCaller == CallerF) ++NumUses;
    }

    DEBUG(dbgs() << "Function: " << F->getName() << '\n');
    DesignMetrics::DesignCost Cost = lookupOrComputeCost(F);
    uint64_t Threshold = 64000;
    uint64_t IncreasedCost = Cost.getCostInc(NumUses, 1, 8, 0) * Cost.StepLB;

    DEBUG(dbgs() << "Cost: " << Cost << '\n'
                 << "Increased cost: " << IncreasedCost << ' '
                 << "IncThreshold: " << Threshold << '\n');
    // FIXME: Read the threshold from the constraints script.
    if (IncreasedCost < Threshold) {
      DEBUG(dbgs() << "...going to inline function\n");
      // The cost of ParentF is changed.
      CachedCost.erase(CallerF);
      return InlineCost::getAlways();
    }

    DEBUG(dbgs() << "...not inline function\n");
    return InlineCost::getNever();
  }

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
