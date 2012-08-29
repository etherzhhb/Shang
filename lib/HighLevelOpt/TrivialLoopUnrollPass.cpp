//===-- TrivialLoopUnroll.cpp - Loop unroller pass ------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This pass implements a simple loop unroller.  It works best when loops have
// been canonicalized by the -indvars pass, allowing it to determine the trip
// counts of loops easily.
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "trivial-loop-unroll"
#include "vtm/Passes.h"
#include "vtm/DesignMetrics.h"
#include "vtm/FUInfo.h"

#include "llvm/IntrinsicInst.h"
#include "llvm/Transforms/Scalar.h"
#include "llvm/Analysis/LoopPass.h"
#include "llvm/Analysis/LoopIterator.h"
#include "llvm/Analysis/ScalarEvolution.h"
#include "llvm/Analysis/ScalarEvolutionExpressions.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Transforms/Utils/UnrollLoop.h"
#include "llvm/Support/CallSite.h"
#include "llvm/Target/TargetData.h"
#include <climits>

using namespace llvm;

namespace {
class TrivialLoopUnroll : public LoopPass {
public:
  static char ID; // Pass ID, replacement for typeid
  TrivialLoopUnroll() : LoopPass(ID) {
    initializeTrivialLoopUnrollPass(*PassRegistry::getPassRegistry());
  }

  bool runOnLoop(Loop *L, LPPassManager &LPM);

  /// This transformation requires natural loop information & requires that
  /// loop preheaders be inserted into the CFG...
  ///
  virtual void getAnalysisUsage(AnalysisUsage &AU) const {
    AU.addRequired<TargetData>();
    AU.addRequired<LoopInfo>();
    AU.addPreserved<LoopInfo>();
    AU.addRequiredID(LoopSimplifyID);
    AU.addPreservedID(LoopSimplifyID);
    AU.addRequiredID(LCSSAID);
    AU.addPreservedID(LCSSAID);
    AU.addRequired<ScalarEvolution>();
    AU.addPreserved<ScalarEvolution>();
    // FIXME: Loop unroll requires LCSSA. And LCSSA requires dom info.
    // If loop unroll does not preserve dom info then LCSSA pass on next
    // loop will receive invalid dom info.
    // For now, recreate dom info, if loop is unrolled.
    AU.addPreserved<DominatorTree>();
  }
};
}

char TrivialLoopUnroll::ID = 0;
INITIALIZE_PASS_BEGIN(TrivialLoopUnroll, "trivial-loop-unroll",
                      "Unroll trivial loops", false, false)
INITIALIZE_PASS_DEPENDENCY(LoopInfo)
INITIALIZE_PASS_DEPENDENCY(LoopSimplify)
INITIALIZE_PASS_DEPENDENCY(LCSSA)
INITIALIZE_PASS_DEPENDENCY(ScalarEvolution)
INITIALIZE_PASS_END(TrivialLoopUnroll, "trivial-loop-unroll",
                    "Unroll trivial loops", false, false)

Pass *llvm::createTrivialLoopUnrollPass() {
  return new TrivialLoopUnroll();
}

bool TrivialLoopUnroll::runOnLoop(Loop *L, LPPassManager &LPM) {
  // Only unroll the deepest loops in the loop nest.
  if (!L->empty()) return false;

  LoopInfo *LI = &getAnalysis<LoopInfo>();
  ScalarEvolution *SE = &getAnalysis<ScalarEvolution>();

  BasicBlock *Header = L->getHeader();
  DEBUG(dbgs() << "Loop Unroll: F[" << Header->getParent()->getName()
        << "] Loop %" << Header->getName() << "\n");
  (void)Header;

  // Find trip count and trip multiple if count is not available
  unsigned TripCount = 0;
  unsigned TripMultiple = 1;
  // Find "latch trip count". UnrollLoop assumes that control cannot exit
  // via the loop latch on any iteration prior to TripCount. The loop may exit
  // early via an earlier branch.
  BasicBlock *LatchBlock = L->getLoopLatch();
  if (LatchBlock) {
    TripCount = SE->getSmallConstantTripCount(L, LatchBlock);
    TripMultiple = SE->getSmallConstantTripMultiple(L, LatchBlock);
  }
  // Use a default unroll-count if the user doesn't specify a value
  // and the trip count is a run-time value.  The default is different
  // for run-time or compile-time trip count loops.
  // Conservative heuristic: if we know the trip count, see if we can
  // completely unroll (subject to the threshold, checked below); otherwise
  // try to find greatest modulo of the trip count which is still under
  // threshold value.
  if (TripCount == 0)
    return false;

  unsigned Count = TripCount;

  DesignMetrics Metrics(&getAnalysis<TargetData>());

  LoopBlocksDFS DFS(L);
  DFS.perform(LI);

  // Visit the blocks in top-order.
  Metrics.visit(DFS.beginRPO(), DFS.endRPO());

  unsigned NumInlineCandidates = Metrics.getNumCalls();

  if (NumInlineCandidates != 0) {
    DEBUG(dbgs() << "  Not unrolling loop with inlinable calls.\n");
    return false;
  }

  DesignMetrics::DesignCost Cost = Metrics.getCost();
  DEBUG(dbgs() << "Body cost = " << Cost << "\n");

  // FIXME: Read the threshold from the constraints script.
  unsigned Threshold = VFUs::MulCost[63] * 8;

  if (TripCount != 1 && Cost.getCostInc(Count) > Threshold) {
    DEBUG(dbgs() << "  Too large to fully unroll with count: " << Count
          << " because size: " << Cost.getCostInc(Count)
          << ">" << Threshold << "\n");
    if (TripCount) {
      // Search a feasible count by binary search.
      unsigned MaxCount = Count, MinCount = 1;

      while (MinCount <= MaxCount) {
        unsigned MidCount = MinCount + (MaxCount - MinCount) / 2;

        if (Cost.getCostInc(MidCount) <= Threshold) {
          // MidCount is ok, try a bigger one.
          Count = MidCount;
          MinCount = MidCount + 1;
        } else
          // Else we had to try a smaller count.
          MaxCount = MidCount - 1;
      }

      // Reduce unroll count to be modulo of TripCount for partial unrolling
      while (Count != 0 && TripCount % Count != 0)
        --Count;
    }

    if (Count < 2) {
      DEBUG(dbgs() << "  could not unroll partially\n");
      return false;
    }
    DEBUG(dbgs() << "  partially unrolling with count: " << Count << "\n");
  }

  // Unroll the loop.
  if (!UnrollLoop(L, Count, TripCount, false, TripMultiple, LI, &LPM))
    return false;

  return true;
}
