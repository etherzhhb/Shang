//===-- HLSLoopUnroll.cpp - Loop unroller pass -------------------------------===//
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

#define DEBUG_TYPE "loop-unroll"
#include "vtm/Passes.h"

#include "llvm/IntrinsicInst.h"
#include "llvm/Transforms/Scalar.h"
#include "llvm/Analysis/LoopPass.h"
#include "llvm/Analysis/CodeMetrics.h"
#include "llvm/Analysis/ScalarEvolution.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Transforms/Utils/UnrollLoop.h"
#include "llvm/Support/CallSite.h"
#include "llvm/Target/TargetData.h"
#include <climits>

using namespace llvm;

namespace {
  class HLSLoopUnroll : public LoopPass {
  public:
    static char ID; // Pass ID, replacement for typeid
    HLSLoopUnroll(int T = -1, int C = -1) : LoopPass(ID) {
      CurrentThreshold = (T == -1) ? 150 : unsigned(T);
      CurrentCount = (C == -1) ? 0 : unsigned(C);
      initializeHLSLoopUnrollPass(*PassRegistry::getPassRegistry());
    }

    /// A magic value for use with the Threshold parameter to indicate
    /// that the loop unroll should be performed regardless of how much
    /// code expansion would result.
    static const unsigned NoThreshold = UINT_MAX;

    // Threshold to use when optsize is specified (and there is no
    // explicit -unroll-threshold).
    static const unsigned OptSizeUnrollThreshold = 50;

    // Default unroll count for loops with run-time trip count if
    // -unroll-count is not set
    static const unsigned UnrollRuntimeCount = 8;

    unsigned CurrentCount;
    unsigned CurrentThreshold;

    bool runOnLoop(Loop *L, LPPassManager &LPM);

    /// This transformation requires natural loop information & requires that
    /// loop preheaders be inserted into the CFG...
    ///
    virtual void getAnalysisUsage(AnalysisUsage &AU) const {
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

char HLSLoopUnroll::ID = 0;
INITIALIZE_PASS_BEGIN(HLSLoopUnroll, "loop-unroll", "Unroll loops", false, false)
INITIALIZE_PASS_DEPENDENCY(LoopInfo)
INITIALIZE_PASS_DEPENDENCY(LoopSimplify)
INITIALIZE_PASS_DEPENDENCY(LCSSA)
INITIALIZE_PASS_DEPENDENCY(ScalarEvolution)
INITIALIZE_PASS_END(HLSLoopUnroll, "loop-unroll", "Unroll loops", false, false)

Pass *llvm::createHLSLoopUnrollPass() {
  return new HLSLoopUnroll();
}

namespace {
struct HLSCodeMetrics : public CodeMetrics {
  /// analyzeBasicBlock - Fill in the current structure with information gleaned
  /// from the specified block.
  void analyzeBasicBlock(const BasicBlock *BB, const TargetData *TD,
                         ScalarEvolution *SE) {
    ++NumBlocks;
    unsigned NumInstsBeforeThisBB = NumInsts;
    for (BasicBlock::const_iterator II = BB->begin(), E = BB->end();
         II != E; ++II) {
      const Instruction *I = II;

      if (isa<PHINode>(I)) continue;           // PHI nodes don't count.      

      // Special handling for calls.
      switch (I->getOpcode()) {
      case Instruction::And:
      case Instruction::Xor:
      case Instruction::Or:
        continue;
      case Instruction::Call:
      case Instruction::Invoke:
        visitCallInst(I, BB);
        continue;
      }

      if (const AllocaInst *AI = dyn_cast<AllocaInst>(I)) {
        if (!AI->isStaticAlloca())
          this->usesDynamicAlloca = true;
      }

      if (isa<ExtractElementInst>(II) || II->getType()->isVectorTy())
        ++NumVectorInsts;

      // Zero cost instructions.
      if (I->isCast() || (I->isBinaryOp() && isa<Constant>(I->getOperand(1)))
          || I->isTerminator())
        continue;

      ++NumInsts;
    }

    if (isa<ReturnInst>(BB->getTerminator()))
      ++NumRets;

    // We never want to inline functions that contain an indirectbr.  This is
    // incorrect because all the blockaddress's (in static global initializers
    // for example) would be referring to the original function, and this indirect
    // jump would jump from the inlined copy of the function into the original
    // function which is extremely undefined behavior.
    // FIXME: This logic isn't really right; we can safely inline functions
    // with indirectbr's as long as no other function or global references the
    // blockaddress of a block within the current function.  And as a QOI issue,
    // if someone is using a blockaddress without an indirectbr, and that
    // reference somehow ends up in another function or global, we probably
    // don't want to inline this function.
    if (isa<IndirectBrInst>(BB->getTerminator()))
      containsIndirectBr = true;

    // Remember NumInsts for this BB.
    NumBBInsts[BB] = NumInsts - NumInstsBeforeThisBB;
  }

  void visitCallInst(const Instruction *I, const BasicBlock * BB ) {
    if (const IntrinsicInst *IntrinsicI = dyn_cast<IntrinsicInst>(I)) {
      switch (IntrinsicI->getIntrinsicID()) {
      default: break;
      case Intrinsic::dbg_declare:
      case Intrinsic::dbg_value:
      case Intrinsic::invariant_start:
      case Intrinsic::invariant_end:
      case Intrinsic::lifetime_start:
      case Intrinsic::lifetime_end:
      case Intrinsic::objectsize:
      case Intrinsic::ptr_annotation:
      case Intrinsic::var_annotation:
        // These intrinsics don't count as size.
        return;
      }
    }

    ImmutableCallSite CS(cast<Instruction>(I));

    if (const Function *F = CS.getCalledFunction()) {
      // If a function is both internal and has a single use, then it is
      // extremely likely to get inlined in the future (it was probably
      // exposed by an interleaved devirtualization pass).
      if (!CS.isNoInline() && F->hasInternalLinkage() && F->hasOneUse())
        ++NumInlineCandidates;

      // If this call is to function itself, then the function is recursive.
      // Inlining it into other functions is a bad idea, because this is
      // basically just a form of loop peeling, and our metrics aren't useful
      // for that case.
      if (F == BB->getParent())
        isRecursive = true;
    }
  }
};
}

bool HLSLoopUnroll::runOnLoop(Loop *L, LPPassManager &LPM) {
  LoopInfo *LI = &getAnalysis<LoopInfo>();
  ScalarEvolution *SE = &getAnalysis<ScalarEvolution>();

  BasicBlock *Header = L->getHeader();
  DEBUG(dbgs() << "Loop Unroll: F[" << Header->getParent()->getName()
        << "] Loop %" << Header->getName() << "\n");
  (void)Header;

  // Determine the current unrolling threshold.  While this is normally set
  // from UnrollThreshold, it is overridden to a smaller value if the current
  // function is marked as optimize-for-size, and the unroll threshold was
  // not user specified.
  unsigned Threshold = 16;

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
  unsigned Count = CurrentCount;

  if (Count == 0) {
    // Conservative heuristic: if we know the trip count, see if we can
    // completely unroll (subject to the threshold, checked below); otherwise
    // try to find greatest modulo of the trip count which is still under
    // threshold value.
    if (TripCount == 0)
      return false;
    Count = TripCount;
  }

  // Compute the loop size
  const TargetData *TD = getAnalysisIfAvailable<TargetData>();

  HLSCodeMetrics Metrics;
  for (Loop::block_iterator I = L->block_begin(), E = L->block_end();
    I != E; ++I)
    Metrics.analyzeBasicBlock(*I, TD, SE);
  unsigned NumInlineCandidates = Metrics.NumInlineCandidates;

  // Don't allow an estimate of size zero.  This would allows unrolling of loops
  // with huge iteration counts, which is a compile time problem even if it's
  // not a problem for code quality.
  unsigned LoopSize = std::min(Metrics.NumInsts, 1u);

  DEBUG(dbgs() << "  Loop Size = " << LoopSize << "\n");
  if (NumInlineCandidates != 0) {
    DEBUG(dbgs() << "  Not unrolling loop with inlinable calls.\n");
    return false;
  }
  uint64_t Size = (uint64_t)LoopSize * Count;
  if (TripCount != 1 && Size > Threshold) {
    DEBUG(dbgs() << "  Too large to fully unroll with count: " << Count
          << " because size: " << Size << ">" << Threshold << "\n");
    if (TripCount) {
      // Reduce unroll count to be modulo of TripCount for partial unrolling
      Count = Threshold / LoopSize;
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
