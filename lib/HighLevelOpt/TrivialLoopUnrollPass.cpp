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

#include "vtm/Passes.h"
#include "vtm/DesignMetrics.h"
#include "vtm/FUInfo.h"
#include "vtm/Utilities.h"

#include "llvm/IntrinsicInst.h"
#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/Analysis/LoopPass.h"
#include "llvm/Analysis/LoopIterator.h"
#include "llvm/Analysis/ScalarEvolution.h"
#include "llvm/Analysis/ScalarEvolutionExpressions.h"
#include "llvm/Transforms/Scalar.h"
#include "llvm/Transforms/Utils/UnrollLoop.h"
#include "llvm/Target/TargetData.h"
#include "llvm/Support/CallSite.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/raw_ostream.h"
#define DEBUG_TYPE "trivial-loop-unroll"
#include "llvm/Support/Debug.h"

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

// The dependency graph of the loop body.
class LoopDepGraph {
  typedef DenseMap<const Value*, unsigned> DepInfoTy;
  // The map that hold the dependency distance from the load instructions.
  // In contrast, the the dependency graph should only contains load and store.
  typedef DenseMap<const Value*, DepInfoTy> DepMapTy;
  DepMapTy DepMap;

  // The current Loop.
  Loop *L;

  const Instruction *getAsNonTrivial(const Value *Src) {
    if (const Instruction *Inst = dyn_cast<Instruction>(Src)) {
      if (Inst->mayHaveSideEffects()) return Inst;

      // The PHINode introducing the back-edge are also nontrivial.
      if (const PHINode *PN = dyn_cast<PHINode>(Inst))
        if (PN->getParent() == L->getHeader())
          return Inst;
    }

    return 0;
  }

  static void insertDep(DepInfoTy &Dep, const Value *Src, unsigned Distance) {
    // Relax the distance in the DepInfo.
    unsigned &OldDistance = Dep[Src];
    OldDistance = std::max(OldDistance, Distance);
  }

  void insertDep(const Value *Src, const Value *Dst, unsigned Distance) {
    DepInfoTy &Deps = DepMap[Dst];
    assert(!Deps.empty() && "Unexpected empty dependence set!");
    insertDep(Deps, Src, Distance);
  }

  static void mergeDeps(DepInfoTy &To, const DepInfoTy &From) {
    typedef DepInfoTy::const_iterator iterator;
    for (iterator I = From.begin(), E = From.end(); I != E; ++I)
      insertDep(To, I->first, I->second);
  }

  bool buildDepMap(LoopInfo *LI, ScalarEvolution *SE, AliasAnalysis *AA);
  void buildDep(const Instruction *Inst, unsigned Distance);
  void buildDepForBackEdges();
  void buildMemDep(ArrayRef<const Instruction*> MemOps, ScalarEvolution *SE,
                   AliasAnalysis *AA);
  static
  AliasAnalysis::Location getLocation(const Instruction *I, AliasAnalysis *AA) {
    if (const LoadInst *LI = dyn_cast<LoadInst>(I))
      return AA->getLocation(LI);

    if (const StoreInst *SI = dyn_cast<StoreInst>(I))
      return AA->getLocation(SI);

    llvm_unreachable("Unexpected instruction!");
    return AliasAnalysis::Location();
  }

  int getDepDistance(const Instruction *Src, const Instruction *Dst,
                     ScalarEvolution *SE, AliasAnalysis *AA, bool SrcBeforeDst);

  void buildTransitiveClosure() {}
public:

  explicit LoopDepGraph(Loop *L) : L(L) {}

  // Build the dependency graph of the loop body, return true if success, false
  // otherwise.
  bool buildDepGraph(LoopInfo *LI, ScalarEvolution *SE, AliasAnalysis *AA);
};
}

void LoopDepGraph::buildDep(const Instruction *Inst, unsigned Distance) {
  typedef Instruction::const_op_iterator op_iterator;
  DepInfoTy &DepInfo = DepMap[Inst];

  for (op_iterator I = Inst->op_begin(), E = Inst->op_end(); I != E; ++I) {
    const Value *Src = *I;
    // Simply insert the nontrivial instruction to DepInfo.
    if (const Value *Nontrivial = getAsNonTrivial(Src)) {
      insertDep(DepInfo, Nontrivial, Distance);
      continue;
    }

    // If Src is not a leaf, we need to forward all its dependencies to the
    // current DepInfo.
    DepMapTy::iterator at = DepMap.find(Src);
    // Ignore the loop invariants.
    if (at == DepMap.end()) continue;

    // Merge the DepInfo.
    mergeDeps(DepInfo, at->second);
  }

  // Make the current node depends on the header of the Loop (i.e. the root of
  // the dependency graph) if it do not have any source.
  if (DepInfo.empty())  insertDep(DepInfo, L->getHeader(), 0);
}

bool
LoopDepGraph::buildDepMap(LoopInfo *LI, ScalarEvolution *SE, AliasAnalysis *AA){
  // Do not mess up with multi-block Loop right now.
  if (L->getNumBlocks() > 1) return false;

  // Find "latch trip count". UnrollLoop assumes that control cannot exit
  // via the loop latch on any iteration prior to TripCount. The loop may exit
  // early via an earlier branch.
  BasicBlock *LatchBlock = L->getLoopLatch();
  if (LatchBlock == 0) return false;

  LoopBlocksDFS DFS(L);
  DFS.perform(LI);

  std::vector<const Instruction*> Nontrivials, TrivialInsts;

  // Visit the blocks in top-order.
  typedef LoopBlocksDFS::RPOIterator top_iterator;
  typedef BasicBlock::iterator iterator;
  for (top_iterator I = DFS.beginRPO(), E = DFS.endRPO(); I != E; ++I) {
    BasicBlock *BB = *I;
    for (iterator BI = BB->begin(), BE = BB->end(); BI != BE; ++BI) {
      // Ignore the loops with CallSite in its body.
      if (isa<CallInst>(BI) || isa<InvokeInst>(BI)) return false;

      // Collect the nontrivial instructions, i.e. the load/store/call.
      if (const Instruction *Nontrivial = getAsNonTrivial(BI))
        Nontrivials.push_back(Nontrivial);
      else
        TrivialInsts.push_back(BI);

      // Build flow-dependences for the current Instruction, the iterate
      // distance of the dependences are 0.
      buildDep(BI, 0);
    }
  }

  buildDepForBackEdges();

  buildMemDep(Nontrivials, SE, AA);

  // Remove the entry of the trivial instruction in the dependency map.
  while (!TrivialInsts.empty()) {
    DepMap.erase(TrivialInsts.back());
    TrivialInsts.pop_back();
  }


  return true;
}

void LoopDepGraph::buildDepForBackEdges() {
  BasicBlock *Header = L->getHeader();

  typedef BasicBlock::iterator iterator;
  for (iterator I = Header->begin(); isa<PHINode>(I); ++I)
    // The distance of the back-edges are 1.
    buildDep(I, 1);
}

int LoopDepGraph::getDepDistance(const Instruction *Src, const Instruction *Dst,
                                 ScalarEvolution *SE, AliasAnalysis *AA,
                                 bool SrcBeforeDst) {
  // Ignore RAR dependencies.
  if (!Src->mayWriteToMemory() && !Dst->mayWriteToMemory())
    return -1;

  AliasAnalysis::Location SrcLoc = getLocation(Src, AA),
                          DstLoc = getLocation(Dst, AA);
  Value *SrcAddr = const_cast<Value*>(SrcLoc.Ptr),
        *DstAddr = const_cast<Value*>(DstLoc.Ptr);

  if (AA->isNoAlias(SrcLoc, DstLoc)) return -1;

  if (L->isLoopInvariant(SrcAddr) && L->isLoopInvariant(DstAddr))
    return getLoopDepDist(SrcBeforeDst);

  if (SrcLoc.Size == DstLoc.Size) {
    const SCEV *SrcAddrSCEV = SE->getSCEVAtScope(SrcAddr, L);
    const SCEV *DstAddrSCEV = SE->getSCEVAtScope(DstAddr, L);
    return getLoopDepDist(SrcAddrSCEV, DstAddrSCEV,
                          !Src->mayWriteToMemory(), !Dst->mayWriteToMemory(),
                          SrcBeforeDst, SrcLoc.Size, SE);
  }

  return getLoopDepDist(SrcBeforeDst);
}

void LoopDepGraph::buildMemDep(ArrayRef<const Instruction*> MemOps,
                               ScalarEvolution *SE, AliasAnalysis *AA) {
  for (unsigned i = 1; i < MemOps.size(); ++i) {
    const Instruction *Dst = MemOps[i];
    for (unsigned j = 0; j < i; ++j) {
      const Instruction *Src = MemOps[i];
      int Distance = getDepDistance(Src, Dst, SE, AA, true);
      if (Distance >= 0) insertDep(Src, Dst, Distance);

      Distance = getDepDistance(Dst, Src, SE, AA, false);
      if (Distance >= 0) insertDep(Dst, Src, Distance);
    }
  }
}

bool LoopDepGraph::buildDepGraph(LoopInfo *LI, ScalarEvolution *SE,
                                 AliasAnalysis *AA) {
  if (!buildDepMap(LI, SE, AA)) return false;

  return true;
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
  unsigned Threshold = VFUs::MulCost[63] * 4;

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
