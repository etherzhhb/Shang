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
  ScalarEvolution &SE;

  const Instruction *getAsNonTrivial(const Value *Src) {
    if (const Instruction *Inst = dyn_cast<Instruction>(Src)) {
      if (Inst->mayHaveSideEffects() || Inst->mayReadOrWriteMemory())
        return Inst;

      // The PHINode introducing the back-edge are also nontrivial.
      if (const PHINode *PN = dyn_cast<PHINode>(Inst))
        if (PN->getParent() == L->getHeader())
          return Inst;
    }

    return 0;
  }

  static bool insertDep(DepInfoTy &Dep, const Value *Src, unsigned Distance) {
    unsigned &OldDistance = Dep[Src];

    // Relax the distance in the DepInfo.
    if (OldDistance) {
      // No need to change the old distance.
      if (OldDistance <= Distance) return false;

      // Replace the distance by a shorter one.
      OldDistance = Distance;
      return true;
    }

    OldDistance = Distance;
    return OldDistance != 0;
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

  bool buildDepMap(LoopInfo *LI, AliasAnalysis *AA);
  void buildDep(const Instruction *Inst, unsigned Distance);
  void buildDepForBackEdges();
  void buildMemDep(ArrayRef<const Instruction*> MemOps, AliasAnalysis *AA);
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
                     AliasAnalysis *AA, bool SrcBeforeDst);

  void buildTransitiveClosure(ArrayRef<const Instruction*> MemOps);
public:

  LoopDepGraph(Loop *L, ScalarEvolution &SE) : L(L), SE(SE) {}

  // Build the dependency graph of the loop body, return true if success, false
  // otherwise.
  bool buildDepGraph(LoopInfo *LI, AliasAnalysis *AA);
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

  // Prevent loop when building dependency graph.
  DepInfo.erase(Inst);
  // Make the current node depends on the header of the Loop (i.e. the root of
  // the dependency graph) if it do not have any source.
  if (DepInfo.empty())  insertDep(DepInfo, L->getHeader(), 0);
}

bool
LoopDepGraph::buildDepMap(LoopInfo *LI, AliasAnalysis *AA){
  // Do not mess up with multi-block Loop right now.
  if (L->getNumBlocks() > 1) return false;

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

  // Remove the entry of the trivial instruction in the dependency map.
  while (!TrivialInsts.empty()) {
    DepMap.erase(TrivialInsts.back());
    TrivialInsts.pop_back();
  }

  buildMemDep(Nontrivials, AA);

  // Simply create the entry for the root of the dependencies graph.
  DepMap.insert(std::make_pair(L->getHeader(), DepInfoTy()));

  // Build the transitive closure of the dependency relation.
  // FIXME: Calculate the max parallel distance for each SCC.
  buildTransitiveClosure(Nontrivials);

  return true;
}

void LoopDepGraph::buildTransitiveClosure(ArrayRef<const Instruction*> MemOps) {
  bool Changed = true;
  while (Changed) {
    Changed = false;

    for (unsigned i = 0; i < MemOps.size(); ++i) {
      const Instruction *CurInst = MemOps[i];
      DepInfoTy &CurDep = DepMap[CurInst];
      assert(!CurDep.empty() && "Unexpected empty dependency map!");

      typedef DepInfoTy::iterator iterator;
      for (iterator I = CurDep.begin(), E = CurDep.end(); I != E; ++I) {
        const Value *Src = I->first;
        unsigned Distance = I->second;

        const DepInfoTy &SrcDep = DepMap[Src];
        assert((!SrcDep.empty() || isa<BasicBlock>(Src))
               && "Unexpected empty dependency map!");

        // Forward the dependencies of Src.
        typedef DepInfoTy::const_iterator const_iterator;
        for (const_iterator SI = SrcDep.begin(), SE = SrcDep.end();
             SI != SE; ++SI) {
          unsigned NewDistance = Distance + SI->second;
          Changed |= insertDep(CurDep, I->first, NewDistance);
        }
      }
    }
  }
}

void LoopDepGraph::buildDepForBackEdges() {
  BasicBlock *Header = L->getHeader();

  typedef BasicBlock::iterator iterator;
  for (iterator I = Header->begin(); isa<PHINode>(I); ++I)
    // The distance of the back-edges are 1.
    buildDep(I, 1);
}

int LoopDepGraph::getDepDistance(const Instruction *Src, const Instruction *Dst,
                                 AliasAnalysis *AA, bool SrcBeforeDst) {
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
    const SCEV *SAddrSCEV = SE.getSCEVAtScope(SrcAddr, L);
    const SCEV *DAddrSCEV = SE.getSCEVAtScope(DstAddr, L);
    return getLoopDepDist(SAddrSCEV, DAddrSCEV, SrcBeforeDst, SrcLoc.Size, &SE);
  }

  return getLoopDepDist(SrcBeforeDst);
}

void LoopDepGraph::buildMemDep(ArrayRef<const Instruction*> MemOps,
                               AliasAnalysis *AA) {
  for (unsigned i = 1; i < MemOps.size(); ++i) {
    const Instruction *Dst = MemOps[i];
    // Ignore the PHI's.
    if (isa<PHINode>(Dst))  continue;

    for (unsigned j = 0; j < i; ++j) {
      const Instruction *Src = MemOps[j];
      // Ignore the PHI's.
      if (isa<PHINode>(Src))  continue;

      int Distance = getDepDistance(Src, Dst, AA, true);
      if (Distance >= 0) insertDep(Src, Dst, Distance);

      Distance = getDepDistance(Dst, Src, AA, false);
      if (Distance >= 0) insertDep(Dst, Src, Distance);
    }
  }
}

bool LoopDepGraph::buildDepGraph(LoopInfo *LI, AliasAnalysis *AA) {
  if (!buildDepMap(LI, AA)) return false;

  unsigned NumParallelIt = UINT32_MAX;

  typedef DepMapTy::iterator iterator;
  DEBUG(dbgs() << "loops in dependency graph:\n");
  for (iterator I = DepMap.begin(), E = DepMap.end(); I != E; ++I) {
    if (const Instruction *Inst = dyn_cast<Instruction>(I->first)) {
      unsigned LoopDistance = I->second.lookup(Inst);
      DEBUG(dbgs() << *Inst<< " loop-distance: " << LoopDistance << '\n');

      if (LoopDistance) NumParallelIt = std::min(NumParallelIt, LoopDistance);
    }
  }

  return true;
}

char TrivialLoopUnroll::ID = 0;
INITIALIZE_PASS_BEGIN(TrivialLoopUnroll, "trivial-loop-unroll",
                      "Unroll trivial loops", false, false)
INITIALIZE_AG_DEPENDENCY(AliasAnalysis)
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
