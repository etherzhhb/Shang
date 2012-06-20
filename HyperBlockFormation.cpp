//===- HyperBlockFormation.cpp - Merge Fall Through Blocks ---*- C++ -*-===//
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
// This file implements a pass that merge the fall through blocks into its
// predecessor blocks to increase instruction level parallelism.
//
//===----------------------------------------------------------------------===//

#include "BBDelayAnalysis.h"

#include "vtm/Passes.h"
#include "vtm/VerilogBackendMCTargetDesc.h"
#include "vtm/VInstrInfo.h"
#include "vtm/Utilities.h"

#include "llvm/../../lib/CodeGen/BranchFolding.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineBranchProbabilityInfo.h"
#include "llvm/CodeGen/MachineBlockFrequencyInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineLoopInfo.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Format.h"
#define DEBUG_TYPE "vtm-hyper-block"
#include "llvm/Support/Debug.h"
#include <set>
#include <map>

using namespace llvm;

STATISTIC(BBsMerged, "Number of blocks are merged into hyperblock");
STATISTIC(TrivialBBsMerged, "Number of trivial blocks are merged into hyperblock");
STATISTIC(BBsMergedNotBenefit, "Number of blocks are not merged into hyperblock"
                               " (weighted path not benefit)");
STATISTIC(BBsMergedBigIncrement, "Number of blocks are not merged into hyperblock"
                               " (BB delay increase too much)");

cl::opt<unsigned> PathDiffThreshold("vtm-hyper-block-path-threshold",
                                    cl::desc("HyperBlock merging threshold"),
                                    cl::init(8));
cl::opt<unsigned>
TrivialBBThreshold("vtm-hyper-block-trivial-bb",
                   cl::desc("The minimal delay of bb that will be treated as"
                            " non-trivial bb in HyperBlock Formation"),
                            cl::init(1)); //Disabled at the moment.

namespace {
struct HyperBlockFormation : public MachineFunctionPass {
  static char ID;

  const TargetInstrInfo *TII;
  MachineRegisterInfo *MRI;
  MachineLoopInfo *LI;
  MachineBranchProbabilityInfo *MBPI;
  MachineBlockFrequencyInfo *MBFI;
  BBDelayAnalysis *BBDelay;

  typedef SmallVector<unsigned, 4> IntSetTy;
  typedef DenseMap<unsigned, IntSetTy> CFGMapTy;
  CFGMapTy CFGMap;

  // Trace number, Trace bit map.
  typedef std::pair<uint8_t, int64_t> TraceInfo;
  // MBB number -> trace number.
  typedef DenseMap<unsigned, TraceInfo> TraceMapTy;
  typedef DenseMap<unsigned, TraceMapTy> AllTraceMapTy;
  AllTraceMapTy AllTraces;
  uint8_t NextTraceNum;

  HyperBlockFormation()
    : MachineFunctionPass(ID), TII(0), MRI(0), LI(0), MBPI(0), MBFI(0),
      BBDelay(0), NextTraceNum(0) {
    initializeHyperBlockFormationPass(*PassRegistry::getPassRegistry());
  }

  void getAnalysisUsage(AnalysisUsage &AU) const {
    MachineFunctionPass::getAnalysisUsage(AU);
    AU.addRequired<MachineLoopInfo>();
    AU.addRequired<MachineBranchProbabilityInfo>();
    AU.addRequired<MachineBlockFrequencyInfo>();
    AU.addRequired<BBDelayAnalysis>();
    //AU.addPreserved<BBDelayAnalysis>();
  }

  void buildCFGForBB(MachineBasicBlock *MBB);

  void annotateLocalCFGInfo(MachineOperand *MO, uint8_t TraceNum,
                            int64_t TraceBitMap) {
    assert((TraceBitMap || TraceNum == 0) && "Not have any predecessor?");
    MO->ChangeToImmediate(TraceBitMap);
    assert(TraceNum < 64 && "Only support 64 local BB at most!");
    MO->setTargetFlags(TraceNum);
  }

  int64_t buildTraceBitMap(unsigned SrcBBNum, unsigned DstBBNum,
                           const TraceMapTy &TraceMap) const {
    if (SrcBBNum == DstBBNum) return UINT64_C(0);

    const IntSetTy &Preds = CFGMap.lookup(DstBBNum);
    typedef IntSetTy::const_iterator pred_it;
    int64_t Result = 0ull;
    for (pred_it I = Preds.begin(), E = Preds.end(); I != E; ++I) {
      unsigned PredBBNum = *I;

      TraceInfo Info = TraceMap.lookup(PredBBNum);
      // Add the bit of the current predecessor.
      Result |= UINT64_C(1) << Info.first;
      // And the predecessors of the current predecessor.
      assert((Info.second || PredBBNum == SrcBBNum)
             && "Do not have predecessor trace?");
      Result |= Info.second;
    }

    assert(Result && "Not have any predecessor?");
    return Result;
  }

  unsigned mergeHyperBlockTrace(unsigned CurBBNum, uint8_t NextTraceNum,
                                int64_t CurTraceBitMap, TraceMapTy &ParentTrace)
                                const;

  bool runOnMachineFunction(MachineFunction &MF);
  MachineBasicBlock *getMergeDst(MachineBasicBlock *Src,
                                 VInstrInfo::JT &SrcJT,
                                 VInstrInfo::JT &DstJT);

  typedef SmallVectorImpl<MachineBasicBlock*> BBVecTy;
  bool mergeBlocks(MachineBasicBlock *MBB, BBVecTy &BBsToMerge);

  bool mergeBlock(MachineBasicBlock *FromBB, MachineBasicBlock *ToBB);
  typedef DenseMap<MachineBasicBlock*, unsigned> BB2DelayMapTy;
  bool mergeSuccBlocks(MachineBasicBlock *MBB, BB2DelayMapTy &UnmergedDelays);

  bool mergeTrivialSuccBlocks(MachineBasicBlock *MBB);
  void PredicateBlock(unsigned ToBBNum, MachineOperand Cnd,
                      MachineBasicBlock *BB);

  bool optimizeRetBB(MachineFunction &MF, MachineBasicBlock &RetBB);
  void moveRetValBeforePHI(MachineInstr *PHI, MachineBasicBlock *RetBB);
  MachineInstr *getInsertPosBeforePHI(MachineBasicBlock *SrcBB,
                                      MachineBasicBlock *DstBB,
                                      MachineOperand &Pred);
  const char *getPassName() const { return "Hyper-Block Formation Pass"; }
};

// Helper class for BB sorting.
struct sort_bb_by_freq {
  MachineBlockFrequencyInfo *MBFI;

  explicit sort_bb_by_freq(MachineBlockFrequencyInfo *mbfi) : MBFI(mbfi) {}

  inline bool operator() (const MachineBasicBlock *LHS,
                          const MachineBasicBlock *RHS) const {
    return MBFI->getBlockFreq(LHS) < MBFI->getBlockFreq(RHS);
  }
};

template<typename T>
static T gcd(T a, T b) {
  while (b > 0) {
    T temp = b;
    b = a % b; // % is remainder
    a = temp;
  }

  return a;
}

template<typename T>
static T lcm(T a, T b) {
  return a * (b / gcd(a, b));
}

//template<typename T>
//static long gcd(ArrayRef<T> input) {
//  T result = input[0];
//
//  for(unsigned i = 1; i < input.size(); i++)
//    result = gcd(result, input[i]);
//
//  return result;
//}
//
//template<typename T>
//static T lcm(ArrayRef<T> input) {
//  T result = input[0];
//
//  for(int i = 1; i < input.size(); i++)
//    result = lcm(result, input[i]);
//
//  return result;
//}

struct Probability {
  // Numerator
  uint32_t N;

  // Denominator
  uint32_t D;

  void scale(uint32_t S) {
    N /= S;
    D /= S;
  }

  void scaleByGCD() {
    scale(gcd(N, D));
  }

  /*implicit*/ Probability(BranchProbability P)
    : N(P.getNumerator()), D(P.getDenominator()) {
    assert(D > 0 && "Denomiator cannot be 0!");
    //scaleByGCD();
  }

  explicit Probability(uint32_t n = 0, uint32_t d = 1) : N(n), D(d) {
    assert(D > 0 && "Denomiator cannot be 0!");
    //scaleByGCD();
  }

  uint32_t getNumerator() const { return N; }
  uint32_t getDenominator() const { return D; }

  Probability &operator+=(Probability P) {

    uint32_t Numerator = getNumerator() * P.getDenominator() +
                        P.getNumerator() * getDenominator();
    uint32_t Denominator = getDenominator() * P.getDenominator();
    N = Numerator;
    D = Denominator;
    scaleByGCD();
    return *this;
  }

  Probability &operator*=(Probability P) {
    uint32_t Numerator = getNumerator() * P.getNumerator();
    uint32_t Denominator = getDenominator() * P.getDenominator();
    N = Numerator;
    D = Denominator;
    scaleByGCD();
    return *this;
  }

  void print(raw_ostream &OS) const {
    OS << N << " / " << D << " = " << format("%g%%", ((double)N / D) * 100.0);
  }
};

raw_ostream &operator<<(raw_ostream &OS, const Probability &Prob) {
  Prob.print(OS);
  return OS;
}
}

char HyperBlockFormation::ID = 0;

INITIALIZE_PASS_BEGIN(HyperBlockFormation, "vtm-hyper-block",
                      "VTM - Hyper Block Formation", false, false)
  INITIALIZE_PASS_DEPENDENCY(MachineBranchProbabilityInfo)
  INITIALIZE_PASS_DEPENDENCY(MachineBlockFrequencyInfo)
  INITIALIZE_PASS_DEPENDENCY(BBDelayAnalysis)
INITIALIZE_PASS_END(HyperBlockFormation, "vtm-hyper-block",
                    "VTM - Hyper Block Formation", false, false)

bool HyperBlockFormation::mergeBlocks(MachineBasicBlock *MBB, BBVecTy &BBs) {
  bool ActuallyMerged = false;
  while (!BBs.empty()) {
    MachineBasicBlock *SuccBB = BBs.back();
    BBs.pop_back();

    ActuallyMerged |= mergeBlock(SuccBB, MBB);
  }

  CycleLatencyInfo CL(*MRI);
  // Update the delay.
  if (ActuallyMerged)
    BBDelay->updateDelay(MBB, CL.computeLatency(*MBB));

  return ActuallyMerged;
}

bool HyperBlockFormation::mergeSuccBlocks(MachineBasicBlock *MBB,
                                          BB2DelayMapTy &UnmergedDelays) {
  VInstrInfo::JT CurJT, SuccJT;
  uint64_t WeightSum = 0;

  // Latency statistics.
  CycleLatencyInfo CL(*MRI);
  unsigned MBBDelay = CL.computeLatency(*MBB), MaxMergedDelay = 0;
  // Weighted delays.
  uint64_t UnmergedTotalDelay = 0, MergedTotalDelay = 0;
  // The delay after BB is merged.
  SmallVector<MachineBasicBlock*, 8> BBsToMerge;
  SmallPtrSet<MachineBasicBlock*, 8> UmergedBBs;

  typedef MachineBasicBlock::succ_iterator succ_iterator;
  for (succ_iterator I = MBB->succ_begin(), E = MBB->succ_end(); I != E; ++I) {
    MachineBasicBlock *SuccBB = *I;
    SuccJT.clear();
    CurJT.clear();

    uint64_t EdgeWeight = MBPI->getEdgeWeight(MBB, SuccBB);
    WeightSum += EdgeWeight;

    unsigned UnmergedPathDelay = MBBDelay + BBDelay->getBBDelay(SuccBB);
    DenseMap<MachineBasicBlock*, unsigned>::const_iterator at;
    bool updated;
    tie(at, updated) = UnmergedDelays.insert(std::make_pair(MBB,
                                                            UnmergedPathDelay));
    // Delay already in the map.
    if (!updated) UnmergedPathDelay = at->second;
    // Weighted delay of unmerged edge.
    UnmergedTotalDelay += UnmergedPathDelay * EdgeWeight;

    MachineBasicBlock *MergeDst = getMergeDst(SuccBB, SuccJT, CurJT);
    if (!MergeDst) {
      UmergedBBs.insert(SuccBB);
      continue;
    }

    assert(MergeDst == MBB && "Succ have more than one Pred?");
    unsigned MergedDelay = CL.computeLatency(*SuccBB);
    BBsToMerge.push_back(SuccBB);
    MaxMergedDelay = std::max(MaxMergedDelay, MergedDelay);
  }

  // Compute the delay of paths after BBs are merged.
  for (succ_iterator I = MBB->succ_begin(), E = MBB->succ_end(); I != E; ++I) {
    MachineBasicBlock *SuccBB = *I;
    // The path delay of the merged edge becomes the max delay of the merged block.
    unsigned PathDelay = MaxMergedDelay;
    // The block delay still contributes to the path delay for umerged block.
    if (UmergedBBs.count(SuccBB)) PathDelay += BBDelay->getBBDelay(SuccBB);

    MergedTotalDelay += MaxMergedDelay * MBPI->getEdgeWeight(MBB, SuccBB);
  }

  // Only merge the SuccBB into current BB if the merge is beneficial.
  // TODO: Remove the SuccBB with smallest beneficial and try again.
  if (UnmergedTotalDelay < MergedTotalDelay) {
    ++BBsMergedNotBenefit;
    return false;
  }

  // Do not completely trust the branch probability analysis! Do merge the
  // blocks if the delay increase too much.
  if (double(MaxMergedDelay) / double(MBBDelay) >
      PathDiffThreshold / double(LI->getLoopDepth(MBB) + 1)) {
    ++BBsMergedBigIncrement;
    return false;
  }

  return mergeBlocks(MBB, BBsToMerge);
}

bool HyperBlockFormation::mergeTrivialSuccBlocks(MachineBasicBlock *MBB) {
  SmallVector<MachineBasicBlock*, 8> BBsToMerge;
  VInstrInfo::JT CurJT, SuccJT;

  typedef MachineBasicBlock::succ_iterator succ_iterator;
  for (succ_iterator I = MBB->succ_begin(), E = MBB->succ_end(); I != E; ++I) {
    MachineBasicBlock *Succ = *I;
    CurJT.clear();
    SuccJT.clear();
    // Cannot merge Succ to MBB.
    if (getMergeDst(Succ, SuccJT, CurJT) != MBB) continue;

    if (Succ->empty()||Succ->getFirstInstrTerminator() ==Succ->instr_begin()) {
      BBsToMerge.push_back(Succ);
      continue;
    }

    unsigned Delay = BBDelay->getBBDelay(Succ);
    if (Delay < TrivialBBThreshold) {
      ++TrivialBBsMerged;
      BBsToMerge.push_back(Succ);
    }
  }

  return mergeBlocks(MBB, BBsToMerge);
}

void HyperBlockFormation::buildCFGForBB(MachineBasicBlock *MBB) {
  unsigned BBNum = MBB->getNumber();

  typedef MachineBasicBlock::pred_iterator pred_it;
  for (pred_it I = MBB->pred_begin(), E = MBB->pred_end(); I != E; ++I)
    CFGMap[BBNum].push_back((*I)->getNumber());
}

bool HyperBlockFormation::runOnMachineFunction(MachineFunction &MF) {
  TII = MF.getTarget().getInstrInfo();
  MRI = &MF.getRegInfo();
  LI = &getAnalysis<MachineLoopInfo>();
  MBPI = &getAnalysis<MachineBranchProbabilityInfo>();
  MBFI = &getAnalysis<MachineBlockFrequencyInfo>();
  BBDelay = &getAnalysis<BBDelayAnalysis>();

  bool MakeChanged = false;
  AllTraces.clear();
  typedef MachineFunction::reverse_iterator rev_it;

  MF.RenumberBlocks();
  // Sort the BBs according they frequency.
  std::vector<MachineBasicBlock*> SortedBBs;
  CFGMap.clear();

  for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I) {
    SortedBBs.push_back(I);
    buildCFGForBB(I);
    fixTerminators(I);
  }

  std::sort(SortedBBs.begin(), SortedBBs.end(), sort_bb_by_freq(MBFI));

  // Delays before branching to not BB that cannot be merged.
  DenseMap<MachineBasicBlock*, unsigned> UnmergedDelays;

  while (!SortedBBs.empty()) {
    bool BlockMerged = false;
    MachineBasicBlock *MBB = SortedBBs.back();
    SortedBBs.pop_back();

    unsigned CurBBNum = MBB->getNumber();
    // Initialize the local CFG.
    AllTraces[CurBBNum][CurBBNum] = std::make_pair(0, UINT64_C(0));
    NextTraceNum = 1;

    // Merge trivial blocks and hot blocks.
    do {
      BlockMerged = false;
      MakeChanged |= BlockMerged |= mergeTrivialSuccBlocks(MBB);
      MakeChanged |= BlockMerged |= mergeSuccBlocks(MBB, UnmergedDelays);
    } while (BlockMerged && NextTraceNum < 64);
    // Prepare for next block.
    UnmergedDelays.clear();


    VInstrInfo::JT T;
    VInstrInfo::extractJumpTable(*MBB, T, false);
  }

  for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I)
    if (I->succ_size() == 0 && !I->empty()
        && I->back().getOpcode() == VTM::VOpRet) {
      MakeChanged |= optimizeRetBB(MF, *I);
    }

  // Optimize the cfg, but do not perform tail merge.
  BranchFolder BF(true, true);
  MakeChanged |= BF.OptimizeFunction(MF, TII, MF.getTarget().getRegisterInfo(),
                                     getAnalysisIfAvailable<MachineModuleInfo>());



  // Fix terminators after branch folding.
  for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I)
    fixTerminators(I);

  MF.RenumberBlocks();

  return MakeChanged;
}

bool HyperBlockFormation::optimizeRetBB(MachineFunction &MF,
                                        MachineBasicBlock &RetBB) {
  // Return block do not have any successor.
  if (RetBB.succ_size()) return false;

  MachineInstr *RetVal = 0, *RetValDef = 0;
  typedef MachineBasicBlock::instr_iterator it;
  for (it I = RetBB.instr_begin(), E = RetBB.instr_end(); I != E; ++I) {
    if (I->getOpcode() != VTM::VOpRetVal) continue;

    RetVal = I;
    break;
  }

  // Nothing to optimize.
  if (RetVal == 0) return false;

  // Try to move the VOpRetVal to the predecessor block.
  MachineOperand RetValMO = RetVal->getOperand(0);
  if (RetValMO.isReg()) {
    RetValDef = MRI->getVRegDef(RetValMO.getReg());
    assert(RetValDef && "Not in SSA Form!");
    // Cannot move to predecessor block.
    if (RetValDef->getParent() != &RetBB || !RetValDef->isPHI())
      return false;
  }

  if (!RetValDef || !RetValDef->isPHI()) {
    RetVal->eraseFromParent();
    typedef MachineBasicBlock::pred_iterator pred_it;
    for (pred_it I = RetBB.pred_begin(), E = RetBB.pred_end(); I != E; ++I) {
      MachineOperand Pred = VInstrInfo::CreatePredicate();
      it IP = getInsertPosBeforePHI(*I, &RetBB, Pred);
      BuildMI(**I, IP, DebugLoc(), TII->get(VTM::VOpRetVal))
        .addOperand(RetValMO).addOperand(VInstrInfo::CreateImm(0, 8))
        .addOperand(Pred).addOperand(VInstrInfo::CreateTrace());
    }

    return true;
  }

  RetVal->eraseFromParent();
  moveRetValBeforePHI(RetValDef, &RetBB);

  return true;
}

void HyperBlockFormation::moveRetValBeforePHI(MachineInstr *PHI,
                                              MachineBasicBlock *RetBB) {
  for (unsigned i = 1, e = PHI->getNumOperands(); i != e; i += 2) {
    MachineOperand &SrcMO = PHI->getOperand(i);
    MachineInstr *DefMI = MRI->getVRegDef(SrcMO.getReg());
    assert(DefMI && "Not in SSA form?");

    MachineBasicBlock *SrcBB = PHI->getOperand(i + 1).getMBB();

    MachineOperand Pred = VInstrInfo::CreatePredicate();
    typedef MachineBasicBlock::instr_iterator it;
    it IP = getInsertPosBeforePHI(SrcBB, RetBB, Pred);

    BuildMI(*SrcBB, IP, DebugLoc(), TII->get(VTM::VOpRetVal))
      .addOperand(SrcMO).addOperand(VInstrInfo::CreateImm(0, 8))
      .addOperand(Pred).addOperand(VInstrInfo::CreateTrace());
  }

  assert(MRI->use_empty(PHI->getOperand(0).getReg()) && "Unexpected other use!");
  PHI->eraseFromParent();
}

MachineInstr *HyperBlockFormation::getInsertPosBeforePHI(MachineBasicBlock *Src,
                                                         MachineBasicBlock *Dst,
                                                         MachineOperand &Pred) {
  VInstrInfo::JT SrcJT;
  bool success = !VInstrInfo::extractJumpTable(*Src, SrcJT, false);
  assert(success && "Broken machine code?");
  // TODO: Handle critical edges.

  // Insert the PHI copy.
  VInstrInfo::JT::iterator at = SrcJT.find(Dst);
  assert(at != SrcJT.end() && "Broken CFG?");
  Pred = at->second;
  return Src->getFirstInstrTerminator();
}

MachineBasicBlock *HyperBlockFormation::getMergeDst(MachineBasicBlock *SrcBB,
                                                     VInstrInfo::JT &SrcJT,
                                                     VInstrInfo::JT &DstJT) {
  // Only handle simple case at the moment
  if (SrcBB->pred_size() != 1) return 0;

  MachineBasicBlock *DstBB = *SrcBB->pred_begin();
  // Do not change the parent loop of MBB.
  if (LI->getLoopFor(SrcBB) != LI->getLoopFor(DstBB)) return 0;

  // Do not mess up with strange CFG.
  if (VInstrInfo::extractJumpTable(*DstBB, DstJT)) return 0;
  if (VInstrInfo::extractJumpTable(*SrcBB, SrcJT)) return 0;

  // Do not mess up with self loop.
  if (SrcJT.count(SrcBB)) return 0;

  VInstrInfo::JT::iterator at = DstJT.find(SrcBB);
  assert(at != DstJT.end() && "SrcBB is not the successor of DstBB?");
  if (VInstrInfo::isAlwaysTruePred(at->second)) {
    typedef MachineBasicBlock::iterator it;
    // We need to predicate the block when merging it.
    for (it I = SrcBB->begin(), E = SrcBB->end();I != E;++I){
      MachineInstr *MI = I;
      if (!TII->isPredicable(MI))
        return 0;
    }
  }

  return DstBB;
}

bool HyperBlockFormation::mergeBlock(MachineBasicBlock *FromBB,
                                     MachineBasicBlock *ToBB)
{
  VInstrInfo::JT FromJT, ToJT;
  MachineBasicBlock *MergeDst = getMergeDst(FromBB, FromJT, ToJT);
  assert(MergeDst == ToBB && "Cannot merge block!");

  DEBUG(dbgs() << "Merging BB#" << FromBB->getNumber() << "\n");

  TII->RemoveBranch(*ToBB);
  TII->RemoveBranch(*FromBB);

  // Get the condition of jumping from ToBB to FromBB
  typedef VInstrInfo::JT::iterator jt_it;
  jt_it at = ToJT.find(FromBB);
  assert(at != ToJT.end() && "ToBB not branching to FromBB?");
  MachineOperand PredCnd = at->second;

  PredicateBlock(ToBB->getNumber(), PredCnd, FromBB);

  // And merge the block into its predecessor.
  ToBB->splice(ToBB->end(), FromBB, FromBB->begin(), FromBB->end());

  SmallVector<MachineOperand, 1> PredVec(1, PredCnd);

  typedef DenseMap<MachineBasicBlock*, Probability> ProbMapTy;
  ProbMapTy BBProbs;
  uint32_t WeightLCM = 1;
  // The probability of the edge from ToBB to FromBB.
  BranchProbability MergedBBProb =MBPI->getEdgeProbability(ToBB, FromBB);

  for (jt_it I = FromJT.begin(),E = FromJT.end(); I != E; ++I){
    MachineBasicBlock *Succ = I->first;
    // Merge the PHINodes.
    VInstrInfo::mergePHISrc(Succ, FromBB, ToBB, *MRI, PredVec);

    Probability SuccProb = MBPI->getEdgeProbability(FromBB, Succ);
    // In the merged block, sum of probabilities of FromBB's successor should
    // be the original edge probability from ToBB to FromBB.
    SuccProb *= MergedBBProb;
    BBProbs.insert(std::make_pair(Succ, SuccProb));
    WeightLCM = lcm(WeightLCM, SuccProb.getDenominator());

    FromBB->removeSuccessor(Succ);

    // And predicate the jump table.
    I->second = VInstrInfo::MergePred(I->second, PredCnd, *ToBB, ToBB->end(),
                                      MRI, TII, VTM::VOpAnd);
  }

  typedef MachineBasicBlock::succ_iterator succ_it;
  for (unsigned i = ToBB->succ_size(); i > 0; --i) {
    succ_it SuccIt = ToBB->succ_begin() + i - 1;
    MachineBasicBlock *Succ = *SuccIt;
    if (Succ != FromBB) {
      Probability &SuccProb = BBProbs[Succ];
      SuccProb += MBPI->getEdgeProbability(ToBB, Succ);
      WeightLCM = lcm(WeightLCM, SuccProb.getDenominator());
    }

    ToBB->removeSuccessor(SuccIt);
  }

  // Rebuild the successor list with new weights.
  for (ProbMapTy::iterator I = BBProbs.begin(), E = BBProbs.end(); I != E; ++I){
    Probability Prob = I->second;
    uint32_t w = Prob.getNumerator() * WeightLCM / Prob.getDenominator();
    DEBUG(dbgs() << I->first->getName() << ' ' << I->second << ' ' << w <<'\n');
    ToBB->addSuccessor(I->first, w);
  }

  // Do not jump to FromBB any more.
  ToJT.erase(FromBB);
  // We had assert FromJT not contains FromBB.
  // FromJT.erase(FromBB);

  // Build the new Jump table.
  for (jt_it I = FromJT.begin(), E = FromJT.end(); I != E; ++I) {
    MachineBasicBlock *Succ = I->first;
    jt_it at = ToJT.find(Succ);
    // If the entry already exist in target jump table, merge it with opcode OR.
    if (at != ToJT.end())
      at->second = VInstrInfo::MergePred(at->second, I->second,
                                         *ToBB, ToBB->end(), MRI, TII,
                                         VTM::VOpOr);
    else // Simply insert the entry.
      ToJT.insert(*I);
  }

  // Re-insert the jump table.
  VInstrInfo::insertJumpTable(*ToBB, ToJT, DebugLoc());
  ++BBsMerged;
  return true;
}

void HyperBlockFormation::PredicateBlock(unsigned ToBBNum, MachineOperand Pred,
                                         MachineBasicBlock *MBB) {
  typedef std::map<unsigned, unsigned> PredMapTy;
  PredMapTy PredMap;
  SmallVector<MachineOperand, 1> PredVec(1, Pred);
  const bool isPredNotAlwaysTrue = !VInstrInfo::isAlwaysTruePred(Pred);

  TraceMapTy &TraceMap = AllTraces[ToBBNum];
  const unsigned CurBBNum = MBB->getNumber();

  const uint64_t TraceBitMap =
    buildTraceBitMap(ToBBNum, CurBBNum, TraceMap);
  // Create the trace entry.
  TraceInfo &Info = TraceMap[CurBBNum];
  Info.second = TraceBitMap;
  uint8_t &TraceNum = Info.first;

  // Build the local CFG information for this block.
  // Predicate the Block.
  for (MachineBasicBlock::iterator I = MBB->begin(), E = MBB->end();
       I != E; ++I) {
    if (I->isDebugValue())
      continue;

    if (VInstrInfo::isDatapath(I->getOpcode()))
      continue;

    bool IsAlreadyPredicated = TII->isPredicated(I);
    // Need to predicate the block, if the predicate is not always true.
    if (isPredNotAlwaysTrue) {
      if (IsAlreadyPredicated) {
        MachineOperand *MO = VInstrInfo::getPredOperand(I);
        unsigned k = (MO->getReg() << 1)
          | (VInstrInfo::isPredicateInverted(*MO) ? 1 :0 );
        unsigned &Reg = PredMap[k];
        if (!Reg)
          Reg = VInstrInfo::MergePred(*MO, Pred, *MBB, I, MRI,
          TII, VTM::VOpAnd).getReg();

        MO->ChangeToRegister(Reg, false);
        MO->setTargetFlags(1);

        // Also update the trace number.
        assert (MO[1].getImm() == 0);
      } else if (I->getOpcode() <= TargetOpcode::COPY) {
        MachineInstr *PseudoInst = I;
        ++I; // Skip current instruction, we may change it.
        PseudoInst = VInstrInfo::PredicatePseudoInstruction(PseudoInst,
          PredVec);
        assert(PseudoInst && "Cannot predicate pseudo instruction!");
        I = PseudoInst;
      } else {
        bool Predicated = TII->PredicateInstruction(I, PredVec);
        assert(Predicated && "Cannot predicate instruction!");
      }
    }

    // Also update the trace number.
    MachineOperand *MO = VInstrInfo::getPredOperand(I);
    if (MO == 0) continue;
    // Move from the predicate to the trace operand.
    ++MO;

    // Create the local BBNum entry on demand.
    if (TraceNum == 0) {
      // Note that we need to allocate new trace number even the predicate is
      // always true, because the current block maybe the join block of a
      // diamond CFG, which have two predecessors.
      TraceNum = NextTraceNum++;
      // If the current BB is already a hyper-block, merge its traces.
      NextTraceNum =
        mergeHyperBlockTrace(CurBBNum, NextTraceNum, TraceBitMap, TraceMap);
    }

    uint8_t CurTraceNum = TraceNum;
    int64_t CurTraceBitMap = TraceBitMap;

    // Transform the trace information for new local CFG, with the offset of
    // current local BBNum.
    if (IsAlreadyPredicated) {
      // Adjust the local BBNum.
      CurTraceNum += MO->getTargetFlags();
      assert(CurTraceNum < NextTraceNum && "Trace number not sync!");
      CurTraceBitMap |= MO->getImm() << TraceNum;
    }

    annotateLocalCFGInfo(MO, CurTraceNum, CurTraceBitMap);
  }
}

unsigned HyperBlockFormation::mergeHyperBlockTrace(unsigned CurBBNum,
                                                   uint8_t NextTraceNum,
                                                   int64_t CurTraceBitMap,
                                                   TraceMapTy &ParentTraceMap)
                                                   const {
  AllTraceMapTy::const_iterator at = AllTraces.find(CurBBNum);

  // If current BB a Hyper-block?
  if (at == AllTraces.end() || at->second.size() == 1) return NextTraceNum;

  const uint8_t CurTraceNum = NextTraceNum - 1;
  // Merge the trace information of the hyper-block.
  const TraceMapTy &TraceMap = at->second;
  typedef TraceMapTy::const_iterator it;
  for (it I = TraceMap.begin(), E = TraceMap.end(); I != E; ++I) {
    uint8_t NewTraceNum = I->second.first + CurTraceNum;
    // There is an entry for the trace, allocate its number in current
    // hyper-block.
    if (NewTraceNum != CurTraceNum) ++NextTraceNum;

    // Move the bitmap.
    int64_t TraceBitMap = (I->second.second << CurTraceNum) | CurTraceBitMap;

    unsigned BBNum = I->first;
    // Insert the entry into the parent trace map.
    ParentTraceMap[BBNum] = std::make_pair(NewTraceNum, TraceBitMap);
  }

  return NextTraceNum;
}

Pass *llvm::createHyperBlockFormationPass() {
  return new HyperBlockFormation();
}
