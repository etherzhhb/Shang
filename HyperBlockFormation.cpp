//===- HyperBlockFormation.cpp - Merge Fall Through Blocks ---*- C++ -*-===//
//
//                            The Verilog Backend
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements a pass that merge the fall through blocks into its
// predecessor blocks to increase instruction level parallelism.
//
//===----------------------------------------------------------------------===//

#include "BBDelayAnalysis.h"

#include "vtm/Passes.h"
#include "vtm/VerilgoBackendMCTargetDesc.h"
#include "vtm/VInstrInfo.h"
#include "vtm/MicroState.h"

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
#define DEBUG_TYPE "vtm-merge-fallthroughs"
#include "llvm/Support/Debug.h"
#include <set>
#include <map>

using namespace llvm;

STATISTIC(BBMerged, "Number of blocks merged into hyperblock");

namespace {
struct HyperBlockFormation : public MachineFunctionPass {
  static char ID;

  const TargetInstrInfo *TII;
  MachineRegisterInfo *MRI;
  MachineLoopInfo *LI;
  MachineBranchProbabilityInfo *MBPI;
  MachineBlockFrequencyInfo *MBFI;
  BBDelayAnalysis *BBDelay;

  HyperBlockFormation()
    : MachineFunctionPass(ID), TII(0), MRI(0), LI(0), MBPI(0), MBFI(0),
      BBDelay(0) {
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

  bool runOnMachineFunction(MachineFunction &MF);
  MachineBasicBlock *getMergeDst(MachineBasicBlock *Src,
                                 VInstrInfo::JT &SrcJT,
                                 VInstrInfo::JT &DstJT);

  bool mergeBlock(MachineBasicBlock *FromBB,
                             MachineBasicBlock *ToBB);

  bool mergeSuccBlocks(MachineBasicBlock *MBB);

  void PredicateBlock(MachineOperand Cnd, MachineBasicBlock *BB);

  bool mergeReturnBB(MachineFunction &MF, MachineBasicBlock &RetBB,
                     const TargetInstrInfo *TII);
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

bool HyperBlockFormation::mergeSuccBlocks(MachineBasicBlock *MBB) {
  // Collect the successor blocks to merge.
  std::vector<MachineBasicBlock*> BBs;

  VInstrInfo::JT CurJT, SuccJT;
  BlockFrequency BBFreq = MBFI->getBlockFreq(MBB);

  // Latency statistics.
  CycleLatencyInfo CL;
  unsigned MBBDelay = CL.computeLatency(*MBB), MaxMergeLatnecy = 0;
  uint64_t AvePathDelay = 0;
  std::map<MachineBasicBlock*, int64_t> BBSavedCyles;

  typedef MachineBasicBlock::succ_iterator succ_iterator;
  for (succ_iterator I = MBB->succ_begin(), E = MBB->succ_end(); I != E; ++I) {
    MachineBasicBlock *SuccBB = *I;
    SuccJT.clear();
    CurJT.clear();

    uint64_t BrFreq =
      (BBFreq * MBPI->getEdgeProbability(MBB, SuccBB)).getFrequency();

    MachineBasicBlock *MergeDst = getMergeDst(SuccBB, SuccJT, CurJT);
    if (!MergeDst) {
      // SuccBB can not be merged, the path delay is simply the delay of current
      // BB.
      AvePathDelay += MBBDelay * BrFreq;
      continue;
    }

    assert(MergeDst == MBB && "Succ have more than one Pred?");
    unsigned MergeLatency = CL.computeLatency(*SuccBB);
    MaxMergeLatnecy = std::max(MaxMergeLatnecy, MergeLatency);
    // Compute the original path delay including current BB and success BB.
    unsigned PathDelay = MBBDelay + BBDelay->getBBDelay(SuccBB);
    AvePathDelay += PathDelay * BrFreq;
    BBs.push_back(SuccBB);
  }

  if (BBs.empty()) return false;

  // Only merge the SuccBB into current BB if the merge is beneficial.
  // TODO: Remove the SuccBB with smallest beneficial and try again.
  if (AvePathDelay < MaxMergeLatnecy * BBFreq.getFrequency())
    return false;

  bool ActuallyMerged = false;
  while (!BBs.empty()) {
    MachineBasicBlock *SuccMBB = BBs.back();
    BBs.pop_back();

    ActuallyMerged |= mergeBlock(SuccMBB, MBB);
  }

  // Update the delay.
  BBDelay->updateDelay(MBB, CL.computeLatency(*MBB, true));
  return ActuallyMerged;
}

bool HyperBlockFormation::runOnMachineFunction(MachineFunction &MF) {
  TII = MF.getTarget().getInstrInfo();
  MRI = &MF.getRegInfo();
  LI = &getAnalysis<MachineLoopInfo>();
  MBPI = &getAnalysis<MachineBranchProbabilityInfo>();
  MBFI = &getAnalysis<MachineBlockFrequencyInfo>();
  BBDelay = &getAnalysis<BBDelayAnalysis>();

  bool MakeChanged = false;
  typedef MachineFunction::reverse_iterator rev_it;

  MF.RenumberBlocks();

  // Sort the BBs according they frequency.
  std::vector<MachineBasicBlock*> SortedBBs;

  for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I)
    SortedBBs.push_back(I);

  std::sort(SortedBBs.begin(), SortedBBs.end(), sort_bb_by_freq(MBFI));

  while (!SortedBBs.empty()) {
    bool BlockMerged = false;
    MachineBasicBlock *MBB = SortedBBs.back();
    SortedBBs.pop_back();

    do {
      MakeChanged |= BlockMerged = mergeSuccBlocks(MBB);
    } while (BlockMerged);
  }

  for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I) {
    if (I->succ_size() == 0 && !I->empty()
        && I->back().getOpcode() == VTM::VOpRet)
      MakeChanged |= mergeReturnBB(MF, *I, TII);
  }

  // Optimize the cfg, but do not perform tail merge.
  BranchFolder BF(true, true);
  MakeChanged |= BF.OptimizeFunction(MF, TII, MF.getTarget().getRegisterInfo(),
                                     getAnalysisIfAvailable<MachineModuleInfo>());

  MF.RenumberBlocks();

  return MakeChanged;
}

bool HyperBlockFormation::mergeReturnBB(MachineFunction &MF,
                                           MachineBasicBlock &RetBB,
                                           const TargetInstrInfo *TII) {
  // Return port do not have any successor.
  if (RetBB.succ_size()) return false;

  assert(!RetBB.empty() && "Unexpected empty return block!");

  MachineInstr *RetValPHI = 0, *RetVal = 0, *Ret = 0;

  MachineBasicBlock::iterator I = RetBB.begin();
  if (I->isPHI()) {
    // Too much PHIs.
    if (RetValPHI) return false;

    RetValPHI = I;
    ++I;
  }

  if (I->getOpcode() == VTM::VOpRetVal) {
    RetVal = I;
    ++I;
  }

  if (I->getOpcode() == VTM::VOpRet) {
    Ret = I;
    ++I;
  }

  // Do not merge, if there are any unexpected instructions.
  if (I != RetBB.end() || !Ret) return false;

  unsigned RetReg = 0;
  if (RetVal && RetVal->getOperand(0).isReg())
    RetReg = RetVal->getOperand(0).getReg();

  // Build the income value map, it is enough to only store the register number
  // since the PHI node will not change the bit width of the register.
  std::map<MachineBasicBlock*, unsigned> SrcMOs;
  if (RetValPHI)
    for (unsigned i = 1, e = RetValPHI->getNumOperands(); i != e; i += 2)
      SrcMOs.insert(std::make_pair(RetValPHI->getOperand(i + 1).getMBB(),
                                   RetValPHI->getOperand(i).getReg()));

  // Replace branch to retbb by the return instruction
  SmallVector<MachineBasicBlock*, 8> PredBBs(RetBB.pred_begin(),
                                             RetBB.pred_end());
  while (!PredBBs.empty()) {
    MachineBasicBlock *PredBB = PredBBs.pop_back_val();
    // Do not merge RetBB into loops
    if (LI->getLoopFor(PredBB))
      continue;

    MachineBasicBlock::iterator FirstTerm = PredBB->getFirstTerminator();
    for (MachineBasicBlock::iterator PI = FirstTerm, PE = PredBB->end();
         PI != PE ; ++PI) {
      MachineInstr *Term = PI;
      assert(VInstrInfo::isBrCndLike(Term->getOpcode()) && "Unexpected opcode!");
      if (Term->getOperand(1).getMBB() == &RetBB) {
        SmallVector<MachineOperand, 1> Cnds(1, Term->getOperand(0));

        if (RetVal) {
          std::map<MachineBasicBlock*, unsigned>::iterator at
            = SrcMOs.find(PredBB);
          if (at != SrcMOs.end()) {
            RetReg = at->second;
            // Remove the incoming value from the PHI.
            for (unsigned i = 1, e = RetValPHI->getNumOperands();i != e;i += 2)
              if (RetValPHI->getOperand(i + 1).getMBB() == PredBB) {
                RetValPHI->RemoveOperand(i);
                RetValPHI->RemoveOperand(i);
                break;
              }
          }

          MachineInstr *NewRetVal = MF.CloneMachineInstr(RetVal);
          // Update the return value and the predicate.
          NewRetVal->getOperand(0).ChangeToRegister(RetReg, false);
          TII->PredicateInstruction(NewRetVal, Cnds);
          // Insert the returnvalue.
          PredBB->insert(FirstTerm, NewRetVal);
        }

        // Predicate and insert the return
        MachineInstr *NewRet = MF.CloneMachineInstr(Ret);
        //TII->PredicateInstruction(NewRet, Cnds);
        // DirtyHack: PredicateInstruction will not predicate return, predicate
        // it manually
        MachineOperand &RetPred = NewRet->getOperand(0);
        RetPred.setTargetFlags(Cnds.front().getTargetFlags());
        RetPred.ChangeToRegister(Cnds.front().getReg(), false);
        // Insert it into the predecessor.
        PredBB->insert(FirstTerm, NewRet);

        // The original branch instruction is not used any more.
        Term->eraseFromParent();
        // Go on process next predicate.
        break;
      }
    }

    // We had merge the retbb into the its predecessor, and not jumping to the
    // retbb anymore.
    PredBB->removeSuccessor(&RetBB);
  }

  // Clear the return block if it is unreachable
  if (RetBB.pred_empty()) RetBB.clear();
  else if (RetBB.pred_size() == 1 && RetValPHI) {
    // The PHI should be delete if there is only 1 incoming bb.
    RetVal->getOperand(0).ChangeToRegister(RetValPHI->getOperand(1).getReg(),
                                           false);
    RetValPHI->removeFromParent();
  }

  return true;
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
                                     MachineBasicBlock *ToBB) {
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

  if (!VInstrInfo::isAlwaysTruePred(PredCnd))
    PredicateBlock(PredCnd, FromBB);

  // And merge the block into its predecessor.
  ToBB->splice(ToBB->end(), FromBB, FromBB->begin(), FromBB->end());

  SmallVector<MachineOperand, 1> PredVec(1, PredCnd);
  std::set<MachineBasicBlock*> NewSuccs;

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
    dbgs() << I->first->getName() << ' ' << I->second << ' ' << w <<'\n';
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
  ++BBMerged;
  return true;
}

void HyperBlockFormation::PredicateBlock(MachineOperand Pred,
                                            MachineBasicBlock *MBB ){
  typedef std::map<unsigned, unsigned> PredMapTy;
  PredMapTy PredMap;
  SmallVector<MachineOperand, 1> PredVec(1, Pred);

  // Predicate the Block.
  for (MachineBasicBlock::iterator I = MBB->begin(), E = MBB->end();
       I != E; ++I) {
    if (I->isDebugValue())
      continue;

    if (VInstrInfo::isDatapath(I->getOpcode()))
      continue;

    if (TII->isPredicated(I)) {
      ucOperand *MO = cast<ucOperand>(VInstrInfo::getPredOperand(I));
      unsigned k = MO->getReg() << 1 | (MO->isPredicateInverted() ? 1 :0 );
      unsigned &Reg = PredMap[k];
      if (!Reg)
        Reg = VInstrInfo::MergePred(*MO, Pred, *MBB, I, MRI,
                                    TII, VTM::VOpAnd).getReg();

      MO->ChangeToRegister(Reg, false);
      MO->setTargetFlags(1);
    } else if (I->getOpcode() <= TargetOpcode::COPY) {
      MachineInstr *PseudoInst = I;
      ++I; // Skip current instruction, we may change it.
      PseudoInst = VInstrInfo::PredicatePseudoInstruction(PseudoInst,
                                                          PredVec);
      if (!PseudoInst) {
#ifndef NDEBUG
        dbgs() << "Unable to predicate " << *I << "!\n";
#endif
        llvm_unreachable(0);
      }
      I = PseudoInst;
    } else if (!TII->PredicateInstruction(I, PredVec)) {
#ifndef NDEBUG
      dbgs() << "Unable to predicate " << *I << "!\n";
#endif
      llvm_unreachable(0);
    }
  }
}

Pass *llvm::createHyperBlockFormationPass() {
  return new HyperBlockFormation();
}
