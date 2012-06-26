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
#include "vtm/Passes.h"
#include "vtm/VerilogBackendMCTargetDesc.h"
#include "vtm/VInstrInfo.h"
#include "vtm/Utilities.h"

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineLoopInfo.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/MachineBranchProbabilityInfo.h"
#include "llvm/CodeGen/MachineDominators.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/Support/BranchProbability.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Format.h"
#define DEBUG_TYPE "vtm-hyper-block"
#include "llvm/Support/Debug.h"
#include <set>
#include <map>

using namespace llvm;

STATISTIC(BBsMerged, "Number of blocks are merged into hyperblock");

namespace {
struct HyperBlockFormation : public MachineFunctionPass {
  static char ID;

  const TargetInstrInfo *TII;
  MachineRegisterInfo *MRI;
  MachineLoopInfo *LI;
  MachineDominatorTree *DT;
  MachineBranchProbabilityInfo *MBPI;
  typedef std::set<unsigned> IntSetTy;
  typedef DenseMap<unsigned, IntSetTy> CFGMapTy;
  CFGMapTy CFGMap;

  // Trace number, Trace bit map.
  typedef std::pair<uint8_t, int64_t> TraceInfo;
  // MBB number -> trace number.
  typedef DenseMap<unsigned, TraceInfo> TraceMapTy;
  typedef DenseMap<unsigned, TraceMapTy> AllTraceMapTy;
  AllTraceMapTy AllTraces;
  uint8_t NextTraceNum;

  HyperBlockFormation() : MachineFunctionPass(ID), TII(0), MRI(0), LI(0),
                          DT(0), MBPI(0), NextTraceNum(0) {
    initializeHyperBlockFormationPass(*PassRegistry::getPassRegistry());
  }

  void getAnalysisUsage(AnalysisUsage &AU) const {
    MachineFunctionPass::getAnalysisUsage(AU);
    AU.addRequired<MachineBranchProbabilityInfo>();
    AU.addRequired<MachineDominatorTree>();
    AU.addPreserved<MachineDominatorTree>();
    AU.addRequired<MachineLoopInfo>();
  }

  static bool isBlockAlmostEmtpy(MachineBasicBlock *MBB) {
    if (MBB->empty()) return true;

    return MBB->getFirstNonPHI()->isTerminator();
  }

  void addPredToSet(MachineBasicBlock *MBB, IntSetTy &Set);
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

  bool simplifyCFG(MachineFunction &MF);

  MachineBasicBlock *getMergeDst(MachineBasicBlock *Src,
                                 VInstrInfo::JT &SrcJT,
                                 VInstrInfo::JT &DstJT);

  typedef SmallVectorImpl<MachineBasicBlock*> BBVecTy;
  bool mergeBlocks(MachineBasicBlock *MBB, BBVecTy &BBsToMerge);

  bool mergeBlock(MachineBasicBlock *FromBB, MachineBasicBlock *ToBB);

  void foldCFGEdge(MachineBasicBlock *EdgeSrc, VInstrInfo::JT &SrcJT,
                   MachineBasicBlock *EdgeDst, const VInstrInfo::JT &DstJT);

  bool mergeTrivialSuccBlocks(MachineBasicBlock *MBB);
  void PredicateBlock(unsigned ToBBNum, MachineOperand Cnd,
                      MachineBasicBlock *BB);

  // Delete the machine basic block and update the dominator tree.
  void deleteMBBAndUpdateDT(MachineBasicBlock *MBB);

  bool optimizeRetBB(MachineBasicBlock &RetBB);
  bool eliminateEmptyBlock(MachineBasicBlock *MBB);
  void moveRetValBeforePHI(MachineInstr *PHI, MachineBasicBlock *RetBB);
  const char *getPassName() const { return "Hyper-Block Formation Pass"; }
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
  INITIALIZE_PASS_DEPENDENCY(MachineDominatorTree)
  INITIALIZE_PASS_DEPENDENCY(MachineBranchProbabilityInfo)
INITIALIZE_PASS_END(HyperBlockFormation, "vtm-hyper-block",
                    "VTM - Hyper Block Formation", false, false)

void HyperBlockFormation::deleteMBBAndUpdateDT(MachineBasicBlock *MBB) {
  MachineDomTreeNode *MBBNode = DT->getNode(MBB);
  MachineDomTreeNode *IDomNode = MBBNode->getIDom();

  // Transfer the children of the node to be deleted to its immediate dominator.
  typedef MachineDomTreeNode::iterator child_iterator;
  while (MBBNode->getNumChildren())
    DT->changeImmediateDominator(MBBNode->getChildren().back(), IDomNode);

  DT->eraseNode(MBB);
  MBB->eraseFromParent();
}

bool HyperBlockFormation::mergeBlocks(MachineBasicBlock *MBB, BBVecTy &BBs) {
  bool ActuallyMerged = false;
  while (!BBs.empty()) {
    MachineBasicBlock *SuccBB = BBs.back();
    BBs.pop_back();

    ActuallyMerged |= mergeBlock(SuccBB, MBB);
  }

  return ActuallyMerged;
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

    if (isBlockAlmostEmtpy(Succ)) {
      BBsToMerge.push_back(Succ);
      continue;
    }
  }

  return mergeBlocks(MBB, BBsToMerge);
}

void HyperBlockFormation::addPredToSet(MachineBasicBlock *MBB, IntSetTy &Set) {
  typedef MachineBasicBlock::pred_iterator pred_it;
  for (pred_it I = MBB->pred_begin(), E = MBB->pred_end(); I != E; ++I) {
    MachineBasicBlock *PredBB = (*I);

    Set.insert(PredBB->getNumber());
  }
}

void HyperBlockFormation::buildCFGForBB(MachineBasicBlock *MBB) {
  IntSetTy &PredSet = CFGMap[MBB->getNumber()];
  addPredToSet(MBB, PredSet);
}

bool HyperBlockFormation::runOnMachineFunction(MachineFunction &MF) {
  TII = MF.getTarget().getInstrInfo();
  MRI = &MF.getRegInfo();
  LI = &getAnalysis<MachineLoopInfo>();
  MBPI = &getAnalysis<MachineBranchProbabilityInfo>();
  DT = &getAnalysis<MachineDominatorTree>();

  bool MakeChanged = false;
  AllTraces.clear();

  // Sort the BBs according they frequency.
  std::vector<MachineBasicBlock*> SortedBBs;
  CFGMap.clear();

  // Eliminate the empty blocks.
  while (simplifyCFG(MF))
    MakeChanged = true;

  MF.RenumberBlocks();

  if (MF.size() == 1) return MakeChanged;

  // Cache the original CFG.
  for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I) {
    SortedBBs.push_back(I);
    buildCFGForBB(I);
  }

  // Need to compare I to MF.end() in every iteration, because we will delete
  // the merged MBB.
  for (MachineFunction::iterator I = MF.begin(); I != MF.end(); ++I) {
    bool BlockMerged = false;
    MachineBasicBlock *MBB = I;

    unsigned CurBBNum = MBB->getNumber();
    // Initialize the local CFG.
    AllTraces[CurBBNum][CurBBNum] = std::make_pair(0, UINT64_C(0));
    NextTraceNum = 1;

    // Merge trivial blocks and hot blocks.
    do {
      hoistDatapathOpInSuccs(MBB, DT, MRI);
      MakeChanged |= BlockMerged = mergeTrivialSuccBlocks(MBB);
    } while (BlockMerged && NextTraceNum < 64);
  }

  for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I)
    if (I->succ_size() == 0 && !I->empty()
        && I->back().getOpcode() == VTM::VOpRet)
      MakeChanged |= optimizeRetBB(*I);

  while (simplifyCFG(MF))
    MakeChanged = true;

  MF.RenumberBlocks();

  return MakeChanged;
}

namespace {
class PHIEditor {
  typedef std::map<MachineBasicBlock*, unsigned> PHISrcMapTy;
  PHISrcMapTy SrcMap;
  MachineRegisterInfo &MRI;
  MachineInstr *PHI;
  unsigned BitWidth;
  const TargetRegisterClass *RC;

  void buildSrcMap() {
    assert(PHI->isPHI() && "Expect PHINode!");

    while (PHI->getNumOperands() > 1) {
      unsigned Idx = PHI->getNumOperands() - 2;
      MachineBasicBlock *SrcBB = PHI->getOperand(Idx + 1).getMBB();
      unsigned SrcReg = PHI->getOperand(Idx).getReg();
      bool inserted = SrcMap.insert(std::make_pair(SrcBB, SrcReg)).second;
      assert(inserted && "Entry already existed?");
      PHI->RemoveOperand(Idx + 1);
      PHI->RemoveOperand(Idx);
    }

    BitWidth = VInstrInfo::getBitWidth(PHI->getOperand(0));
    RC = MRI.getRegClass(PHI->getOperand(0).getReg());
  }

  void addNewSrcValFrom(unsigned NewSrcReg, MachineBasicBlock *EdgeSrc,
                        MachineBasicBlock *EdgeDst) {
    unsigned &Reg = SrcMap[EdgeSrc];

    // Identical incoming value found, nothing to do.
    if (Reg == NewSrcReg) return;

    // Simply create the new entry.
    if (Reg == 0) {
      Reg = NewSrcReg;
      return;
    }

    // Else we need to merge the value.
    MachineOperand Pred;
    MachineInstr *InsertPos
      = VInstrInfo::getEdgeCndAndInsertPos(EdgeSrc, EdgeDst, Pred);
    // Build the selection:
    // NewReg = (Branch taken from EdgeSrc to EdgeDst) ?
    //          Value from EdgeDst (NewSrcReg) : Value from EdgeSrc (Reg).
    unsigned NewReg = MRI.createVirtualRegister(RC);
    BuildMI(*EdgeSrc, InsertPos, DebugLoc(), VInstrInfo::getDesc(VTM::VOpSel))
      .addOperand(VInstrInfo::CreateReg(NewReg, BitWidth, true))
      .addOperand(Pred).addOperand(VInstrInfo::CreateReg(NewSrcReg, BitWidth))
      .addOperand(VInstrInfo::CreateReg(Reg, BitWidth))
      .addOperand(VInstrInfo::CreatePredicate())
      .addOperand(VInstrInfo::CreateTrace());

    Reg = NewReg;
  }

public:
  PHIEditor(MachineRegisterInfo &MRI, MachineInstr *PHI = 0)
    : MRI(MRI), PHI(PHI), BitWidth(0), RC(0) {
    if (PHI) buildSrcMap();
  }

  void reset(MachineInstr *P = 0) {
    SrcMap.clear();
    BitWidth = 0;
    RC = 0;
    if ((PHI = P))  buildSrcMap();    
  }

  void forwardAllIncomingFrom(MachineBasicBlock *PredBB) {
    PHISrcMapTy::iterator at = SrcMap.find(PredBB);
    assert(at != SrcMap.end() && "Bad PredBB!");

    unsigned Reg = at->second;
    SrcMap.erase(at);

    MachineInstr *DefMI = MRI.getVRegDef(Reg);
    assert(DefMI && "Not in SSA Form?");

    if (!DefMI->isPHI() || DefMI->getParent() != PredBB) {
      assert(DefMI->getParent() != PredBB && "Cannot PHI source values!");

      typedef MachineBasicBlock::pred_iterator pred_iterator;
      for (pred_iterator I = PredBB->pred_begin(), E = PredBB->pred_end();
           I != E; ++I) {
        addNewSrcValFrom(Reg, *I, PredBB);
      }
      return;
    }

    for (unsigned i = 1, e = DefMI->getNumOperands(); i != e; i += 2) {
      unsigned SrcReg = DefMI->getOperand(i).getReg();
      MachineBasicBlock *SrcBB = DefMI->getOperand(i + 1).getMBB();
      addNewSrcValFrom(SrcReg, SrcBB, PredBB);
    }
  }

  // Change the incoming value from SrcMBB to DstMBB.
  void forwardIncoming(MachineBasicBlock *SrcMBB, MachineBasicBlock *DstMBB,
                       const MachineOperand &CndDst2Src) {
    PHISrcMapTy::iterator at = SrcMap.find(SrcMBB);
    assert(at != SrcMap.end() && "Bad SrcMBB!");

    unsigned RegToForward = at->second;
    SrcMap.erase(at);

    unsigned &ExistedIncomingReg = SrcMap[DstMBB];

    // Identical incoming value found, nothing to do.
    if (ExistedIncomingReg == RegToForward) return;

    // Simply create the new entry.
    if (ExistedIncomingReg == 0) {
      ExistedIncomingReg = RegToForward;
      return;
    }

    // Else we need to merge the value.
    MachineInstr *InsertPos = DstMBB->getFirstInstrTerminator();
    // Build the selection:
    // NewReg = (Branch taken from EdgeSrc to EdgeDst) ?
    //          Value from EdgeDst (NewSrcReg) : Value from EdgeSrc (Reg).
    unsigned NewReg = MRI.createVirtualRegister(RC);
    BuildMI(*DstMBB, InsertPos, DebugLoc(), VInstrInfo::getDesc(VTM::VOpSel))
      .addOperand(VInstrInfo::CreateReg(NewReg, BitWidth, true))
      .addOperand(CndDst2Src)
      .addOperand(VInstrInfo::CreateReg(RegToForward, BitWidth))
      .addOperand(VInstrInfo::CreateReg(ExistedIncomingReg, BitWidth))
      .addOperand(VInstrInfo::CreatePredicate())
      .addOperand(VInstrInfo::CreateTrace());

    ExistedIncomingReg = NewReg;
  }

  template<typename iterator>
  void fillBySelfLoop(iterator begin, iterator end){
    unsigned PHIReg = PHI->getOperand(0).getReg();
    while (begin != end) {
      MachineBasicBlock *SrcBB = *begin++;      
      SrcMap.insert(std::make_pair(SrcBB, PHIReg));
    }
  }

  void removeSrc(MachineBasicBlock *From) {
    SrcMap.erase(From);
  }

  MachineInstr *flush() {
    if (SrcMap.size() > 1) {
      for (PHISrcMapTy::iterator I = SrcMap.begin(), E = SrcMap.end();I != E;++I){
        PHI->addOperand(VInstrInfo::CreateReg(I->second, BitWidth));
        PHI->addOperand(MachineOperand::CreateMBB(I->first));
      }

      MachineInstr *PHIToReturn = PHI;
      reset();
      return PHIToReturn;
    }

    unsigned PHIReg = PHI->getOperand(0).getReg();
    PHI->eraseFromParent();
    unsigned SrcVal = SrcMap.size() ? SrcMap.begin()->second : 0;
    MRI.replaceRegWith(PHIReg, SrcVal);
    reset();
    return 0;
  }

  static unsigned DoPHITranslation(unsigned Reg, MachineBasicBlock *CurBB,
                                   MachineBasicBlock *PredBB,
                                   MachineRegisterInfo &MRI) {
    MachineInstr *DefMI = MRI.getVRegDef(Reg);
    assert(DefMI && "Register not defined!");
    if (DefMI->getParent() == CurBB && DefMI->isPHI()) {
      for (unsigned i = 1, e = DefMI->getNumOperands(); i != e; i += 2)
        if (DefMI->getOperand(i + 1).getMBB() == PredBB)
          return DefMI->getOperand(i).getReg();

      llvm_unreachable("PredBB is not the incoming block?");
    }

    return Reg;
  }

  static void forwardIncomingFrom(MachineBasicBlock *MBB,
                                  MachineRegisterInfo &MRI) {
    typedef MachineBasicBlock::succ_iterator succ_it;
    typedef MachineBasicBlock::instr_iterator instr_iterator;
    PHIEditor Editor(MRI);
    // Fix the incoming value of PHIs.
    for (succ_it SI = MBB->succ_begin(), SE = MBB->succ_end(); SI != SE; ++SI) {
      MachineBasicBlock *Succ = *SI;
      for (instr_iterator MI = Succ->instr_begin(), ME = Succ->instr_end();
           MI != ME && MI->isPHI(); /*++MI*/) {
        Editor.reset(MI++);
        Editor.forwardAllIncomingFrom(MBB);
        Editor.flush();
      }
    }
  }

  static void forwardIncoming(MachineBasicBlock *SrcMBB,
                              MachineBasicBlock *DstMBB,
                              const MachineOperand &CndDst2Src,
                              MachineRegisterInfo &MRI) {
    typedef MachineBasicBlock::succ_iterator succ_it;
    typedef MachineBasicBlock::instr_iterator instr_iterator;
    PHIEditor Editor(MRI);
    // Fix the incoming value of PHIs.
    for (succ_it SI = SrcMBB->succ_begin(), SE = SrcMBB->succ_end();
         SI != SE; ++SI) {
      MachineBasicBlock *Succ = *SI;
      for (instr_iterator MI = Succ->instr_begin(), ME = Succ->instr_end();
           MI != ME && MI->isPHI(); /*++MI*/) {
        Editor.reset(MI++);
        Editor.forwardIncoming(SrcMBB, DstMBB, CndDst2Src);
        Editor.flush();
      }
    }
  }
};
}

bool HyperBlockFormation::eliminateEmptyBlock(MachineBasicBlock *MBB) {
  assert(isBlockAlmostEmtpy(MBB) && "Not an empty MBB!");
  // Cannot handle.
  if (MBB->succ_size() > 1 && MBB->instr_begin()->isPHI()) return false;
  
  VInstrInfo::JT EmptyBBJT, PredJT;
  if (VInstrInfo::extractJumpTable(*MBB, EmptyBBJT))
    return false;

  SmallVector<MachineBasicBlock*, 8> Preds(MBB->pred_begin(), MBB->pred_end()),
                                     Succs(MBB->succ_begin(), MBB->succ_end());

  typedef MachineBasicBlock::instr_iterator instr_iterator;
  typedef SmallVectorImpl<MachineBasicBlock*>::iterator pred_iterator;

  PHIEditor::forwardIncomingFrom(MBB, *MRI);

  PHIEditor Editor(*MRI);
  for (instr_iterator MI = MBB->instr_begin(), ME = MBB->instr_end();
       MI != ME && MI->isPHI(); /*++MI*/) {
    if (MRI->use_empty(MI->getOperand(0).getReg())) {
      MachineInstr *MIToDel = MI++;
      MIToDel->eraseFromParent();
      continue;
    }

    // FIXME: This not work if there is more then 1 successors.
    assert(Succs.size() == 1 && "Cannot handle yet!");
    Editor.reset(MI++);
    MachineBasicBlock *Succ = Succs.front(); 
    Editor.fillBySelfLoop(Succ->pred_begin(), Succ->pred_end());
    Editor.removeSrc(MBB);

    if (MachineInstr *MIToMove = Editor.flush()) {
      MIToMove->removeFromParent();
      Succ->insert(Succ->instr_begin(), MIToMove);
    }
  }

  TII->RemoveBranch(*MBB);
  for (pred_iterator PI = Preds.begin(), PE = Preds.end(); PI != PE; ++PI) {
    MachineBasicBlock *PredBB = *PI;
    PredJT.clear();
    bool fail = VInstrInfo::extractJumpTable(*PredBB, PredJT, false);
    assert(!fail && "Cannot extract jump table!");
    TII->RemoveBranch(*PredBB);
    foldCFGEdge(PredBB, PredJT, MBB, EmptyBBJT);
  }

  while (MBB->succ_size())
    MBB->removeSuccessor(MBB->succ_begin());

  //typedef SmallVectorImpl<MachineBasicBlock*>::iterator succ_iterator;
  // Move the PHIs to Successor blocks.

  assert(MBB->succ_empty() && MBB->pred_empty() && "Cannot fold MBB!");
  deleteMBBAndUpdateDT(MBB);
  return true;
}

bool HyperBlockFormation::simplifyCFG(MachineFunction &MF) {
  bool changed = false;

  MachineBasicBlock *Entry = MF.begin();
  MachineFunction::iterator I = MF.begin();
  while (I != MF.end()) {
    MachineBasicBlock *MBB = I++;

    // Directly eliminate the unreachable blocks.
    if (MBB->pred_empty() && MBB != Entry) {
      assert(MBB->succ_empty() && "Unreachable block has successor?");
      deleteMBBAndUpdateDT(MBB);
      changed = true;
      continue;
    }

    // We need to fix the terminators so the eliminateEmptyBlock can work
    // correctly.
    fixTerminators(MBB);
  }

  // Try to eliminate the almost empty blocks.
  I = MF.begin();
  while (I != MF.end()) {
    MachineBasicBlock *MBB = I++;

    if (isBlockAlmostEmtpy(MBB))
      changed |= eliminateEmptyBlock(MBB);
  }

  I = MF.begin();
  while (I != MF.end()) {
    MachineBasicBlock *MBB = I++;

    if (MBB->pred_size() != 1) continue;

    MachineBasicBlock *Pred = *MBB->pred_begin();

    if (Pred->succ_size() != 1) continue;

    // Do merge the block if there is a early return.
    if (Pred->getFirstInstrTerminator()->getOpcode() == VTM::VOpRet)
      continue;

    // Now the edge is a trivial edge, merge the MBBs.
    TII->RemoveBranch(*Pred);

    // And merge the block into its predecessor.
    Pred->splice(Pred->end(), MBB, MBB->begin(), MBB->end());
    Pred->transferSuccessorsAndUpdatePHIs(MBB);
    Pred->removeSuccessor(MBB);
    deleteMBBAndUpdateDT(MBB);
    changed = true;
  }

  return changed;
}

bool HyperBlockFormation::optimizeRetBB(MachineBasicBlock &RetBB) {
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
  if (RetVal) {
    // Try to move the VOpRetVal to the predecessor block.
    MachineOperand RetValMO = RetVal->getOperand(0);
    if (RetValMO.isReg()) {
      RetValDef = MRI->getVRegDef(RetValMO.getReg());
      assert(RetValDef && "Not in SSA Form!");
      // Cannot move to predecessor block.
      if (RetValDef->getParent() != &RetBB || !RetValDef->isPHI())
        return false;

      // The register has more than one use, it is not benefit to eliminate.
      if (!MRI->hasOneUse(RetValMO.getReg())) return false;
    }

    RetVal->eraseFromParent();

    if (!RetValDef || !RetValDef->isPHI()) {
      typedef MachineBasicBlock::pred_iterator pred_it;

      for (pred_it I = RetBB.pred_begin(), E = RetBB.pred_end(); I != E; ++I) {
        MachineOperand Pred = VInstrInfo::CreatePredicate();
        it IP = VInstrInfo::getEdgeCndAndInsertPos(*I, &RetBB, Pred);
        BuildMI(**I, IP, DebugLoc(), TII->get(VTM::VOpRetVal))
          .addOperand(RetValMO).addOperand(VInstrInfo::CreateImm(0, 8))
          .addOperand(Pred).addOperand(VInstrInfo::CreateTrace());
      }
    } else
      moveRetValBeforePHI(RetValDef, &RetBB);
  }

  // Also move the VOpRet to the predecessor block.
  if (RetBB.begin()->getOpcode() == VTM::VOpRet) {
    typedef SmallVectorImpl<MachineBasicBlock*>::iterator pred_it;
    SmallVector<MachineBasicBlock*, 4> Preds(RetBB.pred_begin(),
                                             RetBB.pred_end());

    for (pred_it I = Preds.begin(), E = Preds.end(); I != E; ++I) {
      MachineBasicBlock *PredBB = *I;
      MachineOperand Pred = VInstrInfo::CreatePredicate();
      it IP = VInstrInfo::getEdgeCndAndInsertPos(PredBB, &RetBB, Pred);
      BuildMI(**I, IP, DebugLoc(), TII->get(VTM::VOpRet))
        .addOperand(Pred).addOperand(VInstrInfo::CreateTrace());
      PredBB->removeSuccessor(&RetBB);
      // Also remove the corresponding branch condition.
      MachineBasicBlock::reverse_instr_iterator RI = PredBB->instr_rbegin();
      while (VInstrInfo::isBrCndLike(RI->getOpcode())) {
        if (RI->getOperand(1).getMBB() == &RetBB) {
          RI->eraseFromParent();
          break;
        }

        ++RI;
      }
    }

    RetBB.begin()->eraseFromParent();
  }

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
    it IP = VInstrInfo::getEdgeCndAndInsertPos(SrcBB, RetBB, Pred);

    BuildMI(*SrcBB, IP, DebugLoc(), TII->get(VTM::VOpRetVal))
      .addOperand(SrcMO).addOperand(VInstrInfo::CreateImm(0, 8))
      .addOperand(Pred).addOperand(VInstrInfo::CreateTrace());
  }

  assert(MRI->use_empty(PHI->getOperand(0).getReg()) && "Unexpected other use!");
  PHI->eraseFromParent();
}

MachineBasicBlock *HyperBlockFormation::getMergeDst(MachineBasicBlock *SrcBB,
                                                    VInstrInfo::JT &SrcJT,
                                                    VInstrInfo::JT &DstJT) {
  // Only handle simple case at the moment
  if (SrcBB->pred_size() != 1) return 0;

  MachineBasicBlock *DstBB = *SrcBB->pred_begin();
  // Do not change the parent loop of MBB.
  if (LI->getLoopFor(SrcBB) != LI->getLoopFor(DstBB)
      && !isBlockAlmostEmtpy(SrcBB))
    return 0;

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

void HyperBlockFormation::foldCFGEdge(MachineBasicBlock *EdgeSrc,
                                      VInstrInfo::JT &SrcJT,
                                      MachineBasicBlock *EdgeDst,
                                      const VInstrInfo::JT &DstJT) {
  typedef VInstrInfo::JT::iterator jt_it;
  typedef VInstrInfo::JT::const_iterator const_jt_it;
  jt_it at = SrcJT.find(EdgeDst);
  assert(at != SrcJT.end() && "ToBB not branching to FromBB?");
  MachineOperand PredCnd = at->second;

  SmallVector<MachineOperand, 1> PredVec(1, PredCnd);

  typedef DenseMap<MachineBasicBlock*, Probability> ProbMapTy;
  ProbMapTy BBProbs;
  uint32_t WeightLCM = 1;
  // The probability of the edge from ToBB to FromBB.
  BranchProbability MergedBBProb = MBPI->getEdgeProbability(EdgeSrc, EdgeDst);

  for (const_jt_it I = DstJT.begin(),E = DstJT.end(); I != E; ++I){
    MachineBasicBlock *Succ = I->first;

    Probability SuccProb = MBPI->getEdgeProbability(EdgeDst, Succ);
    // In the merged block, sum of probabilities of FromBB's successor should
    // be the original edge probability from ToBB to FromBB.
    SuccProb *= MergedBBProb;
    BBProbs.insert(std::make_pair(Succ, SuccProb));
    WeightLCM = lcm(WeightLCM, SuccProb.getDenominator());
  }

  typedef MachineBasicBlock::succ_iterator succ_it;
  for (unsigned i = EdgeSrc->succ_size(); i > 0; --i) {
    succ_it SuccIt = EdgeSrc->succ_begin() + i - 1;
    MachineBasicBlock *Succ = *SuccIt;
    if (Succ != EdgeDst) {
      Probability &SuccProb = BBProbs[Succ];
      SuccProb += MBPI->getEdgeProbability(EdgeSrc, Succ);
      WeightLCM = lcm(WeightLCM, SuccProb.getDenominator());
    }

    EdgeSrc->removeSuccessor(SuccIt);
  }

  // Rebuild the successor list with new weights.
  for (ProbMapTy::iterator I = BBProbs.begin(), E = BBProbs.end(); I != E; ++I){
    Probability Prob = I->second;
    uint32_t w = Prob.getNumerator() * WeightLCM / Prob.getDenominator();
    DEBUG(dbgs() << I->first->getName() << ' ' << I->second << ' ' << w <<'\n');
    EdgeSrc->addSuccessor(I->first, w);
  }

  // Do not jump to FromBB any more.
  SrcJT.erase(EdgeDst);
  // We had assert FromJT not contains FromBB.
  // FromJT.erase(FromBB);

  // Build the new Jump table.
  for (const_jt_it I = DstJT.begin(), E = DstJT.end(); I != E; ++I) {
    MachineBasicBlock *Succ = I->first;
    // And predicate the jump table.
    MachineOperand DstPred = VInstrInfo::MergePred(I->second, PredCnd, *EdgeSrc,
                                                   EdgeSrc->end(), MRI, TII,
                                                   VTM::VOpAnd);

    jt_it at = SrcJT.find(Succ);
    // If the entry already exist in target jump table, merge it with opcode OR.
    if (at != SrcJT.end())
      at->second = VInstrInfo::MergePred(at->second, DstPred, *EdgeSrc,
                                         EdgeSrc->end(), MRI, TII, VTM::VOpOr);
    else // Simply insert the entry.
      SrcJT.insert(std::make_pair(Succ, DstPred));
  }

  // Re-insert the jump table.
  VInstrInfo::insertJumpTable(*EdgeSrc, SrcJT, DebugLoc());
  ++BBsMerged;
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

  PredicateBlock(ToBB->getNumber(), PredCnd, FromBB);

  // And merge the block into its predecessor.
  ToBB->splice(ToBB->end(), FromBB, FromBB->begin(), FromBB->end());

  foldCFGEdge(ToBB, ToJT, FromBB, FromJT);
  PHIEditor::forwardIncoming(FromBB, ToBB, PredCnd, *MRI);

  SmallVector<MachineOperand, 1> PredVec(1, PredCnd);
  // The Block is dead, remove all its successor.
  for (jt_it I = FromJT.begin(),E = FromJT.end(); I != E; ++I){
    MachineBasicBlock *Succ = I->first;
    FromBB->removeSuccessor(Succ);
  }

  assert(FromBB->empty() && FromBB->pred_empty() && FromBB->succ_empty()
         && "BB not fully merged!");
  deleteMBBAndUpdateDT(FromBB);
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
