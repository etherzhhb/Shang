//===- MergeFallThroughBlocks.cpp - Merge Fall Through Blocks ---*- C++ -*-===//
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

#include "vtm/Passes.h"
#include "vtm/VTM.h"
#include "vtm/VInstrInfo.h"
#include "vtm/MicroState.h"

#include "llvm/../../lib/CodeGen/BranchFolding.h"

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineLoopInfo.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/CommandLine.h"
#define DEBUG_TYPE "vtm-merge-fallthroughs"
#include "llvm/Support/Debug.h"
#include <set>
#include <map>

using namespace llvm;

STATISTIC(NumFallThroughMerged,
          "VTM - Number of Fall Through Blocks Merged");

namespace {
struct MergeFallThroughBlocks : public MachineFunctionPass {
  static char ID;

  const TargetInstrInfo *TII;
  MachineRegisterInfo *MRI;
  MachineLoopInfo *LI;

  MergeFallThroughBlocks() : MachineFunctionPass(ID), TII(0), MRI(0) {}

  void getAnalysisUsage(AnalysisUsage &AU) const {
    MachineFunctionPass::getAnalysisUsage(AU);
    AU.addRequired<MachineLoopInfo>();
  }

  bool runOnMachineFunction(MachineFunction &MF);
  bool canMerge(MachineBasicBlock *MBB);

  void mergeFallThroughBlock(MachineBasicBlock *MBB);
};
}

char MergeFallThroughBlocks::ID = 0;

bool MergeFallThroughBlocks::runOnMachineFunction(MachineFunction &MF) {
  TII = MF.getTarget().getInstrInfo();
  MRI = &MF.getRegInfo();
  LI = &getAnalysis<MachineLoopInfo>();

  typedef MachineFunction::reverse_iterator rev_it;
  for (rev_it I = MF.rbegin(), E = MF.rend(); I != E; ++I) {
    MachineBasicBlock *MBB = &*I;

    if (canMerge(MBB))
      mergeFallThroughBlock(MBB);
  }

  // Tail merge tend to expose more if-conversion opportunities.
  BranchFolder BF(true);
  BF.OptimizeFunction(MF, TII, MF.getTarget().getRegisterInfo(),
                      getAnalysisIfAvailable<MachineModuleInfo>());

  MF.RenumberBlocks();
  return true;
}

bool MergeFallThroughBlocks::canMerge(MachineBasicBlock *MBB) {
  // Only handle simple case at the moment
  if (MBB->pred_size() != 1) return false;

  MachineBasicBlock *Pred = *MBB->pred_begin(), *PredTBB = 0, *PredFBB = 0,
                            *TBB = 0, *FBB = 0;
  SmallVector<MachineOperand, 1> Cnd;

  // Do not mess up with strange CFG.
  if (TII->AnalyzeBranch(*Pred, PredTBB, PredFBB, Cnd)) return false;

  if (TII->AnalyzeBranch(*MBB, TBB, FBB, Cnd)) return false;

  // Do not change the parent loop of MBB.
  if (LI->getLoopFor(MBB) != LI->getLoopFor(Pred)) return false;

  // Make sure the MBB and TBB are the same block.
  if (PredTBB != MBB)
    std::swap(PredTBB, PredFBB);

  assert(PredTBB == MBB && "ToBB is not the predeccessor of FromBB?");

  // Try to avoid produce more than 2 successor after merge.
  if (PredFBB && TBB && FBB && (PredFBB != TBB && PredFBB != FBB))
    return false;

  // We need to predicate the block when merging it.
  for (MachineBasicBlock::iterator I = MBB->begin(), E = MBB->end();I != E;++I){
    MachineInstr *MI = I;
    if (!TII->isPredicable(MI))
      return false;
  }

  return true;
}

void MergeFallThroughBlocks::mergeFallThroughBlock(MachineBasicBlock *FromBB) {
  MachineBasicBlock *ToBB = *FromBB->pred_begin(), *ToTBB = 0, *ToFBB = 0,
                    *FromTBB = 0, *FromFBB = 0;
  SmallVector<MachineOperand, 1> FromBBCnd, Cond;

  if (TII->AnalyzeBranch(*ToBB, ToTBB, ToFBB, FromBBCnd))
    return;

  if (TII->AnalyzeBranch(*FromBB, FromTBB, FromFBB, Cond))
    return;

  // Make sure the MBB and TBB are the same block.
  if (ToTBB != FromBB) {
    std::swap(ToTBB, ToFBB);
    TII->ReverseBranchCondition(FromBBCnd);
  }

  assert(ToTBB == FromBB && "ToBB is not the predeccessor of FromBB?");

  TII->RemoveBranch(*ToBB);
  TII->RemoveBranch(*FromBB);

  typedef std::map<unsigned, unsigned> PredMapTy;
  PredMapTy PredMap;

  if (!FromBBCnd.empty()) {
    // Predicate the Block.
    for (MachineBasicBlock::iterator I = FromBB->begin(), E = FromBB->end();
         I != E; ++I) {
      if (I->isDebugValue())
        continue;

      if (TII->isPredicated(I)) {
        ucOperand *MO = cast<ucOperand>(VInstrInfo::getPredOperand(I));
        unsigned k = MO->getReg() << 1 | (MO->isPredicateInverted() ? 1 :0 );
        unsigned &Reg = PredMap[k];
        if (!Reg)
          Reg = VInstrInfo::MergePred(*MO, FromBBCnd.front(), *FromBB, I, MRI, TII).getReg();

        MO->ChangeToRegister(Reg, false);
        MO->setTargetFlags(1);
      } else if (I->getOpcode() <= TargetOpcode::COPY) {
        MachineInstr *PseudoInst = I;
        ++I; // Skip current instruction, we may change it.
        PseudoInst = VInstrInfo::PredicatePseudoInstruction(PseudoInst, TII, FromBBCnd);
        if (!PseudoInst) {
#ifndef NDEBUG
          dbgs() << "Unable to predicate " << *I << "!\n";
#endif
          llvm_unreachable(0);
        }
        I = PseudoInst;
      } else if (!TII->PredicateInstruction(I, FromBBCnd)) {
#ifndef NDEBUG
        dbgs() << "Unable to predicate " << *I << "!\n";
#endif
        llvm_unreachable(0);
      }
    }
  }

  // And merge the block into its predecessor.
  ToBB->splice(ToBB->end(), FromBB, FromBB->begin(), FromBB->end());

  VInstrInfo::mergePHISrc(FromTBB, FromBB, ToBB, *MRI, FromBBCnd);
  if (FromFBB)
    VInstrInfo::mergePHISrc(FromFBB, FromBB, ToBB, *MRI, FromBBCnd);

  // MBB is unreachable now.
  ToBB->removeSuccessor(ToTBB);
  if (FromTBB && FromTBB != ToFBB) {
    ToBB->addSuccessor(FromTBB);
    FromBB->removeSuccessor(FromTBB);
  }
  // Do not add/remove the same block twice.
  if (FromFBB && FromFBB != ToFBB && FromTBB != FromFBB) {
    ToBB->addSuccessor(FromFBB);
    FromBB->removeSuccessor(FromFBB);
  }

  VInstrInfo::MergeBranches(ToFBB, FromBBCnd, FromTBB, FromFBB, Cond, TII);
  VInstrInfo::BuildCondition(*ToBB, Cond, MRI, TII);

  TII->InsertBranch(*ToBB, FromTBB, FromFBB, Cond, DebugLoc());
  ++NumFallThroughMerged;
}

Pass *llvm::createMergeFallThroughBlocksPass() {
  return new MergeFallThroughBlocks();
}
