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

  VInstrInfo::JT PredJT, CurJT;

  MachineBasicBlock *Pred = *MBB->pred_begin();
  // Do not change the parent loop of MBB.
  if (LI->getLoopFor(MBB) != LI->getLoopFor(Pred)) return false;

  // Do not mess up with strange CFG.
  if (VInstrInfo::extractJumpTable(*Pred, PredJT)) return false;
  if (VInstrInfo::extractJumpTable(*MBB, CurJT)) return false;

  // Do not mess up with self loop.
  if (CurJT.count(MBB)) return false;

  // We need to predicate the block when merging it.
  for (MachineBasicBlock::iterator I = MBB->begin(), E = MBB->end();I != E;++I){
    MachineInstr *MI = I;
    if (!TII->isPredicable(MI))
      return false;
  }

  unsigned IncreasedLatency = 0;
  CompLatency CL;
  unsigned PredLatency = CL.computeLatency(*Pred);
  unsigned MergedLatency = CL.computeLatency(*MBB);
  if (MergedLatency > PredLatency)
    IncreasedLatency = MergedLatency - PredLatency;
  double IncreaseRate = double(IncreasedLatency)/double(PredLatency);

  DEBUG(dbgs() << "Merging BB#" << MBB->getNumber() << " To BB#"
         << Pred->getNumber() << " IncreasedLatency " << IncreasedLatency
         << ' ' << int(IncreaseRate * 100) << "%\n");

  return IncreasedLatency < 5 && IncreaseRate < 0.2;
}

void MergeFallThroughBlocks::mergeFallThroughBlock(MachineBasicBlock *FromBB) {
  MachineBasicBlock *ToBB = *FromBB->pred_begin();
  VInstrInfo::JT FromJT, ToJT;

  // Do not mess up with strange CFG.
  if (VInstrInfo::extractJumpTable(*ToBB, ToJT)) return;

  if (VInstrInfo::extractJumpTable(*FromBB, FromJT)) return;
  assert(!FromJT.count(FromBB) && "Unexpected self loop!");

  TII->RemoveBranch(*ToBB);
  TII->RemoveBranch(*FromBB);

  // Get the condition of jumping from ToBB to FromBB
  typedef VInstrInfo::JT::iterator jt_it;
  jt_it at = ToJT.find(FromBB);
  assert(at != ToJT.end() && "ToBB not branching to FromBB?");
  MachineOperand JumpingCnd = at->second;
  SmallVector<MachineOperand, 1> JumpingCndVec(1, JumpingCnd);

  typedef std::map<unsigned, unsigned> PredMapTy;
  PredMapTy PredMap;

  assert(JumpingCnd.getReg() != 0 && "Unexpected unconditional branch!");
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
        Reg = VInstrInfo::MergePred(*MO, JumpingCnd, *FromBB, I, MRI,
                                    TII, VTM::VOpAnd).getReg();

      MO->ChangeToRegister(Reg, false);
      MO->setTargetFlags(1);
    } else if (I->getOpcode() <= TargetOpcode::COPY) {
      MachineInstr *PseudoInst = I;
      ++I; // Skip current instruction, we may change it.
      PseudoInst = VInstrInfo::PredicatePseudoInstruction(PseudoInst,
                                                          JumpingCndVec);
      if (!PseudoInst) {
#ifndef NDEBUG
        dbgs() << "Unable to predicate " << *I << "!\n";
#endif
        llvm_unreachable(0);
      }
      I = PseudoInst;
    } else if (!TII->PredicateInstruction(I, JumpingCndVec)) {
#ifndef NDEBUG
      dbgs() << "Unable to predicate " << *I << "!\n";
#endif
      llvm_unreachable(0);
    }
  }

  // And merge the block into its predecessor.
  ToBB->splice(ToBB->end(), FromBB, FromBB->begin(), FromBB->end());

  std::set<MachineBasicBlock*> NewSuccs;
  for (jt_it I = FromJT.begin(),E = FromJT.end(); I != E; ++I){
    MachineBasicBlock *Succ = I->first;
    // Merge the PHINodes.
    VInstrInfo::mergePHISrc(Succ, FromBB, ToBB, *MRI, JumpingCndVec);
    // We had assert FromJT not contains FromBB, so we do not need to worry
    // about adding FromBB to the successor of ToBB again.
    // Is it the successor of FromBB become the new successor of ToBB?
    if (!ToJT.count(Succ)) ToBB->addSuccessor(Succ);

    FromBB->removeSuccessor(Succ);

    // And predicate the jump table.
    I->second = VInstrInfo::MergePred(I->second, JumpingCnd, *ToBB, ToBB->end(),
                                      MRI, TII, VTM::VOpAnd);
  }

  // Do not jump to FromBB any more.
  ToBB->removeSuccessor(FromBB);
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
  ++NumFallThroughMerged;
}

Pass *llvm::createMergeFallThroughBlocksPass() {
  return new MergeFallThroughBlocks();
}
