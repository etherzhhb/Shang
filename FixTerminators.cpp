//===- FixMachineCode.cpp - Fix The Terminators in Machine Code -*- C++ -*-===//
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
// This file implements a pass that add branch instruction that branch to fall
// through block explicitly.
//
//===----------------------------------------------------------------------===//

#include "vtm/Utilities.h"
#include "vtm/VerilogBackendMCTargetDesc.h"
#include "vtm/VInstrInfo.h"

#include "llvm/../../lib/CodeGen/BranchFolding.h"

#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/CommandLine.h"
#define DEBUG_TYPE "vtm-fix-terminator"
#include "llvm/Support/Debug.h"
#include <set>

using namespace llvm;
STATISTIC(UnconditionalBranches,
  "Number of unconditionnal branches inserted for fall through edges");
STATISTIC(Unreachables,
  "Number of Unreachable inserted for machine basic block without sucessor");

namespace llvm {
void fixTerminators(MachineBasicBlock *MBB, const TargetInstrInfo *TII) {
  SmallPtrSet<MachineBasicBlock*, 2> MissedSuccs;
  MissedSuccs.insert(MBB->succ_begin(), MBB->succ_end());
  MachineInstr *FirstTerminator = 0;

  for (MachineBasicBlock::iterator II = MBB->getFirstTerminator(),
       IE = MBB->end(); II != IE; ++II) {
    MachineInstr *Inst = II;
    if (!VInstrInfo::isBrCndLike(Inst->getOpcode())) continue;

    MachineBasicBlock *TargetBB = Inst->getOperand(1).getMBB();
    MachineOperand Cnd = Inst->getOperand(0);
    //bool inserted;
    //jt_it at;
    //tie(at, inserted) = Table.insert(std::make_pair(TargetBB, Cnd));
    // BranchFolding may generate code that jumping to same bb with multiple
    // instruction, merge the condition.
    //if (!inserted) {
    //  at->second = VInstrInfo::MergePred(Cnd, at->second, *MBB,
    //                                     MBB->getFirstTerminator(), &MRI,
    //                                     TII, VTM::VOpOr);
    //}

    // Change the unconditional branch after conditional branch to
    // conditional branch.
    if (FirstTerminator && VInstrInfo::isUnConditionalBranch(Inst)){
      MachineOperand &TrueCnd = FirstTerminator->getOperand(0);
      MachineOperand &FalseCnd = Inst->getOperand(0);
      TrueCnd.setIsKill(false);
      FalseCnd.setReg(TrueCnd.getReg());
      FalseCnd.setTargetFlags(TrueCnd.getTargetFlags());
      VInstrInfo::ReversePredicateCondition(FalseCnd);
    }

    FirstTerminator = Inst;
    MissedSuccs.erase(TargetBB);
  }

  // Make sure each basic block have a terminator.
  if (!MissedSuccs.empty()) {
    assert(MissedSuccs.size() == 1 && "Fall through to multiple blocks?");
    ++UnconditionalBranches;
    MachineOperand Cnd = VInstrInfo::CreatePredicate();
    if (FirstTerminator) {
      MachineOperand &TrueCnd = FirstTerminator->getOperand(0);
      assert(TrueCnd.getReg() != 0 && "Two unconditional branch?");
      // We will use the register somewhere else
      TrueCnd.setIsKill(false);
      Cnd = TrueCnd;
      VInstrInfo::ReversePredicateCondition(Cnd);
    }
    BuildMI(MBB, DebugLoc(), TII->get(VTM::VOpToStateb))
      .addOperand(Cnd).addMBB(*MissedSuccs.begin())
      .addOperand(VInstrInfo::CreatePredicate())
      .addOperand(VInstrInfo::CreateTrace());
  }
  //else if (Table.size() != MBB->succ_size()) {
  //  // Also fix the CFG.
  //  while (!MBB->succ_empty())
  //    MBB->removeSuccessor(MBB->succ_end() - 1);
  //  for (jt_it JI = Table.begin(), JE = Table.end(); JI != JE; ++JI)
  //    MBB->addSuccessor(JI->first);

  //  // Try to correct the CFG.
  //  TII->RemoveBranch(*MBB);
  //  VInstrInfo::insertJumpTable(*MBB, Table, DebugLoc());
  //}

  //Table.clear();

  if (MBB->succ_size() == 0 && MBB->getFirstTerminator() == MBB->end()) {
    ++Unreachables;
    BuildMI(MBB, DebugLoc(), TII->get(VTM::VOpUnreachable))
      .addOperand(VInstrInfo::CreatePredicate())
      .addOperand(VInstrInfo::CreateTrace());
  }
}
}
