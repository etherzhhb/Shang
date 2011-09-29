//===- FixMachineCode.cpp - Fix The Terminators in Machine Code -*- C++ -*-===//
//
//                            The Verilog Backend
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements a pass that add branch instruction that branch to fall
// through block explicitly.
//
//===----------------------------------------------------------------------===//

#include "vtm/Passes.h"
#include "vtm/VTM.h"
#include "vtm/VInstrInfo.h"
#include "vtm/MicroState.h"

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

namespace {
struct FixTerminators : public MachineFunctionPass {
  static char ID;

  FixTerminators() : MachineFunctionPass(ID) {}

  bool runOnMachineFunction(MachineFunction &MF);

  const char *getPassName() const {
    return "Fix Terminators for Verilog backend";
  }
};
}

char FixTerminators::ID = 0;

bool FixTerminators::runOnMachineFunction(MachineFunction &MF) {
  const TargetInstrInfo *TII = MF.getTarget().getInstrInfo();
  SmallPtrSet<MachineBasicBlock*, 2> MissedSuccs;

  for (MachineFunction::iterator BI = MF.begin(), BE = MF.end();BI != BE;++BI) {
    MachineBasicBlock *MBB = BI;
    MissedSuccs.insert(MBB->succ_begin(), MBB->succ_end());
    MachineInstr *FirstTerminator = 0;

    for (MachineBasicBlock::iterator II = MBB->getFirstTerminator(),
         IE = MBB->end(); II != IE; ++II) {
      MachineInstr *Inst = II;
      if (!VInstrInfo::isBrCndLike(Inst->getOpcode())) continue;

      MachineOperand &Cnd = Inst->getOperand(1);
      // Use reg0 for always true.
      if (Cnd.isImm() && Cnd.getImm()) Cnd.ChangeToRegister(0, false);
      // Change the unconditional branch after conditional branch to
      // conditional branch.
      if (FirstTerminator && VInstrInfo::isUnConditionalBranch(Inst)){
        MachineOperand &TrueCnd = FirstTerminator->getOperand(0);
        MachineOperand &FalseCnd = Inst->getOperand(0);
        FalseCnd.setReg(TrueCnd.getReg());
        FalseCnd.setTargetFlags(TrueCnd.getTargetFlags());
        VInstrInfo::ReversePredicateCondition(FalseCnd);
      }

      FirstTerminator = Inst;
      MissedSuccs.erase(Inst->getOperand(1).getMBB());
    }

    // Make sure each basic block have a terminator.
    if (!MissedSuccs.empty()) {
      assert(MissedSuccs.size() == 1 && "Fall through to multiple blocks?");
      ++UnconditionalBranches;
      MachineOperand Cnd = ucOperand::CreatePredicate();
      if (FirstTerminator) {
        MachineOperand &TrueCnd = FirstTerminator->getOperand(0);
        assert(TrueCnd.getReg() != 0 && "Two unconditional branch?");
        Cnd = TrueCnd;
        VInstrInfo::ReversePredicateCondition(Cnd);
      }
      BuildMI(MBB, DebugLoc(), TII->get(VTM::VOpToStateb))
        .addOperand(Cnd).addMBB(*MissedSuccs.begin())
        .addOperand(ucOperand::CreatePredicate())
        .addOperand(ucOperand::CreateTrace(MBB));
    }

    if (MBB->succ_size() == 0 && MBB->getFirstTerminator() == MBB->end()) {
      ++Unreachables;
      BuildMI(MBB, DebugLoc(), TII->get(VTM::VOpRet))
        .addOperand(ucOperand::CreatePredicate())
        .addOperand(ucOperand::CreateTrace(MBB));
    }

    MissedSuccs.clear();
  }

  return true;
}

Pass *llvm::createFixTerminatorsPass() {
  return new FixTerminators();
}
