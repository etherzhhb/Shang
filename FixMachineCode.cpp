//===---------- FixMachineCode.cpp - Fix The Machine Code  ------*- C++ -*-===//
//
//                            The Verilog Backend
//
// Copyright: 2010 by Hongbin Zheng. all rights reserved.
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
// This file implements a pass that fix the machine code to simpler the code in
// later pass.
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
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/CommandLine.h"
#define DEBUG_TYPE "vtm-fix-machine-code"
#include "llvm/Support/Debug.h"
#include <set>

using namespace llvm;

STATISTIC(UnconditionalBranches,
          "Number of unconditionnal branches inserted for fall through edges");
STATISTIC(Unreachables,
     "Number of Unreachable inserted for machine basic block without sucessor");
namespace {
struct FixMachineCode : public MachineFunctionPass {
  static char ID;

  FixMachineCode() : MachineFunctionPass(ID) {}

  void getAnalysisUsage(AnalysisUsage &AU) const {
    MachineFunctionPass::getAnalysisUsage(AU);
    // Is this true?
    // AU.setPreservesAll();
  }

  bool runOnMachineFunction(MachineFunction &MF);

  void handleWireOps(MachineInstr *Inst, MachineRegisterInfo &MRI,
                     const TargetInstrInfo *TII);

  void eliminateMVImm(std::vector<MachineInstr*> &Worklist,
                      MachineRegisterInfo &MRI);

  const char *getPassName() const {
    return "Fix machine code for Verilog backend";
  }
};
}

char FixMachineCode::ID = 0;

void FixMachineCode::handleWireOps(MachineInstr *Inst, MachineRegisterInfo &MRI,
                                   const TargetInstrInfo *TII) {
  if (!VInstrInfo::isWireOp(Inst->getOpcode())) return;

  bool needCopy = false;
  unsigned OriginalReg = Inst->getOperand(0).getReg();
  unsigned NewWireReg = MRI.createVirtualRegister(VTM::WireRegisterClass);

  typedef MachineRegisterInfo::reg_iterator reg_it;
  for (reg_it I = MRI.reg_begin(OriginalReg),E = MRI.reg_end();I != E;/*++I*/) {
    MachineOperand &O = I.getOperand();
    MachineInstr &MI = *I;
    ++I;
    // PHIs need all operands have the same register class.
    if (MI.isPHI()) {
      needCopy = true;
      continue;
    }

    O.setReg(NewWireReg);
  }

  if (needCopy) {
    MachineBasicBlock &MBB = *Inst->getParent();
    DebugLoc dl = Inst->getDebugLoc();
    unsigned BitWidth = cast<ucOperand>(Inst->getOperand(0)).getBitWidth();
    MachineBasicBlock::iterator IP = Inst;
    ++IP;
    BuildMI(MBB, IP, dl, TII->get(VTM::VOpMove_rw))
      .addOperand(ucOperand::CreateReg(OriginalReg, BitWidth, true))
      .addOperand(ucOperand::CreateReg(NewWireReg, BitWidth))
      .addOperand(*VInstrInfo::getPredOperand(Inst));
  }
}

bool FixMachineCode::runOnMachineFunction(MachineFunction &MF) {
  MachineRegisterInfo &MRI = MF.getRegInfo();
  const TargetInstrInfo *TII = MF.getTarget().getInstrInfo();

  std::vector<MachineInstr*> Imms;
  SmallPtrSet<MachineBasicBlock*, 2> MissedSuccs;
  MachineInstr *FirstTerminator = 0;
   // Find out all VOpMove_mi.
  for (MachineFunction::iterator BI = MF.begin(), BE = MF.end();BI != BE;++BI) {
    MachineBasicBlock *MBB = BI;
    MissedSuccs.insert(MBB->succ_begin(), MBB->succ_end());

    for (MachineBasicBlock::iterator II = MBB->begin(), IE = MBB->end();
         II != IE; ++II) {
      MachineInstr *Inst = II;
      // Try to eliminate unnecessary moves.
      if (Inst->getOpcode() == VTM::VOpMove_ri)
        Imms.push_back(Inst);

      // BitSlice, BitCat and BitRepeat are wires.
      handleWireOps(Inst, MRI, TII);

      // Remove the explicit successors from the missed successors set.
      if (VInstrInfo::isBrCndLike(Inst->getOpcode())) {
        MachineOperand &Pred = Inst->getOperand(1);
        // Use reg0 for always true.
        if (Pred.isImm() && Pred.getImm()) Pred.ChangeToRegister(0, false);
        // Change the unconditional branch after conditional branch to
        // conditional branch.
        if (FirstTerminator && !TII->isPredicated(Inst)) {
          MachineOperand *TrueCnd = VInstrInfo::getPredOperand(FirstTerminator);
          MachineOperand *FalseCnd = VInstrInfo::getPredOperand(Inst);
          FalseCnd->setReg(TrueCnd->getReg());
          FalseCnd->setTargetFlags(TrueCnd->getTargetFlags());
          VInstrInfo::ReversePredicateCondition(*FalseCnd);
        }

        FirstTerminator = Inst;

        MissedSuccs.erase(Inst->getOperand(0).getMBB());
      }
    }

    // Make sure each basic block have a terminator.
    if (!MissedSuccs.empty()) {
      assert(MissedSuccs.size() == 1 && "Fall through to multiple blocks?");
      ++UnconditionalBranches;
      MachineOperand Predicate = ucOperand::CreatePredicate();
      if (FirstTerminator) {
        MachineOperand *TrueCnd = VInstrInfo::getPredOperand(FirstTerminator);
        assert(TrueCnd->getReg() != 0 && "Two unconditional branch?");
        Predicate = *TrueCnd;
        VInstrInfo::ReversePredicateCondition(Predicate);
      }
      BuildMI(MBB, DebugLoc(), TII->get(VTM::VOpToStateb))
        .addMBB(*MissedSuccs.begin()).addOperand(Predicate);
    }

    if (MBB->succ_size() == 0 && MBB->getFirstTerminator() == MBB->end()) {
      ++Unreachables;
      BuildMI(MBB, DebugLoc(), TII->get(VTM::VOpUnreachable))
        .addOperand(ucOperand::CreatePredicate());
    }

    MissedSuccs.clear();
    FirstTerminator = 0;
  }

  // Try to replace the register operand with the constant for users of VOpMvImm.
  if (!Imms.empty()) eliminateMVImm(Imms, MRI);

  // Forward the use operand of wireops so we can compute a correct live
  // interval for the use operands of wireops. for example if we have:
  // a = bitslice b, ...
  // ...
  // ... = a ...
  // and we will add the operand of wireops to the instructions that using its
  // results as implicit use:
  // a = bitslice b, ...
  // ...
  // ... = a ..., imp use b
  for (MachineFunction::iterator BI = MF.begin(), BE = MF.end();BI != BE;++BI)
    for (MachineBasicBlock::iterator II = BI->begin(), IE = BI->end();
         II != IE; ++II) {
      MachineInstr *Inst = II;
      if (VInstrInfo::isWireOp(Inst->getOpcode())) continue;

      for (unsigned i = 0; i < Inst->getNumOperands(); ++i) {
        MachineOperand &MO = Inst->getOperand(i);
        if (!MO.isReg() || !MO.isUse() || !MO.getReg()) continue;

        MachineInstr *DefInst = MRI.getVRegDef(MO.getReg());
        assert(DefInst && "Define instruction not exist!");
        if (!VInstrInfo::isWireOp(DefInst->getOpcode())) continue;

        // Add the use operand of wireops
        if (DefInst->getOperand(1).isReg())
          MachineInstrBuilder(Inst).addReg(DefInst->getOperand(1).getReg(),
                                           RegState::Implicit);
        if (DefInst->getOpcode() == VTM::VOpBitCat
            && DefInst->getOperand(2).isReg())
          MachineInstrBuilder(Inst).addReg(DefInst->getOperand(2).getReg(),
                                           RegState::Implicit);
      }
    }

  return true;
}

void FixMachineCode::eliminateMVImm(std::vector<MachineInstr*> &Worklist,
                                    MachineRegisterInfo &MRI) {
  SmallVector<MachineOperand, 8> Ops;
  // Find all replaceable operand.
  std::vector<std::pair<MachineInstr*, unsigned> > ImmUsers;

  while (!Worklist.empty()) {
    MachineInstr *MI = Worklist.back();
    Worklist.pop_back();

    unsigned DstReg = MI->getOperand(0).getReg();

    for (MachineRegisterInfo::use_iterator I = MRI.use_begin(DstReg),
          E = MRI.use_end(); I != E; ++I) {
      // Only replace if user is not a PHINode.
      if (I->getOpcode() == VTM::PHI) continue;

      ImmUsers.push_back(std::make_pair(&*I, I.getOperandNo()));
    }

    // Perform the replacement.
    MachineOperand Imm = MI->getOperand(1);
    Imm.clearParent();

    while (!ImmUsers.empty()) {
      MachineInstr *User = ImmUsers.back().first;
      unsigned Idx = ImmUsers.back().second;
      ImmUsers.pop_back();

      unsigned NumOps = User->getNumOperands();
      while (NumOps > 0) {
        --NumOps;
        if (NumOps == Idx)
          Ops.push_back(Imm);
        else
          Ops.push_back(User->getOperand(NumOps));

        User->RemoveOperand(NumOps);
      }

      while (!Ops.empty())
        User->addOperand(Ops.pop_back_val());
    }

    // Eliminate the instruction if it dead.
    if (MRI.use_empty(DstReg)) MI->eraseFromParent();
  }
}


Pass *llvm::createFixMachineCodePass() {
  return new FixMachineCode();
}
