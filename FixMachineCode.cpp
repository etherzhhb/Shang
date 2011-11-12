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
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/Support/raw_ostream.h"
#define DEBUG_TYPE "vtm-fix-machine-code"
#include "llvm/Support/Debug.h"
#include <set>

using namespace llvm;

namespace {
struct FixMachineCode : public MachineFunctionPass {
  static char ID;

  FixMachineCode() : MachineFunctionPass(ID) {}

  //void getAnalysisUsage(AnalysisUsage &AU) const {
  //  MachineFunctionPass::getAnalysisUsage(AU);
  //  // Is this true?
  //  // AU.setPreservesAll();
  //}

  bool runOnMachineFunction(MachineFunction &MF);

  bool handleImplicitDefs(MachineInstr *MI, MachineRegisterInfo &MRI,
                          const TargetInstrInfo *TII);
  bool mergeSel(MachineInstr *MI, MachineRegisterInfo &MRI,
                const TargetInstrInfo *TII);
  void mergeSelToCase(MachineInstr *CaseMI, MachineInstr *SelMI,
                      MachineOperand Cnd,
                      MachineRegisterInfo &MRI, const TargetInstrInfo *TII);

  void eliminateMVImm(std::vector<MachineInstr*> &Worklist,
                      MachineRegisterInfo &MRI);

  const char *getPassName() const {
    return "Fix machine code for Verilog backend";
  }
};
}

char FixMachineCode::ID = 0;

bool FixMachineCode::runOnMachineFunction(MachineFunction &MF) {
  MachineRegisterInfo &MRI = MF.getRegInfo();
  const TargetInstrInfo *TII = MF.getTarget().getInstrInfo();

  std::vector<MachineInstr*> Imms;
   // Find out all VOpMove_mi.
  for (MachineFunction::iterator BI = MF.begin(), BE = MF.end();BI != BE;++BI) {
    MachineBasicBlock *MBB = BI;

    for (MachineBasicBlock::iterator II = MBB->begin(), IE = MBB->end();
         II != IE; /*++II*/) {
      MachineInstr *Inst = II;
      ++II; // We may delete the current instruction.

      if (handleImplicitDefs(Inst, MRI, TII)) continue;

      // Try to eliminate unnecessary moves.
      if (Inst->getOpcode() == VTM::VOpMove_ri
          || (Inst->getOpcode() == VTM::COPY && Inst->getOperand(1).isImm())) {
        Imms.push_back(Inst);
        continue;
      }

      //if (Inst->getOpcode() == VTM::VOpICmp) {
      //  MachineOperand &DefMO = Inst->getOperand(0);
      //  unsigned DefReg = DefMO.getReg();
      //  unsigned NewReg = MRI.createVirtualRegister(VTM::DRRegisterClass);
      //  DefMO.setReg(NewReg);
      //  DefMO.setTargetFlags(8);

      //  unsigned Idx = VFUs::getICmpPort(Inst->getOperand(3).getImm());
      //  MachineOperand UB = MachineOperand::CreateImm(Idx + 1);
      //  UB.setTargetFlags(8);
      //  MachineOperand LB = MachineOperand::CreateImm(Idx);
      //  LB.setTargetFlags(8);

      //  BuildMI(*MBB, II, DebugLoc(), TII->get(VTM::VOpBitSlice))
      //    .addOperand(ucOperand::CreateReg(DefReg, 1, true))
      //    .addOperand(ucOperand::CreateReg(NewReg, 1, false))
      //    .addOperand(UB).addOperand(LB)
      //    .addOperand(ucOperand::CreatePredicate())
      //    .addOperand(ucOperand::CreateTrace(MBB));
      //  continue;
      //}

      // Try to merge the Select to improve parallelism.
      mergeSel(Inst, MRI, TII);
    }
  }

  eliminateMVImm(Imms, MRI);

  // Set the register class of the result of the wire operations to the right
  // value.
  for (MachineFunction::iterator BI = MF.begin(), BE = MF.end();BI != BE;++BI)
    for (MachineBasicBlock::iterator II = BI->getFirstNonPHI(), IE = BI->end();
         II != IE; ++II) {
      if (VInstrInfo::isControl(II->getOpcode())) continue;

      unsigned Reg = II->getOperand(0).getReg();
      MRI.setRegClass(Reg, VTM::WireRegisterClass);
    }

  return true;
}

bool FixMachineCode::handleImplicitDefs(MachineInstr *MI,
                                        MachineRegisterInfo &MRI,
                                        const TargetInstrInfo *TII) {
  if (!MI->isImplicitDef()) return false;

  unsigned Reg = MI->getOperand(0).getReg();
  bool use_empty = true;

  typedef MachineRegisterInfo::use_iterator use_it;
  for (use_it I = MRI.use_begin(Reg), E = MRI.use_end(); I != E; /*++I*/) {
    ucOperand *MO = cast<ucOperand>(&I.getOperand());
    MachineInstr &UserMI = *I;
    ++I;
    // Implicit value always have 64 bit.
    MO->setBitWidth(64);

    if (UserMI.isPHI()) {
      use_empty = false;
      continue;
    }

    // Just set the implicit defined register to some strange value.
    MO->ChangeToImmediate(TargetRegisterInfo::virtReg2Index(Reg));
  }

  if (use_empty) MI->removeFromParent();
  return false;
}

void FixMachineCode::eliminateMVImm(std::vector<MachineInstr*> &Worklist,
                                    MachineRegisterInfo &MRI) {
  if (Worklist.empty()) return;

  SmallVector<MachineOperand, 8> Ops;
  // Find all replaceable operand.
  std::vector<std::pair<MachineInstr*, unsigned> > ImmUsers;

  while (!Worklist.empty()) {
    MachineInstr *MI = Worklist.back();
    Worklist.pop_back();

    unsigned DstReg = MI->getOperand(0).getReg();
    // Perform the replacement.
    MachineOperand Imm = MI->getOperand(1);

    for (MachineRegisterInfo::use_iterator I = MRI.use_begin(DstReg),
          E = MRI.use_end(); I != E; /*++I*/) {
      MachineInstr &MI = *I;
      unsigned OpNo = I.getOperandNo();
      MachineOperand &MO = I.getOperand();
      ++I;

      // Only replace if user is not a PHINode.
      if (MI.getOpcode() == VTM::PHI) continue;

      if (Imm.isImm()) {
        // Dirty Hack: Do not eliminate the immediate if the user is
        // BitSlice operation.
        if (MI.getOpcode() != VTM::VOpBitSlice) {
          MO.ChangeToImmediate(Imm.getImm());
          MO.setTargetFlags(Imm.getTargetFlags());
        }

        continue;
      }

      ImmUsers.push_back(std::make_pair(&MI, OpNo));
    }

    // Perform the replacement.
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

bool FixMachineCode::mergeSel(MachineInstr *MI, MachineRegisterInfo &MRI,
                              const TargetInstrInfo *TII) {
  if (MI->getOpcode() != VTM::VOpSel) return false;

  MachineOperand TVal = MI->getOperand(2), FVal = MI->getOperand(3);
  MachineInstr *TMI =0, *FMI = 0;
  if (TVal.isReg()) {
    TMI = MRI.getVRegDef(TVal.getReg());
    if (TMI && TMI->getOpcode() != VTM::VOpSel)
      TMI = 0;
  }

  if (FVal.isReg()) {
    FMI = MRI.getVRegDef(FVal.getReg());
    if (FMI && FMI->getOpcode() != VTM::VOpSel)
      FMI = 0;
  }

  // Both operands are not read from VOpSel
  if (!TMI && !FMI) return false;

  MachineInstr *CaseMI =
    BuildMI(*MI->getParent(), MI, MI->getDebugLoc(), TII->get(VTM::VOpCase))
      .addOperand(MI->getOperand(0)). // Result
      addOperand(MI->getOperand(4)). // Predicate
      addOperand(MI->getOperand(5)); // Trace number

  // Merge the select in to the newly build case.
  MachineOperand Cnd = MI->getOperand(1);
  if (TMI)
    mergeSelToCase(CaseMI, TMI, Cnd, MRI, TII);
  else {
    // Re-add the condition into the case statement.
    CaseMI->addOperand(Cnd);
    CaseMI->addOperand(TVal);
  }

  VInstrInfo::ReversePredicateCondition(Cnd);
  if (FMI)
    mergeSelToCase(CaseMI, FMI, Cnd, MRI, TII);
  else {
    // Re-add the condition into the case statement.
    CaseMI->addOperand(Cnd);
    CaseMI->addOperand(FVal);
  }

  MI->eraseFromParent();
  return true;
}

void FixMachineCode::mergeSelToCase(MachineInstr *CaseMI, MachineInstr *SelMI,
                                    MachineOperand Cnd, MachineRegisterInfo &MRI,
                                    const TargetInstrInfo *TII) {
  MachineOperand SelTCnd = SelMI->getOperand(1);
  MachineOperand SelFCnd = SelTCnd;
  VInstrInfo::ReversePredicateCondition(SelFCnd);

  // Merge the condition with predicate of the select operation.
  if (TII->isPredicated(SelMI)) {
    // DIRTYHACK: This pass the testsuite, why?
    //MachineOperand SelPred = *VInstrInfo::getPredOperand(SelMI);

    //SelTCnd = VInstrInfo::MergePred(SelTCnd, SelPred, *CaseMI->getParent(),
    //                                CaseMI, &MRI, TII, VTM::VOpAnd);
    //SelFCnd = VInstrInfo::MergePred(SelFCnd, SelPred, *CaseMI->getParent(),
    //                                CaseMI, &MRI, TII, VTM::VOpAnd);
  }

  // Merge the condition with the condition to select this SelMI.
  SelTCnd = VInstrInfo::MergePred(SelTCnd, Cnd, *CaseMI->getParent(),
                                  CaseMI, &MRI, TII, VTM::VOpAnd);
  SelFCnd = VInstrInfo::MergePred(SelFCnd, Cnd, *CaseMI->getParent(),
                                  CaseMI, &MRI, TII, VTM::VOpAnd);

  // Add the value for True condition.
  CaseMI->addOperand(SelTCnd);
  CaseMI->addOperand(SelMI->getOperand(2));
  // Add the value for False condition.
  CaseMI->addOperand(SelFCnd);
  CaseMI->addOperand(SelMI->getOperand(3));
}

Pass *llvm::createFixMachineCodePass() {
  return new FixMachineCode();
}
