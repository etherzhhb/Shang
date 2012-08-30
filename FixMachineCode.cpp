//===---------- FixMachineCode.cpp - Fix The Machine Code  ------*- C++ -*-===//
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
// This file implements a pass that fix the machine code to simpler the code in
// later pass.
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
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/ADT/PostOrderIterator.h"
#include "llvm/ADT/Statistic.h"
#define DEBUG_TYPE "vtm-fix-machine-code"
#include "llvm/Support/Debug.h"
#include <set>

using namespace llvm;

STATISTIC(ReductionSimplified, "Number of Reductions are simplified");
STATISTIC(BitSliceSimplified, "Number of BitSlices are simplified");
namespace {
struct FixMachineCode : public MachineFunctionPass {
  static char ID;
  MachineRegisterInfo *MRI;
  const TargetInstrInfo *TII;
  bool IsPreOpt;

  explicit FixMachineCode(bool isPreOpt) : MachineFunctionPass(ID), MRI(0),
                                           TII(0), IsPreOpt(isPreOpt) {
    initializeMachineBasicBlockTopOrderPass(*PassRegistry::getPassRegistry());
  }

  //void getAnalysisUsage(AnalysisUsage &AU) const {
  //  MachineFunctionPass::getAnalysisUsage(AU);
  //  // Is this true?
  //  // AU.setPreservesAll();
  //}

  bool runOnMachineFunction(MachineFunction &MF);

  void getAnalysisUsage(AnalysisUsage &AU) const {
    AU.addRequiredID(MachineBasicBlockTopOrderID);
    AU.addPreservedID(MachineBasicBlockTopOrderID);
    MachineFunctionPass::getAnalysisUsage(AU);
  }

  void handlePHI(MachineInstr *PN, MachineBasicBlock *CurBB);
  bool handleImplicitDefs(MachineInstr *MI);

  bool replaceCopyByMove(MachineInstr * Inst);

  const char *getPassName() const {
    return "Fix machine code for Verilog backend";
  }
};
}

char FixMachineCode::ID = 0;

bool FixMachineCode::runOnMachineFunction(MachineFunction &MF) {
  MRI = &MF.getRegInfo();
  TII = MF.getTarget().getInstrInfo();
  std::vector<MachineInstr*> PNs;

  // Find out all VOpMove_mi.
  for (MachineFunction::iterator BI = MF.begin(), BE = MF.end();BI != BE;++BI) {
    MachineBasicBlock *MBB = BI;
    // Fix the terminators first.
    fixTerminators(MBB);

    for (MachineBasicBlock::iterator II = MBB->begin(), IE = MBB->end();
         II != IE; /*++II*/) {
      MachineInstr *Inst = II;

      ++II; // We may delete the current instruction.

      if (Inst->isPHI()) {
        MachineOperand &DstMO = Inst->getOperand(0);
        unsigned DstWidth = VInstrInfo::getBitWidthOrZero(DstMO);
        SmallVector<MachineOperand*, 4> MOs;
        for (unsigned i = 1, e = Inst->getNumOperands(); i < e; i += 2) {
          MachineOperand *SrcMO = &Inst->getOperand(i);
          unsigned SrcBitWidth = VInstrInfo::getBitWidthOrZero(*SrcMO);
          if (SrcBitWidth == 0) {
            MOs.push_back(SrcMO);
            MachineInstr *DefMI = MRI->getVRegDef(SrcMO->getReg());
            MachineOperand &SrcDefMO = DefMI->getOperand(0);
            SrcBitWidth = VInstrInfo::getBitWidthOrZero(SrcDefMO);
          }

          DstWidth = std::max(SrcBitWidth, DstWidth);
        }

        VInstrInfo::setBitWidth(DstMO, DstWidth);
        while (!MOs.empty())
          VInstrInfo::setBitWidth(*MOs.pop_back_val(), DstWidth);

        if (!IsPreOpt) PNs.push_back(Inst);
        continue;
      }

      if (handleImplicitDefs(Inst)) continue;

      // Replace Copy by Move, which can be predicated.
      if (Inst->isCopy() && replaceCopyByMove(Inst))
        continue;

      // No need to predicate the VOpMoveArg.
      if (Inst->getOpcode() == VTM::VOpMoveArg)
        VInstrInfo::getPredOperand(Inst)->ChangeToRegister(0, false);
    }

    //MachineInstr *FirstNotPHI = 0;

    while (!PNs.empty()) {
      MachineInstr *PN = PNs.back();
      PNs.pop_back();

      handlePHI(PN, MBB);
    }
  }

  return true;
}

void FixMachineCode::handlePHI(MachineInstr *PN, MachineBasicBlock *CurBB) {  
  unsigned BitWidth = VInstrInfo::getBitWidth(PN->getOperand(0));
  //bool isAllImpDef = true;

  for (unsigned i = 1, e = PN->getNumOperands(); i != e; i += 2) {
    MachineOperand &SrcMO = PN->getOperand(i);
    MachineInstr *DefMI = MRI->getVRegDef(SrcMO.getReg());
    assert(DefMI && "Not in SSA form?");
    if (DefMI->isImplicitDef())
      continue;

    MachineBasicBlock *SrcBB = PN->getOperand(i + 1).getMBB();

    unsigned NewSrcReg =
      MRI->createVirtualRegister(MRI->getRegClass(SrcMO.getReg()));

    MachineOperand Pred = VInstrInfo::CreatePredicate();
    typedef MachineBasicBlock::instr_iterator it;
    it IP = VInstrInfo::getEdgeCndAndInsertPos(SrcBB, CurBB, Pred);
    BuildMI(*SrcBB, IP, DebugLoc(), TII->get(VTM::VOpMvPhi))
      .addOperand(VInstrInfo::CreateReg(NewSrcReg, BitWidth, true))
      .addOperand(SrcMO).addMBB(CurBB)
      // The phi copy is only active when SrcBB jumping to CurBB.
      .addOperand(Pred)
      .addImm(0);

    SrcMO.ChangeToRegister(NewSrcReg, false);
    //isAllImpDef = false;
  }

  // TODO: Incoming copy?
  // TODO: if all the incoming value of the PHI is ImpDef, erase the PN.
}

bool FixMachineCode::handleImplicitDefs(MachineInstr *MI) {
  if (!MI->isImplicitDef()) return false;

  unsigned Reg = MI->getOperand(0).getReg();
  bool use_empty = true;

  typedef MachineRegisterInfo::use_iterator use_it;
  for (use_it I = MRI->use_begin(Reg), E = MRI->use_end(); I != E; /*++I*/) {
    MachineOperand *MO = &I.getOperand();
    MachineInstr &UserMI = *I;
    ++I;
    // Implicit value always have 64 bit.
    VInstrInfo::setBitWidth(*MO, 64);

    if (UserMI.isPHI()) {
      use_empty = false;
      continue;
    }

    // Change to register 0.
    MO->ChangeToRegister(0, false);
  }

  if (use_empty) MI->removeFromParent();
  return use_empty;
}

bool FixMachineCode::replaceCopyByMove(MachineInstr *Inst) {
  MachineOperand &SrcMO = Inst->getOperand(1);
  if (unsigned Reg = SrcMO.getReg()) {
    MachineInstr *DefMI = MRI->getVRegDef(Reg);
    MachineOperand &SrcDefMO = DefMI->getOperand(0);
    unsigned DstWidth = VInstrInfo::getBitWidth(SrcDefMO);
    VInstrInfo::setBitWidth(SrcMO, DstWidth);
    VInstrInfo::setBitWidth(Inst->getOperand(0), DstWidth);
    VInstrInfo::ChangeCopyToMove(Inst);
    return false;
  } 

  Inst->RemoveOperand(1);
  Inst->setDesc(VInstrInfo::getDesc(VTM::IMPLICIT_DEF));
  return handleImplicitDefs(Inst);
}

Pass *llvm::createFixMachineCodePass(bool IsPreOpt) {
  return new FixMachineCode(IsPreOpt);
}
