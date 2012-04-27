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

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/Support/raw_ostream.h"
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

  FixMachineCode(bool isPreOpt) : MachineFunctionPass(ID), MRI(0), TII(0),
    IsPreOpt(isPreOpt) {}

  //void getAnalysisUsage(AnalysisUsage &AU) const {
  //  MachineFunctionPass::getAnalysisUsage(AU);
  //  // Is this true?
  //  // AU.setPreservesAll();
  //}

  bool runOnMachineFunction(MachineFunction &MF);

  MachineBasicBlock::instr_iterator
  getPHIMoveInsertPos(MachineBasicBlock *SrcBB, MachineBasicBlock *DstBB,
                      MachineOperand &Pred);
  void handlePHI(MachineInstr *PN, MachineBasicBlock *CurBB);
  bool simplifyReduction(MachineInstr *MI);
  bool simplifyBitSlice(MachineInstr *MI);
  bool handleImplicitDefs(MachineInstr *MI);
  void FoldInstructions(std::vector<MachineInstr*> &InstrToFold);

  void FoldMove(MachineInstr *MI, std::vector<MachineInstr*> &InstrToFold);
  void FoldAdd(MachineInstr *MI, std::vector<MachineInstr*> &InstrToFold);

  bool canbeFold(MachineInstr *MI) const;

  const char *getPassName() const {
    return "Fix machine code for Verilog backend";
  }
};
}

char FixMachineCode::ID = 0;

bool FixMachineCode::runOnMachineFunction(MachineFunction &MF) {
  MRI = &MF.getRegInfo();
  TII = MF.getTarget().getInstrInfo();
  std::vector<MachineInstr*> InstrToFold, PNs;

   // Find out all VOpMove_mi.
  for (MachineFunction::iterator BI = MF.begin(), BE = MF.end();BI != BE;++BI) {
    MachineBasicBlock *MBB = BI;

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

      if (simplifyReduction(Inst)) continue;
      if (simplifyBitSlice(Inst)) continue;
      if (handleImplicitDefs(Inst)) continue;

      if (Inst->isCopy()) {
        MachineOperand &SrcMO = Inst->getOperand(1);
        MachineInstr *DefMI = MRI->getVRegDef(SrcMO.getReg());
        MachineOperand &SrcDefMO = DefMI->getOperand(0);
        unsigned DstWidth = VInstrInfo::getBitWidth(SrcDefMO);
        VInstrInfo::setBitWidth(SrcMO, DstWidth);
        VInstrInfo::setBitWidth(Inst->getOperand(0), DstWidth);
        VInstrInfo::ChangeCopyToMove(Inst);
      }

      // Try to eliminate unnecessary moves.
      if (canbeFold(Inst)) {
        InstrToFold.push_back(Inst);
        continue;
      }

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

  FoldInstructions(InstrToFold);

  return true;
}

bool FixMachineCode::simplifyBitSlice(MachineInstr *MI) {
  if (MI->getOpcode() != VTM::VOpBitSlice || !MI->getOperand(1).isReg())
    return false;

  // Only can simplify if lower bound is 0, which means the result simply
  // use the lower part of the src register. We can simply replace the use
  // of dst register by src register, with the original bitwidth information
  // of the dst register, i.e. using the lower part of the src register.
  if (MI->getOperand(3).getImm() != 0) {
    unsigned UB = MI->getOperand(2).getImm();
    // We can narrow the bitwidth of the operand to the upper bound.
    VInstrInfo::setBitWidth(MI->getOperand(1), UB);
    return false;
  }

  unsigned SrcReg = MI->getOperand(1).getReg();
  unsigned DstReg = MI->getOperand(0).getReg();

  MI->eraseFromParent();
  MRI->replaceRegWith(DstReg, SrcReg);
  MRI->clearKillFlags(SrcReg);
  ++BitSliceSimplified;
  return true;
}

bool FixMachineCode::simplifyReduction(MachineInstr *MI) {
  unsigned Opcode = MI->getOpcode();
  if (Opcode != VTM::VOpROr && Opcode != VTM::VOpRAnd && Opcode != VTM::VOpRXor)
    return false;

  MachineOperand &Src = MI->getOperand(1);
  if (VInstrInfo::getBitWidth(Src) != 1) return false;

  // If the bitwidth of src operand is 1, the reduction is not necessary.
  unsigned SrcReg = Src.getReg();
  unsigned DstReg = MI->getOperand(0).getReg();

  MI->eraseFromParent();
  MRI->replaceRegWith(DstReg, SrcReg);
  MRI->clearKillFlags(SrcReg);

  ++ReductionSimplified;
  return true;
}

MachineBasicBlock::instr_iterator
FixMachineCode::getPHIMoveInsertPos(MachineBasicBlock *SrcBB,
                                    MachineBasicBlock *DstBB,
                                    MachineOperand &Pred) {
  VInstrInfo::JT SrcJT;
  bool success = !VInstrInfo::extractJumpTable(*SrcBB, SrcJT, false);
  assert(success && "Broken machine code?");
  // TODO: Handle critical edges.

  // Insert the PHI copy.
  VInstrInfo::JT::iterator at = SrcJT.find(DstBB);
  assert(at != SrcJT.end() && "Broken CFG?");
  Pred = at->second;
  return SrcBB->getFirstInstrTerminator();
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
    it IP = getPHIMoveInsertPos(SrcBB, CurBB, Pred);
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

bool FixMachineCode::canbeFold(MachineInstr *MI) const {
  if (MI->getOpcode() == VTM::VOpMove) return true;

  if (MI->getOpcode() == VTM::VOpAdd || MI->getOpcode() == VTM::VOpAdd_c) {
    // Fold the add only if carry input is 0.
    if (!MI->getOperand(3).isImm() || MI->getOperand(3).getImm() != 0)
      return false;

    if (MI->getOperand(1).isImm() && MI->getOperand(1).getImm() == 0)
      return true;

    if (MI->getOperand(2).isImm() && MI->getOperand(2).getImm() == 0)
      return true;
  }

  return false;
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
  return false;
}

void FixMachineCode::FoldMove(MachineInstr *MI,
                                   std::vector<MachineInstr*> &InstrToFold) {
  unsigned DstReg = MI->getOperand(0).getReg();

  std::set<MachineInstr*> FoldList;
  for (MachineRegisterInfo::use_iterator I = MRI->use_begin(DstReg),
       E = MRI->use_end(); I != E; /*++I*/) {
    MachineInstr &UserMI = *I;
    ++I;

    // Only replace if user is not a PHINode.
    if (UserMI.getOpcode() == VTM::PHI) continue;

    // There maybe an instruction read the same register twice.
    FoldList.insert(&UserMI);
  }

  // Replace the register operand by a immediate operand.
  typedef std::set<MachineInstr*>::iterator it;
  for (it I = FoldList.begin(), E = FoldList.end(); I != E; ++I) {
    MachineInstr *UserMI = *I;
    if (TII->FoldImmediate(UserMI, MI, DstReg, MRI) && canbeFold(UserMI))
      InstrToFold.push_back(UserMI);
  }

  // Eliminate the instruction if it dead.
  if (MRI->use_empty(DstReg)) MI->eraseFromParent();
}

void FixMachineCode::FoldAdd(MachineInstr *MI,
                             std::vector<MachineInstr*> &InstrToFold) {
  unsigned NoneZeroIdx = 1;
  if (MI->getOperand(1).isImm() && MI->getOperand(1).getImm() == 0)
    NoneZeroIdx = 2;

  // Change the add to bitcat(0, NoneZeroOperand) to construct the result.
  MI->setDesc(TII->get(VTM::VOpBitCat));
  MI->RemoveOperand(3);

  if (NoneZeroIdx != 2) {
    unsigned NoneZeroReg = MI->getOperand(NoneZeroIdx).getReg();
    MI->getOperand(2).ChangeToRegister(NoneZeroReg, false);
  }

  // Build the carry bit of the original
  MachineOperand &DummyCarry = MI->getOperand(1);
  DummyCarry.ChangeToImmediate(0);
  VInstrInfo::setBitWidth(DummyCarry, 1);
}

void FixMachineCode::FoldInstructions(std::vector<MachineInstr*> &InstrToFold) {
  while (!InstrToFold.empty()) {
    MachineInstr *MI = InstrToFold.back();
    InstrToFold.pop_back();

    switch (MI->getOpcode()) {
    case VTM::VOpMove:
      FoldMove(MI, InstrToFold);
      break;
    case VTM::VOpAdd:
    case VTM::VOpAdd_c:
      FoldAdd(MI, InstrToFold);
      break;
    default:
      llvm_unreachable("Trying to fold unexpected instruction!");
    }
  }
}

Pass *llvm::createFixMachineCodePass(bool IsPreOpt) {
  return new FixMachineCode(IsPreOpt);
}
