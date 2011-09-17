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

  void forwardWireOpOperands(MachineFunction &MF, MachineRegisterInfo &MRI);


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
  if (!VInstrInfo::isWireOp(Inst->getDesc())) return;

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
   // Find out all VOpMove_mi.
  for (MachineFunction::iterator BI = MF.begin(), BE = MF.end();BI != BE;++BI) {
    MachineBasicBlock *MBB = BI;

    for (MachineBasicBlock::iterator II = MBB->begin(), IE = MBB->end();
         II != IE; ++II) {
      MachineInstr *Inst = II;
      // Try to eliminate unnecessary moves.
      if (Inst->getOpcode() == VTM::VOpMove_ri)
        Imms.push_back(Inst);

      // BitSlice, BitCat and BitRepeat are wires.
      handleWireOps(Inst, MRI, TII);
    }
  }

  eliminateMVImm(Imms, MRI);
  forwardWireOpOperands(MF, MRI);

  return true;
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

void FixMachineCode::forwardWireOpOperands(MachineFunction &MF,
                                           MachineRegisterInfo &MRI) {
  SmallVector<unsigned, 8> WireOperands;
  SmallPtrSet<MachineInstr*, 8> VisitedInsts;
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
      if (VInstrInfo::isWireOp(Inst->getDesc())) continue;

      for (unsigned i = 0, e = Inst->getNumOperands(); i != e; ++i) {
        MachineOperand &MO = Inst->getOperand(i);
        if (!MO.isReg() || !MO.isUse() || !MO.getReg()) continue;

        unsigned Reg = MO.getReg();
        if (VRegisterInfo::IsWire(Reg, &MRI))
          WireOperands.push_back(Reg);
      }

      while (!WireOperands.empty()) {
        unsigned Reg = WireOperands.pop_back_val();

        MachineInstr *DefInst = MRI.getVRegDef(Reg);
        assert(DefInst && "Define instruction not exist!");
        if (!VisitedInsts.insert(DefInst)) continue;

        for (unsigned i = 0, e = DefInst->getNumOperands(); i != e; ++i) {
          MachineOperand &MO = DefInst->getOperand(i);

          if (!MO.isReg() || !MO.isUse() || !MO.getReg()) continue;
          if (DefInst->getDesc().OpInfo[i].isPredicate()) continue;
          assert(!DefInst->isPHI() && "PHI should not define PHI!");

          unsigned Reg = MO.getReg();
          if (VRegisterInfo::IsWire(Reg, &MRI)) {
            WireOperands.push_back(Reg);
            continue;
          }
          // Forward the operand of wireop to its user by add them as implicit
          // use register operand.
          MachineInstrBuilder(Inst).addReg(Reg, RegState::Implicit);
        }
      }

      VisitedInsts.clear();
    }
}

Pass *llvm::createFixMachineCodePass() {
  return new FixMachineCode();
}
