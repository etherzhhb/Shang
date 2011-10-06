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
#include "llvm/Support/CommandLine.h"
#define DEBUG_TYPE "vtm-fix-machine-code"
#include "llvm/Support/Debug.h"
#include <set>

using namespace llvm;

namespace {
struct FixMachineCode : public MachineFunctionPass {
  static char ID;
  // The common implicit defined register for current module.
  unsigned ImplicitDefReg;

  FixMachineCode() : MachineFunctionPass(ID) {}


  void getAnalysisUsage(AnalysisUsage &AU) const {
    MachineFunctionPass::getAnalysisUsage(AU);
    // Is this true?
    // AU.setPreservesAll();
  }

  bool runOnMachineFunction(MachineFunction &MF);

  void forwardWireOpOperands(MachineFunction &MF, MachineRegisterInfo &MRI);

  bool handleImplicitDefs(MachineInstr *MI, MachineRegisterInfo &MRI,
                          const TargetInstrInfo *TII);

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

  unsigned OldReg = Inst->getOperand(0).getReg();
  MRI.setRegClass(OldReg, VTM::WireRegisterClass);
}

bool FixMachineCode::runOnMachineFunction(MachineFunction &MF) {
  MachineRegisterInfo &MRI = MF.getRegInfo();
  const TargetInstrInfo *TII = MF.getTarget().getInstrInfo();
  ImplicitDefReg = 0;

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
      if (Inst->getOpcode() == VTM::VOpMove_ri) {
        Imms.push_back(Inst);
        continue;
      }

      // BitSlice, BitCat and BitRepeat are wires.
      handleWireOps(Inst, MRI, TII);
    }
  }

  eliminateMVImm(Imms, MRI);
  forwardWireOpOperands(MF, MRI);

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
        MO.ChangeToImmediate(Imm.getImm());
        MO.setTargetFlags(Imm.getTargetFlags());
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
    for (MachineBasicBlock::iterator II = BI->getFirstNonPHI(), IE = BI->end();
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

        // The implicit use of wire operands break the instructions that read
        // at emit.
        if (VIDesc(*DefInst).isReadAtEmit()) continue;

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
