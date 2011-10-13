//===- ForwardWireUsers.cpp - Forward the operands use by wire ops-*- C++ -*-=//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implement the pass that forward the use operand of wireops so we
// can compute a correct live interval for the use operands of wireops. for
// example if we have:
// a = bitslice b, ...
// ...
// ... = a ...
// and we will add the operand of wireops to the instructions that using its
// results as implicit use:
// a = bitslice b, ...
// ...
// ... = a ..., imp use b
//
//===----------------------------------------------------------------------===//
#include "vtm/Passes.h"
#include "vtm/VTM.h"
#include "vtm/VInstrInfo.h"

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/ADT/SmallSet.h"
#define DEBUG_TYPE "vtm-fix-machine-code"
#include "llvm/Support/Debug.h"

using namespace llvm;

namespace {
struct ForwardWireUsers : public MachineFunctionPass {
  static char ID;

  ForwardWireUsers() : MachineFunctionPass(ID) {}

  bool runOnMachineFunction(MachineFunction &MF);
};
}

char ForwardWireUsers::ID = 0;

bool ForwardWireUsers::runOnMachineFunction(MachineFunction &MF) {
  MachineRegisterInfo &MRI = MF.getRegInfo();

  // Set the register class of the result of the wire operations to the right
  // value.
  for (MachineFunction::iterator BI = MF.begin(), BE = MF.end();BI != BE;++BI)
    for (MachineBasicBlock::iterator II = BI->getFirstNonPHI(), IE = BI->end();
         II != IE; ++II) {
      if (!VInstrInfo::isWireOp(II->getDesc())) continue;

      unsigned Reg = II->getOperand(0).getReg();
      MRI.setRegClass(Reg, VTM::WireRegisterClass);
    }

  // Forward the user of wire operations.
  SmallVector<unsigned, 8> WireOperands;
  SmallPtrSet<MachineInstr*, 8> VisitedInsts;

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


  return true;
}

Pass *llvm::createForwardWireUsersPass() {
  return new ForwardWireUsers();
}
