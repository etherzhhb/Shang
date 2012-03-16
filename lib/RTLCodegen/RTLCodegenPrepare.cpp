//====- RTLCodegenPrepare.cpp - Perpare for RTL code generation -*- C++ -*-===//
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
// This file implement the RTLCodegenPrepare pass, fix the Machine code to keep
// code in RTLCodegen simple.
//
//===----------------------------------------------------------------------===//

#include "vtm/Passes.h"
#include "vtm/VFInfo.h"
#include "vtm/VRegisterInfo.h"
#include "vtm/MicroState.h"

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/Passes.h"

using namespace llvm;
namespace {
struct RTLCodegenPreapare : public MachineFunctionPass {
  // Mapping the PHI number to accutally register.
  std::map<unsigned, unsigned> PHIsMap;
  static char ID;

  RTLCodegenPreapare() : MachineFunctionPass(ID) {}

  bool runOnMachineFunction(MachineFunction &MF) {
    MachineRegisterInfo &MRI = MF.getRegInfo();

    for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I)
      for (MachineBasicBlock::iterator II = I->begin(), IE = I->end(); II != IE;
           /*++II*/) {
        MachineInstr *MI = II;
        ++II;

        if (!MI->isImplicitDef()) continue;

        unsigned Reg = MI->getOperand(0).getReg();
        MRI.replaceRegWith(Reg, 0);
        MI->removeFromParent();
      }

    return true;
  }

  const char *getPassName() const {
    return "RTL Code Generation Preparation Pass";
  }
};
}

char RTLCodegenPreapare::ID = 0;

Pass *llvm::createRTLCodegenPreparePass() {
  return new RTLCodegenPreapare();
}
