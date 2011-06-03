//===- EliminateMvImm.cpp - Eliminate the redundant MvImms ------*- C++ -*-===//
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
// This file implement a pass that eliminate the redundant MvImm instruction
// that only copy a constant to a register used in a same block.
//
//===----------------------------------------------------------------------===//
#include "vtm/Passes.h"
#include "vtm/VTM.h"

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Target/TargetInstrInfo.h"

#include "llvm/Support/raw_ostream.h"

#define DEBUG_TYPE "elim-set-ri"
#include "llvm/Support/Debug.h"

using namespace llvm;

namespace {
struct ElimMvImm : public MachineFunctionPass {
  static char ID;

  ElimMvImm() : MachineFunctionPass(ID) {}

  void getAnalysisUsage(AnalysisUsage &AU) const {
    MachineFunctionPass::getAnalysisUsage(AU);
    // Is this true?
    // AU.setPreservesAll();
  }

  bool runOnMachineFunction(MachineFunction &MF);
};
}

char ElimMvImm::ID = 0;

bool ElimMvImm::runOnMachineFunction(MachineFunction &MF) {
  bool Changed = false;
  MachineRegisterInfo &MRI = MF.getRegInfo();

  std::vector<MachineInstr*> Worklist;

   // Find out all VOpMvImm.
  for (MachineFunction::iterator BI = MF.begin(), BE = MF.end(); BI != BE; ++BI)
    for (MachineBasicBlock::iterator II = BI->begin(), IE = BI->end();
         II != IE; ++II)
      // Only replace constant.
      if (II->getOpcode() == VTM::VOpMvImm && II->getOperand(1).isImm())
        Worklist.push_back(II);

  // Try to replace the register operand with the constant for users of VOpMvImm.
  while (!Worklist.empty()) {
    MachineInstr *MI = Worklist.back();
    Worklist.pop_back();

    unsigned DstReg = MI->getOperand(0).getReg();

    // Find all replaceable operand.
    std::vector<MachineOperand*> ToReplaces;
    for (MachineRegisterInfo::use_iterator I = MRI.use_begin(DstReg),
         E = MRI.use_end(); I != E; ++I) {
      MachineOperand &MO = I.getOperand();
      
      // Only replace if user is not a PHINode.
      if (I->getOpcode() == VTM::PHI) continue;

      ToReplaces.push_back(&MO);
    }

    // Perform the replacement.
    int64_t Imm = MI->getOperand(1).getImm();
    
    Changed |= !ToReplaces.empty();

    while (!ToReplaces.empty()) {
      ToReplaces.back()->ChangeToImmediate(Imm);
      ToReplaces.pop_back();
    }

    // Eliminate the instruction if it dead.
    if (MRI.use_empty(DstReg)) MI->eraseFromParent();
  }

  return Changed;
}

Pass *llvm::createElimMvImmPass() {
  return new ElimMvImm();
}
