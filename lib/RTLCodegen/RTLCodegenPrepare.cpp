//====- RTLCodegenPrepare.cpp - Perpare for RTL code generation -*- C++ -*-===//
//
//                      The Shang HLS frameowrk                               //
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implement the RTLCodegenPrepare pass, fix the Machine code to keep
// code in RTLCodegen simple.
//
//===----------------------------------------------------------------------===//

#include "vtm/VerilogBackendMCTargetDesc.h"
#include "vtm/Passes.h"
#include "vtm/VFInfo.h"
#include "vtm/VRegisterInfo.h"
#include "vtm/VInstrInfo.h"

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
      for (MachineBasicBlock::instr_iterator II = I->instr_begin(),
           IE = I->instr_end(); II != IE; /*++II*/) {
        MachineInstr *MI = II;
        ++II;

        if (MI->getOpcode() == VTM::VOpReadFU &&
            (!MI->getOperand(0).isReg() || !MI->getOperand(0).getReg()) &&
            VInstrInfo::getPreboundFUId(MI).isTrivial()) {
          I->erase_instr(MI);
        } else if (MI->isImplicitDef()) {
          unsigned Reg = MI->getOperand(0).getReg();
          MRI.replaceRegWith(Reg, 0);
          MI->eraseFromParent();
        }
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
