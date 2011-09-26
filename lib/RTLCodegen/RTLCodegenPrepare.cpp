//====- RTLCodegenPrepare.cpp - Perpare for RTL code generation -*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
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

#include "vtm/Passes.h"
#include "vtm/VFInfo.h"
#include "vtm/VRegisterInfo.h"
#include "vtm/VerilogAST.h"
#include "vtm/MicroState.h"

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/Passes.h"

using namespace llvm;
namespace {
struct RTLCodegenPreapare : public MachineFunctionPass {
  VFInfo *FInfo;
  // Mapping the PHI number to accutally register.
  std::map<unsigned, unsigned> PHIsMap;
  static char ID;

  RTLCodegenPreapare() : MachineFunctionPass(ID) {
    initializePHIEliminationPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) {
    EliminatePseudoPHIs(MF.getRegInfo());
    return true;
  }

  void getAnalysisUsage(AnalysisUsage &AU) const {
    MachineFunctionPass::getAnalysisUsage(AU);
    AU.addRequiredID(PHIEliminationID);
    AU.setPreservesAll();
  }

  void EliminatePseudoPHIs(MachineRegisterInfo &MRI);

  const char *getPassName() const {
    return "RTL Code Generation Preparation Pass";
  }
};
}

char RTLCodegenPreapare::ID = 0;

Pass *llvm::createRTLCodegenPreparePass() {
  return new RTLCodegenPreapare();
}

void RTLCodegenPreapare::EliminatePseudoPHIs(MachineRegisterInfo &MRI) {
  const std::vector<unsigned> &PHIs =
    MRI.getRegClassVirtRegs(VTM::PHIRRegisterClass);

  for (std::vector<unsigned>::const_iterator I = PHIs.begin(), E = PHIs.end();
       I != E; ++I) {
    unsigned PHINum = *I;
    MachineRegisterInfo::use_iterator UI = MRI.use_begin(PHINum);
    assert(MRI.hasOneUse(PHINum) && "PHI Information broken!");

    ucOp PHIDef = ucOp::getParent(UI);
    assert(PHIDef->getOpcode() == VTM::VOpDefPhi && "PHI Information broken!");
    unsigned PHIDst = PHIDef.getOperand(0).getReg();
    // Only fix the PHIs for wire register.
    if (!VRegisterInfo::IsWire(PHIDst, &MRI)) continue;

    PHIDef->changeOpcode(VTM::IMPLICIT_DEF, PHIDef->getPredSlot());
    unsigned NumDef = 0;
    typedef MachineRegisterInfo::def_iterator def_it;
    while (!MRI.def_empty(PHINum)){
      ucOp PHIUse = ucOp::getParent(MRI.def_begin(PHINum));
      PHIUse->changeOpcode(VTM::VOpMove_ww, PHIUse->getPredSlot());
      PHIUse.getOperand(0).setReg(PHIDst);
      ++NumDef;
    }
    assert(NumDef == 1 && "Broken PHINode for wire!");
    (void) NumDef;
  }
}
