//===- DatapathCodeMotion.cpp - Move the data-path operations ---*- C++ -*-===//
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
// This file contains the VTM implementation of the DatapathCodeMotion pass.
//
//===----------------------------------------------------------------------===//

#include "vtm/Utilities.h"
#include "vtm/VInstrInfo.h"
#include "vtm/VerilogBackendMCTargetDesc.h"

#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineDominators.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"

namespace llvm {
bool hoistDatapathOp(MachineInstr *MI, MachineDominatorTree  *DT,
                     MachineRegisterInfo *MRI) {
  assert(VInstrInfo::isDatapath(MI->getOpcode()) && "Expect datapath operation!");

  MachineBasicBlock *MBBToHoist = DT->getRoot();
  MachineBasicBlock *CurMBB = MI->getParent();

  for (unsigned i = 0, e = MI->getNumOperands(); i != e; ++i) {
    const MachineOperand &MO = MI->getOperand(i);

    if (!MO.isReg() || MO.isDef() || !MO.getReg())  continue;

    MachineInstr *DefMI = MRI->getVRegDef(MO.getReg());
    assert(DefMI && "Not in SSA form!");

    MachineBasicBlock *DefMBB = DefMI->getParent();
    if (DT->dominates(MBBToHoist, DefMBB)) {
      MBBToHoist = DefMBB;
      continue;
    }

    assert(DT->dominates(DefMBB, MBBToHoist)
           && "Operands not in a path of the dominator tree!");
  }

  if (MBBToHoist == CurMBB) return false;

  MI->removeFromParent();

  MachineBasicBlock::instr_iterator IP = MBBToHoist->getFirstInstrTerminator();

  // Insert the imp_def before the PHI incoming copies.
  while (llvm::prior(IP)->getOpcode() == VTM::VOpMvPhi)
    --IP;

  MBBToHoist->insert(IP, MI);
  return true;
}

bool hoistDatapathOpInMBB(MachineBasicBlock *MBB, MachineDominatorTree *DT,
                          MachineRegisterInfo *MRI) {
  bool MadeChange = false;

  typedef MachineBasicBlock::instr_iterator instr_it;
  for (instr_it I = MBB->instr_begin(), E = MBB->instr_end(); I != E; /*++I*/) {
    MachineInstr *MI = I++;

    if (VInstrInfo::isDatapath(MI->getOpcode()))
      MadeChange |= hoistDatapathOp(MI, DT, MRI);
  }

  return MadeChange;
}

bool hoistDatapathOpInSuccs(MachineBasicBlock *MBB, MachineDominatorTree *DT,
                            MachineRegisterInfo *MRI) {
  bool MadeChange = false;
  typedef MachineBasicBlock::succ_iterator succ_iterator;
  for (succ_iterator I = MBB->succ_begin(), E = MBB->succ_end(); I != E; ++I)
    MadeChange |= hoistDatapathOpInMBB(MBB, DT, MRI);

  return MadeChange;
}

struct HoistDatapathPass : public MachineFunctionPass {
  static char ID;
  HoistDatapathPass() : MachineFunctionPass(ID) {}

  void getAnalysisUsage(AnalysisUsage &AU) const {
    AU.setPreservesCFG();
    AU.addRequired<MachineDominatorTree>();
    MachineFunctionPass::getAnalysisUsage(AU);
  }

  bool runOnMachineFunction(MachineFunction &MF) {
    bool Changed = false;
    MachineDominatorTree &DT = getAnalysis<MachineDominatorTree>();
    MachineRegisterInfo &MRI = MF.getRegInfo();

    for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I)
      Changed |= hoistDatapathOpInMBB(I, &DT, &MRI);

    return Changed;
  }
};

Pass *createHoistDatapathPass() {
  return new llvm::HoistDatapathPass();
}
}

char llvm::HoistDatapathPass::ID = 0;
