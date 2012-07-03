//===- AdjustLIForBundles.cpp - Adjust live intervals for bundles -*- C++ -*-=//
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
// This file implement the pass that eliminate the dead memory operations
// at machine-code level.
//
//===----------------------------------------------------------------------===//

#include "vtm/VerilogBackendMCTargetDesc.h"
#include "vtm/VInstrInfo.h"
#include "vtm/Passes.h"
#include "vtm/Utilities.h"

#include "llvm/Analysis/AliasSetTracker.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/ADT/PointerIntPair.h"
#include "llvm/ADT/SetOperations.h"
#include "llvm/ADT/Statistic.h"
#define DEBUG_TYPE "vtm-dead-memop-elimination"
#include "llvm/Support/Debug.h"

using namespace llvm;

STATISTIC(MemOpEliminated, "Number of dead memory operations eliminated");

namespace {
struct DeadMemOpElimination : public MachineFunctionPass {
  static char ID;

  DeadMemOpElimination() : MachineFunctionPass(ID) {}

  void getAnalysisUsage(AnalysisUsage &AU) const {
    AU.addRequired<AliasAnalysis>();
    AU.addPreserved<AliasAnalysis>();
    AU.setPreservesCFG();
    MachineFunctionPass::getAnalysisUsage(AU);
  }


  bool runOnMachineBasicBlock(MachineBasicBlock &MBB, AliasAnalysis &AA);

  bool runOnMachineFunction(MachineFunction &MF) {
    AliasAnalysis &AA = getAnalysis<AliasAnalysis>();
    bool changed = false;

    for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I)
      changed |= runOnMachineBasicBlock(*I, AA);

    MF.verify(this, "After Memory operations fusion.");

    return changed;
  }
};
}

char DeadMemOpElimination::ID = 0;

bool DeadMemOpElimination::runOnMachineBasicBlock(MachineBasicBlock &MBB,
                                                  AliasAnalysis &AA) {
  AliasSetTracker AST(AA);
  return true;
}
