//===-- MemOpsFusing - Fuse MemOps to Boost Speed Performance  --*- C++ -*-===//
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
// This file implement the MemOpsFusing Pass, which fully utilize the bandwidth
// of memory bus to boost the speed performance of the design.
//
//===----------------------------------------------------------------------===//

#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"

using namespace llvm;

namespace {
struct MemOpsFusing : public MachineFunctionPass {
  static char ID;

  MemOpsFusing() : MachineFunctionPass(ID) {}

  void getAnalysisUsage(AnalysisUsage &AU) const {
    MachineFunctionPass::getAnalysisUsage(AU);
  }

  bool runOnMachineFunction(MachineFunction &MF) {
    bool changed = false;

    for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I)
      changed |= runOnMachineBasicBlock(*I);

    return changed;
  }

  bool runOnMachineBasicBlock(MachineBasicBlock &MBB);
};
}

char MemOpsFusing::ID = 0;

bool MemOpsFusing::runOnMachineBasicBlock(MachineBasicBlock &MBB) {
  // 1. Collect the fusing candidates.
  // 2. Check if we can actually fuse them.
  // 3. Fuse them.
  return false;
}
