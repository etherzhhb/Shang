//===- BBDelayAnalysis.h - Compute Delay for MachineBasicBlocks -*- C++ -*-===//
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
// Definition of analysis that compute delay for MachineBasicBlocks in a
// MachineFunction and cache the result.
//
//===----------------------------------------------------------------------===//

#ifndef BB_DELAY_ANALYSIS_H
#define BB_DELAY_ANALYAIS_H

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/ADT/DenseMap.h"

namespace llvm {
class MachineBasicBlock;

class BBDelayAnalysis : public MachineFunctionPass {
  DenseMap<const MachineBasicBlock*, unsigned> BBDelay;
public:
  static char ID;

  BBDelayAnalysis();

  void getAnalysisUsage(AnalysisUsage &AU) const {
    MachineFunctionPass::getAnalysisUsage(AU);
    AU.setPreservesAll();
  }

  virtual void releaseMemory() { BBDelay.clear(); }

  bool runOnMachineFunction(MachineFunction &MF);

  unsigned getBBDelay(const MachineBasicBlock *MBB) const {
    unsigned Delay = BBDelay.lookup(MBB);
    assert(Delay && "MBB not found?");
    return Delay;
  }

  void updateDelay(const MachineBasicBlock *MBB, unsigned Delay) {
    BBDelay[MBB] = Delay;
  }
};
}

#endif
