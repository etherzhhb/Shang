//===- BBDelayAnalysis.h - Compute Delay for MachineBasicBlocks -*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
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
