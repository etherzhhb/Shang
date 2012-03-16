//===-- BBDelayAnalysis.cpp - Compute Delay for MachineBasicBlocks --------===//
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
// Implementation of the BBDelayAnalysis pass, which simply compute delay for
// MachineBasicBlocks in a MachineFunction and cache the result.
//
//
//===----------------------------------------------------------------------===//
#include "BBDelayAnalysis.h"

#include "vtm/Passes.h"
#include "vtm/VInstrInfo.h"

#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineBasicBlock.h"

using namespace llvm;

INITIALIZE_PASS(BBDelayAnalysis, "vtm-bb-delay",
                "VTM - MachineBasicBlock Delay Analysis",
                false, true)

BBDelayAnalysis::BBDelayAnalysis() : MachineFunctionPass(ID) {
  initializeBBDelayAnalysisPass(*PassRegistry::getPassRegistry());
}

bool BBDelayAnalysis::runOnMachineFunction(MachineFunction &MF) {
  CycleLatencyInfo CL(MF.getRegInfo());

  for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I) {
    MachineBasicBlock *MBB = I;
    BBDelay.insert(std::make_pair(MBB, CL.computeLatency(*MBB, true)));
  }

  return false;
}

char BBDelayAnalysis::ID = 0;
