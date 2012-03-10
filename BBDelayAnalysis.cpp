//===-- BBDelayAnalysis.cpp - Compute Delay for MachineBasicBlocks --------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
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
  CycleLatencyInfo CL;

  for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I) {
    MachineBasicBlock *MBB = I;
    BBDelay.insert(std::make_pair(MBB, CL.computeLatency(*MBB, true)));
  }

  return false;
}

char BBDelayAnalysis::ID = 0;
