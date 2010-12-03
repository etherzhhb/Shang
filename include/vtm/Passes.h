//===--------- Passes.h - Passes for Verilog target machine -----*- C++ -*-===//
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
// SUnits optimization passes
//
//===----------------------------------------------------------------------===//
#ifndef VBE_HARDWARE_ATOM_PASSES_H
#define VBE_HARDWARE_ATOM_PASSES_H

namespace llvm {
class LLVMContext;
class Pass;
class FunctionPass;
class raw_ostream;
class TargetMachine;
class PassRegistry;

struct VTargetMachine;
struct VRegisterInfo;

// Bit level information analysis
Pass *createBitLevelInfoPass();

// Scheduling pass.
Pass *createVPreRegAllocSchedPass(const VTargetMachine &TM);

// Register allocation.
FunctionPass *createOptimalSSARegisterAllocator();

// Register allocation.
FunctionPass *createDynPhyRegsBuilderPass(VRegisterInfo &VRI);

// RTL writer.
Pass *createRTLWriterPass(VTargetMachine &TM, raw_ostream &O);

//
void initializeBitLevelInfoPass(PassRegistry &Registry);
void initializeRTLInfoPass(PassRegistry &Registry);

void initializeRAOptimalSSAPass(PassRegistry &Registry);
} // end namespace

#endif
