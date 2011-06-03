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
class TargetIntrinsicInfo;

Pass *createLowerFrameInstrsPass(const TargetIntrinsicInfo &IntrinsicInfo);

Pass *createFunctionFilterPass(raw_ostream &O);

// Bit level information analysis
Pass *createBitLevelInfoPass();

// Eliminate the VOpMvImm instructions.
Pass *createElimMvImmPass();

// Scheduling pass.
Pass *createVPreRegAllocSchedPass();

Pass *createPHIEliminationPass();

// Register allocation.
FunctionPass *createSimpleRegisterAllocator();

// Fix copy instruction after register allocation
Pass *createCopyEliminationPass();

// RTL code generation.
Pass *createRTLCodegenPass(raw_ostream &O);

Pass *createScriptingPass(const char *Name, const char *Script);

// Verilator interface code generation.
Pass *createVLTIfCodegenPass(raw_ostream &O);

// PLB interface code generation.
Pass *createPLBIfCodegenPass(raw_ostream &O);

//
void initializeBitLevelInfoPass(PassRegistry &Registry);
void initializeRTLCodegenPass(PassRegistry &Registry);

void initializeRAPass(PassRegistry &Registry);
} // end namespace

#endif
