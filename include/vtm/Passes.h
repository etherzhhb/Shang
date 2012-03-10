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
class VTargetMachine;

FunctionPass *createVISelDag(VTargetMachine &TM);

Pass *createLowerFrameInstrsPass(const TargetIntrinsicInfo &IntrinsicInfo);

// Always inline function.
Pass *createAlwaysInlineFunctionPass();

//Convert the AllocaInst to GlobalVariable.
Pass *createStackToGlobalPass();

Pass *createFunctionFilterPass(raw_ostream &O);


// Bit level information analysis
Pass *createBitLevelInfoPass();
Pass *createLogicSynthesisPass();
Pass *createFixMachineCodePass(bool IsPreOpt);
Pass *createFixTerminatorsPass();
Pass *createForwardWireUsersPass();
Pass *createMergeFallThroughBlocksPass();
Pass *createPrebindMuxPass();
Pass *createVPreRegAllocSchedPass();

// Scheduling pass.
Pass *createVPreRegAllocSchedPass();

// Register allocation.
FunctionPass *createSimpleRegisterAllocator();

// Find Shortest Path.
Pass *createCFGShortestPathPass();

// Analyse the Combination Path Delay.
Pass *createCombPathDelayAnalysisPass();

// Analysis the dependency between registers
Pass *createRtlSSAAnalysisPass();

// RTL code generation.
Pass *createVerilogASTBuilderPass();
Pass *createVerilogASTWriterPass(raw_ostream &O);
Pass *createRTLCodegenPreparePass();

Pass *createScriptingPass(const char *Name, const char *FScript,
                          const char *GScript);


//ContoBram Pass
Pass *createContoBromPass(const TargetIntrinsicInfo &IntrinsicInfo);

//
void initializeBitLevelInfoPass(PassRegistry &Registry);

//Add the initialization implementation of StackToGlobal.cpp.
void initializeStackToGlobalPass(PassRegistry &Registry);
void initializeCFGShortestPathPass(PassRegistry &Registry);

void initializeBBDelayAnalysisPass(PassRegistry &Registry);
void initializeCombPathDelayAnalysisPass(PassRegistry &Registry);
void initializeRtlSSAAnalysisPass(PassRegistry &Registry);

void initializeVerilogASTBuilderPass(PassRegistry &Registry);
void initializeVerilogASTWriterPass(PassRegistry &Registry);
void initializeFunctionFilterPass(PassRegistry &Registry); 
void initializeRAPass(PassRegistry &Registry);
void initializeAlwaysInlineFunctionPass(PassRegistry &Registry);
void initializeHWPartitionInfoPass(PassRegistry &Registry);
} // end namespace


#endif
