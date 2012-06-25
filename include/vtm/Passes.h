//===--------- Passes.h - Passes for Verilog target machine -----*- C++ -*-===//
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

extern char &AdjustLIForBundlesID;

FunctionPass *createVISelDag(VTargetMachine &TM);

// Always inline function.
Pass *createHLSInlinerPass();
Pass *createTrivialLoopUnrollPass();
Pass *createLoopVectorizerPass();
//Convert the AllocaInst to GlobalVariable.
Pass *createBlockRAMFormation(const TargetIntrinsicInfo &IntrInfo);
Pass *createAllocaAlignerPass();

Pass *createFunctionFilterPass(raw_ostream &O);

// Bit level information analysis
Pass *createBitLevelInfoPass();
Pass *createLogicSynthesisPass();
Pass *createPreSchedRTLOptPass();
Pass *createFixMachineCodePass(bool IsPreOpt);
Pass *createBuildWireTransitiveUsersPass();
Pass *createHyperBlockFormationPass();
Pass *createPrebindUnbalanceMuxPass();
Pass *createBasicPrebindMuxPass();
Pass *createMemOpsFusingPass();
Pass *createVPreRegAllocSchedPass();

// Scheduling pass.
Pass *createVAliasAnalysisPass();
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

//
void initializeVAliasAnalysisPass(PassRegistry &Registry);
void initializeAdjustLIForBundlesPass(PassRegistry &Registry);
void initializePrebindMuxBasePass(PassRegistry &Registry);
void initializePrebindUnbalanceMuxPass(PassRegistry &Registry);
void initializeBitLevelInfoPass(PassRegistry &Registry);

void initializeCFGShortestPathPass(PassRegistry &Registry);

void initializeHyperBlockFormationPass(PassRegistry &Registry);
void initializeBBDelayAnalysisPass(PassRegistry &Registry);

void initializeCombPathDelayAnalysisPass(PassRegistry &Registry);
void initializeRtlSSAAnalysisPass(PassRegistry &Registry);
void initializeVerilogASTBuilderPass(PassRegistry &Registry);
void initializeVerilogASTWriterPass(PassRegistry &Registry);
void initializeFunctionFilterPass(PassRegistry &Registry); 
void initializeRAPass(PassRegistry &Registry);
void initializeHLSInlinerPass(PassRegistry &Registry);
void initializeTrivialLoopUnrollPass(PassRegistry &Registry);
void initializeLoopVectorizerPass(PassRegistry &Registry);
} // end namespace


#endif
