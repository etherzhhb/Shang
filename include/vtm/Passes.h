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
 
// Promote all the GVs to arguments
Pass *createGVPromotionPass();

// Always inline function.
Pass *createAlwaysInlineFunctionPass();

//Convert the AllocaInst to GlobalVariable.
Pass *createStackToGlobalPass();

Pass *createHWPartitionInfoPass();

Pass *createFunctionFilterPass(raw_ostream &O);


// Bit level information analysis
Pass *createBitLevelInfoPass();
Pass *createBitLevelABCOptPass();

Pass *createFixMachineCodePass();
Pass *createFixTerminatorsPass();
Pass *createForwardWireUsersPass();
Pass *createMergeFallThroughBlocksPass();
Pass *createVIfConverterPass();
Pass *createVPreRegAllocSchedPass();

// Scheduling pass.
Pass *createVPreRegAllocSchedPass();

// Register allocation.
FunctionPass *createSimpleRegisterAllocator();

// Find Shortest Path.
Pass *createFindShortestPathPass();

// RTL code generation.
Pass *createRTLCodegenPass(raw_ostream &O);
Pass *createRTLCodegenPreparePass();

Pass *createScriptingPass(const char *Name, const char *FScript,
                          const char *GScript);


//ContoBram Pass
Pass *createContoBromPass(const TargetIntrinsicInfo &IntrinsicInfo);

//
void initializeVIfConverterPass(PassRegistry &Registry);
void initializeBitLevelInfoPass(PassRegistry &Registry);

void initializeBitLevelABCOptPass(PassRegistry &Registry);

//Add the initialization implementation of StackToGlobal.cpp.
void initializeStackToGlobalPass(PassRegistry &Registry);

void initializeFindShortestPathPass(PassRegistry &Registry);

void initializeRTLCodegenPass(PassRegistry &Registry);
void initializeFunctionFilterPass(PassRegistry &Registry); 

void initializeRAPass(PassRegistry &Registry);

// Add the initialize implementation of GVPromotion.cpp.
void initializeGVPromotionPass(PassRegistry &Registry);

void initializeAlwaysInlineFunctionPass(PassRegistry &Registry);

void initializeHWPartitionInfoPass(PassRegistry &Registry);
} // end namespace


#endif
