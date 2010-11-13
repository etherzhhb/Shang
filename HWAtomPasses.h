//===------------- HWAtomPasses.h - Passes run on HWAtoms --------*- C++ -*-===//
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
// HWAtoms optimization passes
//
//===----------------------------------------------------------------------===//
#ifndef VBE_HARDWARE_ATOM_PASSES_H
#define VBE_HARDWARE_ATOM_PASSES_H

namespace llvm {
class LLVMContext;
class Pass;
class raw_ostream;
class TargetMachine;
struct VTargetMachine;

// Simple As Soon As Possible Scheduler
Pass *createASAPSchedulePass();

// Registers Allocation
Pass *createRegisterAllocationPass();

// Compatibility Path Based Binding
Pass *createCompPathBindingPass();

// Topological sort BBs in structural CFG
Pass *createTopSortBBPass();

// Instruction lowering pass.
Pass *createInstLoweringPass();

// Local LEA based register merging pass.
Pass *createLocalLEAPass();

// Scalar Streamization.
Pass *createScalarStreamizationPass();

// HWAtom
Pass *createHWAtonInfoPass(const TargetMachine &TM);

// Vlang
Pass *createVlangPass();

// RTL writer.
Pass *createRTLWriterPass(VTargetMachine &TM, raw_ostream &O);

// Testbench writer.
Pass *createTestBenchWriterPass(raw_ostream &O);

} // end namespace

#endif
