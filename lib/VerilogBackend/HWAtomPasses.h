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
  class Pass;
}

using namespace llvm;

namespace esyn {

// Simple List Scheduler
Pass *createListSchedulePass();

// Reduce un-necessary registers
Pass *createRegisterReductionPass();

} // end namespace

#endif