//===-- VBackendTargetInfo.cpp - CBackend Target Implementation -----------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "llvm/Support/TargetRegistry.h"

namespace llvm {
  Target TheVBackendTarget;
}

using namespace llvm;


extern "C" void LLVMInitializeVerilogBackendTargetInfo() { 
  RegisterTarget<> X(TheVBackendTarget, "verilog", "Verilog backend");
}

extern "C" void LLVMInitializeCBackendTargetMC() {}
