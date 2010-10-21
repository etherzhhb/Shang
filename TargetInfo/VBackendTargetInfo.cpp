//===-- VBackendTargetInfo.cpp - CBackend Target Implementation -----------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

//#include "VTargetMachine.h"

#include "llvm/Module.h"
#include "llvm/Target/TargetRegistry.h"

namespace llvm {
  //extern Target TheVBackendTarget;
  Target TheVBackendTarget;
}

using namespace llvm;


extern "C" void LLVMInitializeVerilogBackendTargetInfo() { 
  RegisterTarget<> X(TheVBackendTarget, "verilog", "Verilog backend");
}
