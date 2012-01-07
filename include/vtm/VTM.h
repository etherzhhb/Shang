//===-- VBackend.h - Top-level interface for Verilog backend ----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the entry points for global functions defined in the LLVM
// Verilog back-end.
//
//===----------------------------------------------------------------------===//

#ifndef VBACKEND_H
#define VBACKEND_H

#include "vtm/VerilgoBackendMCTargetDesc.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm {
class FunctionPass;
class VTargetMachine;

FunctionPass *createVISelDag(VTargetMachine &TM);
} // end namespace llvm
#endif
