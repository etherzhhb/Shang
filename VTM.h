//===-- VBackend.h - Top-level interface for Verilog backend -----*- C++ -*-===//
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

#include "llvm/Target/TargetMachine.h"

namespace llvm {
class FunctionPass;
struct VTargetMachine;

FunctionPass *createVISelDag(VTargetMachine &TM, CodeGenOpt::Level OptLevel);

extern Target TheVBackendTarget;

} // end namespace llvm

// Defines symbolic names for the Verilog TargetMachine registers. This defines
// a mapping from register name to register number.
#include "VGenRegisterNames.inc"

#include "VGenInstrNames.inc"

// Helper macro to switch opcodes.
#define CASEVOP(OPC)  \
  case VTM::VOp##OPC##i1:  \
  case VTM::VOp##OPC##i8:  \
  case VTM::VOp##OPC##i16: \
  case VTM::VOp##OPC##i32: \
  case VTM::VOp##OPC##i64

#endif
