//===-- VBackend.h - Top-level interface for Verilog backend ----*- C++ -*-===//
//
//                      The Shang HLS frameowrk                               //
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

#ifndef VERILOGBACKEND_MC_TARGET_DESC_H
#define VERILOGBACKEND_MC_TARGET_DESC_H

#include "llvm/Intrinsics.h"
#include "llvm/Target/TargetMachine.h"
namespace llvm {
extern Target TheVBackendTarget;
} // end namespace llvm

namespace vtmIntrinsic {
  enum ID {
    last_non_vtm_intrinsic = llvm::Intrinsic::num_intrinsics - 1,
#define GET_INTRINSIC_ENUM_VALUES
#include "VerilogBackendGenIntrinsics.inc"
#undef GET_INTRINSIC_ENUM_VALUES
    , num_vtm_intrinsics
  };
}

// Defines symbolic names for the Verilog TargetMachine registers. This defines
// a mapping from register name to register number.
#define GET_REGINFO_ENUM
#include "VerilogBackendGenRegisterInfo.inc"

#define GET_INSTRINFO_ENUM
#include "VerilogBackendGenInstrInfo.inc"

#endif
