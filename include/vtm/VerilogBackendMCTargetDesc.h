//===-- VBackend.h - Top-level interface for Verilog backend ----*- C++ -*-===//
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
