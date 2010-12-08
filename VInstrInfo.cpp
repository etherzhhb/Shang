//===---------- VInstrInfo.cpp - VTM Instruction Information -----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the VTM implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "vtm/VInstrInfo.h"
#include "vtm/VTargetMachine.h"
#include "vtm/VTM.h"

#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/Support/ErrorHandling.h"

#include "VGenInstrInfo.inc"

using namespace llvm;

VInstrInfo::VInstrInfo(const TargetData &TD, const TargetLowering &TLI)
  : TargetInstrInfoImpl(VTMInsts, array_lengthof(VTMInsts)), RI(*this, TD, TLI)
  {}

FuncUnitId VTFInfo::getPrebindFUId()  const {
  // Dirty Hack: Bind all memory access to channel 0 at this moment.
  if (getTID().Opcode == VTM::VOpMemAccess)
    return FuncUnitId(VFUs::MemoryBus, 0);

  return FuncUnitId();
}
