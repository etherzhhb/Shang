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

#include "VInstrInfo.h"
#include "VTM.h"
#include "VTMConfig.h"

#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/Support/ErrorHandling.h"

#include "VGenInstrInfo.inc"

using namespace llvm;

VInstrInfo::VInstrInfo() : TargetInstrInfoImpl(VTMInsts, array_lengthof(VTMInsts)),
  RI(*this) {}

unsigned VTIDReader::getLatency(const VTMConfig &VTC) const {
  VInstrInfo::FUTypes ResTy =getHWResType();

  if (ResTy == VInstrInfo::Trivial)
    return getTrivialLatency();

  return VTC.getResType(ResTy)->getLatency();
}
