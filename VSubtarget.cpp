//===-------------- VSubtarget.cpp - VTM Subtarget Information -------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the vtm specific subclass of TargetSubtarget.
//
//===----------------------------------------------------------------------===//
#include "VSubtarget.h"
#define VTMSubtarget VSubtarget

#include "vtm/VTM.h"

#define GET_SUBTARGETINFO_TARGET_DESC
#include "VerilogBackendGenSubtargetInfo.inc"

using namespace llvm;

VSubtarget::VSubtarget(StringRef TT, StringRef FS) {
  StringRef CPU = "generic";
  // Parse features string.
  //ParseSubtargetFeatures(FS, CPU);
}
