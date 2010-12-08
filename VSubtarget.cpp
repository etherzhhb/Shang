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

#define DEBUG_TYPE "vtmconfig"
#include "vtm/VSubtarget.h"

#define VTMSubtarget VSubtarget
#include "VGenSubtarget.inc"

#include "vtm/VTM.h"

#include "llvm/Support/Debug.h"

using namespace llvm;

VSubtarget::VSubtarget(const std::string &TT,
                     const std::string &FS) {
  std::string CPU = "generic";
  // Parse features string.
  ParseSubtargetFeatures(FS, CPU);
}
