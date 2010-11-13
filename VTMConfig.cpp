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
#include "VTMConfig.h"

#include "VFunctionUnit.h"

#define VTMSubtarget VTMConfig
#include "VGenSubtarget.inc"

#include "VTM.h"

#include "llvm/Support/Debug.h"

using namespace llvm;

VTMConfig::VTMConfig(const std::string &TT,
                     const std::string &FS) {
  std::string CPU = "generic";
  // Parse features string.
  ParseSubtargetFeatures(FS, CPU);
}