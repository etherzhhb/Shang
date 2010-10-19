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

using namespace llvm;

VSubtarget::VSubtarget(const std::string &TT,
                       const std::string &FS) {
  std::string CPU = "generic";
  // TODO: Parse lua script here.
}
