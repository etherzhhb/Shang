//===-- VMCAsmInfo.cpp - VTM asm properties -------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the declarations of the VMCAsmInfo properties.
//
//===----------------------------------------------------------------------===//

#include "VMCAsmInfo.h"

using namespace llvm;

VMCAsmInfo::VMCAsmInfo(const Target &T, StringRef TT) {
  CommentString = "//";
  AllowPeriodsInName = false;
  HasSetDirective = false;
}
