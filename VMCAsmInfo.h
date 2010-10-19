//===----------- VMCAsmInfo.h - VTM asm properties -------------*- C++ -*--====//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the declaration of the VMCAsmInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef VTARGETASMINFO_H
#define VTARGETASMINFO_H

#include "llvm/ADT/StringRef.h"
#include "llvm/MC/MCAsmInfo.h"

namespace llvm {
class Target;

struct VMCAsmInfo : public MCAsmInfo {
  explicit VMCAsmInfo(const Target &T, StringRef TT);
};

} // namespace llvm

#endif
