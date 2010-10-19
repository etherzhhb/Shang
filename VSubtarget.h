//===-------- VSubtarget.h - Define Subtarget for the VTM --------*- C++ -*-====//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares the BLACKFIN specific subclass of TargetSubtarget.
//
//===----------------------------------------------------------------------===//

#ifndef VSUBTARGET_H
#define VSUBTARGET_H

#include "llvm/Target/TargetSubtarget.h"
#include <string>

namespace llvm {

class VSubtarget : public TargetSubtarget {
public:
  VSubtarget(const std::string &TT, const std::string &FS);
};

} // end namespace llvm

#endif
