//===-- Utilities.h - Utilities Functions for Verilog Backend ---*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements some utilities functions for Verilog Backend.
//
//===----------------------------------------------------------------------===//
#ifndef VTM_UTILITIES_H
#define VTM_UTILITIES_H

namespace llvm {
inline unsigned getByteEnable(unsigned SizeInBytes) {
  return (0x1 << SizeInBytes) - 1;
}

}

#endif
