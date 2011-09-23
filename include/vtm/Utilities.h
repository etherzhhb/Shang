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
static inline unsigned getByteEnable(unsigned SizeInBytes) {
  return (0x1 << SizeInBytes) - 1;
}


static std::string VBEMangle(const std::string &S) {
  std::string Result;

  for (unsigned i = 0, e = S.size(); i != e; ++i)
    if (isalnum(S[i]) || S[i] == '_') {
      Result += S[i];
    } else {
      Result += '_';
      //Result += 'A'+(S[i]&15);
      //Result += 'A'+((S[i]>>4)&15);
      //Result += '_';
    }
    return Result;
}

}

#endif
