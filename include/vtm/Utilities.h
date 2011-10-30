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

#include "llvm/ADT/StringExtras.h"
#include "llvm/Support/raw_ostream.h"

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
      Result += 'A'+(S[i]&15);
      Result += 'A'+((S[i]>>4)&15);
      Result += '_';
    }
    return Result;
}

// PrintEscapedString - Print each character of the specified string, escaping
// it if it is not printable or if it is an escape char.
static inline void PrintEscapedString(const char *Str, unsigned Length,
                                      raw_ostream &Out) {
    for (unsigned i = 0; i != Length; ++i) {
      unsigned char C = Str[i];
      if (isprint(C) && C != '\\' && C != '"')
        Out << C;
      else if (C == '\\')
        Out << "\\\\";
      else if (C == '\"')
        Out << "\\\"";
      else if (C == '\t')
        Out << "\\t";
      else
        Out << "\\x" << hexdigit(C >> 4) << hexdigit(C & 0x0F);
    }
}

// PrintEscapedString - Print each character of the specified string, escaping
// it if it is not printable or if it is an escape char.
static inline void PrintEscapedString(const std::string &Str, raw_ostream &Out) {
  PrintEscapedString(Str.c_str(), Str.size(), Out);
}

class Module;
class TargetData;
// Allow other pass to run script against the GlobalVariables.
void bindGlobalVariablesToEngine(Module &M, TargetData *TD);
}

#endif
