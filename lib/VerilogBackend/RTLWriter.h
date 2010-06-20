//===------------- RTLWriter.h - HWAtom to RTL verilog  ---------*- C++ -*-===//
//
//                            The Verilog Backend
//
// Copyright: 2010 by Hongbin Zheng. all rights reserved.
// IMPORTANT: This software is supplied to you by Hongbin Zheng in consideration
// of your agreement to the following terms, and your use, installation, 
// modification or redistribution of this software constitutes acceptance
// of these terms.  If you do not agree with these terms, please do not use, 
// install, modify or redistribute this software. You may not redistribute, 
// install copy or modify this software without written permission from 
// Hongbin Zheng. 
//
//===----------------------------------------------------------------------===//
//
// This file define the RTLWriter pass, which write out HWAtom into RTL
// verilog form.
//
//===----------------------------------------------------------------------===//
#ifndef VBE_RTL_WRITER_H
#define VBE_RTL_WRITER_H

#include "llvm/Pass.h"
#include "llvm/Function.h"
#include "llvm/Instructions.h"
#include "llvm/Constants.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Module.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/Support/CFG.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/DerivedTypes.h"
#include "llvm/Target/Mangler.h"

#include "vbe/utils.h"
#include "vbe/ResourceConfig.h"

#include "HWAtomInfo.h"
#include "VLang.h"

using namespace llvm;


namespace esyn {

class RTLWriter : public FunctionPass {
  raw_ostream &Out;

  void clear();

public:
  /// @name FunctionPass interface
  //{
  static char ID;
  explicit RTLWriter(raw_ostream &O)
    : FunctionPass(&ID), Out(O) {}

  bool runOnFunction(Function &F);
  void releaseMemory() { clear(); }
  void getAnalysisUsage(AnalysisUsage &AU) const;
  virtual void print(raw_ostream &O, const Module *M) const;
  //}
};

} //end of namespace
#endif // h guard
