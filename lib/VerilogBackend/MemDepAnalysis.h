//===-- MemDepAnalyzsis.h - Preform Memory dependencies analyze  --*- C++ -*-===//
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
// This file define the MemDepAnalyzsis pass, which provide the memory
// dependencies information between two memory operation.
//
//===----------------------------------------------------------------------===//
//
#ifndef VBE_MEMORY_DEPENDENCIES_ANALYSIS_H
#define VBE_MEMORY_DEPENDENCIES_ANALYSIS_H

#include "vbe/HWAtom.h"


#include "llvm/Instructions.h"
#include "llvm/Function.h"
#include "llvm/Pass.h"
#include "llvm/Target/TargetData.h"
#include <vector>

namespace llvm {
  class LoopInfo;
  class AliasAnalysis;
  class TargetData;
  class ScalarEvolution;
}
using namespace llvm;

namespace esyn {
class HWAtomInfo;

class MemDepInfo : public FunctionPass {
public:
  struct DepInfo {
    HWMemDep::MemDepTypes Dep  : 2;
    unsigned ItDst                  : 30;

    DepInfo(HWMemDep::MemDepTypes dep, unsigned itDst);

    DepInfo() : Dep(HWMemDep::NoDep), ItDst(0) {}

    bool hasDep() const {
      return Dep != HWMemDep::NoDep ;
    }
  };
private:
  AliasAnalysis *AA;
  TargetData *TD;
  ScalarEvolution *SE;
  LoopInfo *LI;

  DepInfo advancedDepAnalysis(GetElementPtrInst *SrcAddr, GetElementPtrInst *DstAddr,
    bool SrcLoad, bool DstLoad, BasicBlock &BB, bool SrcBeforeDest);

  DepInfo AnalyzeDeps(Value *SrcAddr, Value *DstAddr, bool SrcLoad, bool DstLoad,
    BasicBlock &BB, bool SrcBeforeDest);

  DepInfo createDep(bool SrcLoad, bool DstLoad, bool SrcBeforeDest, int Diff = 0);

public:
  static char ID;
  MemDepInfo() : FunctionPass(&ID), AA(0), TD(0), SE(0) {}
  virtual bool runOnFunction(Function &F);

  // getAnalysisUsage
  virtual void getAnalysisUsage(AnalysisUsage &AU) const;

  //get dependence info
  DepInfo getDepInfo(Instruction &Src, Instruction &Dst,
                    BasicBlock &BB, bool SrcBeforeDest);

};
}
#endif
