//===---- VAliasAnalysis.cpp - VTM Specific Alias Analysis  -----*- C++ -*-===//
//
// Copyright: 2011 by SYSU EDA Group. all rights reserved.
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
// This file implement the Verilog Target Machine specific alias analysis.
//
//===----------------------------------------------------------------------===//
#include "vtm/Passes.h"

#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/Analysis/ScalarEvolutionExpressions.h"
#include "llvm/Pass.h"

using namespace llvm;

namespace {
struct VAliasAnalysis: public FunctionPass, public AliasAnalysis {
  static char ID;

  VAliasAnalysis() : FunctionPass(ID) {
    initializeVAliasAnalysisPass(*PassRegistry::getPassRegistry());
  }

  virtual void *getAdjustedAnalysisPointer(AnalysisID ID) {
    if (ID == &AliasAnalysis::ID)
      return (AliasAnalysis*)this;
    return this;
  }

  virtual bool runOnFunction(Function &F) {
    InitializeAliasAnalysis(this);
    return false;
  }

  void getAnalysisUsage(AnalysisUsage &AU) const {
    AU.setPreservesAll();
    AliasAnalysis::getAnalysisUsage(AU);
  }

  AliasResult alias(const Location &LocA, const Location &LocB) {
    PointerType *ATy = cast<PointerType>(LocA.Ptr->getType()),
                *BTy = cast<PointerType>(LocB.Ptr->getType());

    // Pointers pointing to different address space never alias.
    if (ATy->getAddressSpace() != BTy->getAddressSpace())
      return NoAlias;

    // TODO: Constant expression.

    return AliasAnalysis::alias(LocA, LocB);
  }
};

}

char VAliasAnalysis::ID = 0;

INITIALIZE_AG_PASS(VAliasAnalysis, AliasAnalysis, "vaa",
                   "VTM Specific Alias Analyais", false, true, false);

Pass *llvm::createVAliasAnalysisPass() {
  return new VAliasAnalysis();
}
