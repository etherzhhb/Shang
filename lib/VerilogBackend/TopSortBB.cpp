//===-- TopSortBB.cpp - Topological sort BBs in structural CFG  -*- C++ -*-===//
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
// This file implement the TopSortBB pass, which topological sort the BasicBlocks
// in structural CFG.
//
//===----------------------------------------------------------------------===//

#include "HWAtomPasses.h"

#include "llvm/Analysis/RegionInfo.h"
#include "llvm/Analysis/RegionIterator.h"
#include "llvm/Support/CFG.h"

#define DEBUG_TYPE "vbe-top-sort-bb"
#include "llvm/Support/Debug.h"

using namespace llvm;
using namespace esyn;

namespace {
struct TopSortBB : public FunctionPass {
  RegionInfo *RI;
  Function *FN;

  static char ID;
  TopSortBB() : FunctionPass(&ID) {}
  bool runOnFunction(Function &F);
  void getAnalysisUsage(AnalysisUsage &AU) const;
  void verifyAnalysis() const;
  void sortBBInRegion(Region *R);
};
}

void TopSortBB::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<RegionInfo>();
  AU.setPreservesAll();
}

void TopSortBB::verifyAnalysis() const {
  std::set<const BasicBlock*> Visited;
  for (Function::const_iterator I = FN->begin(), E = FN->end(); I != E; ++I) {
    const BasicBlock *BB = I;
    Visited.insert(BB);
    for (const_pred_iterator PI = pred_begin(BB), PE = pred_end(BB);
        PI != PE; ++PI)
      assert(Visited.count(BB) && "Dependent not satisfied!");
  }
}

bool TopSortBB::runOnFunction(Function &F) {
  RegionInfo &RI = getAnalysis<RegionInfo>();
  FN = &F;

  sortBBInRegion(RI.getTopLevelRegion());
  verifyAnalysis();

  return false;
}

void TopSortBB::sortBBInRegion(Region *R) {
  for (Region::element_iterator I = R->element_begin(), E = R->element_end();
      I != E; ++I)
    if (I->isSubRegion())
      sortBBInRegion(I->getNodeAs<Region>());
    else {
      BasicBlock *BB = I->getNodeAs<BasicBlock>();
      Function *F = BB->getParent();
      BB->removeFromParent();
      F->getBasicBlockList().push_back(BB);
    }
}

char TopSortBB::ID = 0;

Pass *esyn::createTopSortBBPass() {
  return new TopSortBB();
}
