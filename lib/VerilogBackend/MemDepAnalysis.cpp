//===- MemDepAnalyzsis.cpp - Preform Memory dependencies analyze  -*- C++ -*-===//
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
// This file implement the MemDepAnalyzsis pass, which provide the memory
// dependencies information between two memory operation.
//
//===----------------------------------------------------------------------===//

#include "HWAtomInfo.h"
#include "MemDepAnalysis.h"

#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/Analysis/ScalarEvolutionExpressions.h"
#include "llvm/Analysis/LoopInfo.h"
#define DEBUG_TYPE "vbe-mem-dep-info"
#include "llvm/Support/Debug.h"

using namespace llvm;
using namespace esyn;

//===----------------------------------------------------------------------===//
MemDepInfo::DepInfo::DepInfo(HWMemDep::MemDepTypes dep, unsigned itDst)
: Dep(dep), ItDst(itDst) {
  DEBUG(
    dbgs() << "Create ";
    switch (Dep) {
    case HWMemDep::AntiDep:
      dbgs() << "Anti ";
      break;
    case HWMemDep::OutputDep:
      dbgs() << "Output ";
      break;
    case HWMemDep::TrueDep:
      dbgs() << "True ";
      break;
    }
    dbgs() << " dep, with distance: " << itDst << '\n';
  );
}

MemDepInfo::DepInfo::DepInfo() : Dep(HWMemDep::NoDep), ItDst(0) { 
}
//===----------------------------------------------------------------------===//
char MemDepInfo::ID = 0;
RegisterPass<MemDepInfo> X("vbe-memdep",
                           "vbe - Memory dependencies analyzsis");

MemDepInfo::DepInfo
MemDepInfo::getDepInfo(Instruction &Src, Instruction &Dst, BasicBlock &BB,
                       bool SrcBeforeDest) {
  assert(&Src != &Dst && "No self deps!");
  DEBUG(dbgs() << "Compute deps between:\n"
               << Src << "\n\tand \n" << Dst << "\n:");

  if (LoadInst *L = dyn_cast<LoadInst>(&Src)) {
    if (StoreInst *S = dyn_cast<StoreInst>(&Dst)) {
      return AnalyzeDeps(L->getPointerOperand(), S->getPointerOperand(),
        true, false, BB, SrcBeforeDest);
    }
    // Ignore RAR dependence.
    return DepInfo();
  }

  StoreInst *S = cast<StoreInst>(&Src);
  if (LoadInst *L = dyn_cast<LoadInst>(&Dst)) {
    return AnalyzeDeps(S->getPointerOperand(), L->getPointerOperand(),
      false, true, BB, SrcBeforeDest);
  }
  StoreInst *SS = cast<StoreInst>(&Dst);

  return AnalyzeDeps(S->getPointerOperand(), SS->getPointerOperand(),
    false, false, BB, SrcBeforeDest);  
}

MemDepInfo::DepInfo
MemDepInfo::AnalyzeDeps(Value *SrcAddr, Value *DstAddr, bool SrcLoad, bool DstLoad,
                        BasicBlock &BB, bool SrcBeforeDest) {
  
  Loop *L = LI->getLoopFor(&BB);
  
  unsigned SrcSize = ~0u;
  const Type *SrcElTy = cast<PointerType>(SrcAddr->getType())->getElementType();
  if (SrcElTy->isSized()) SrcSize = AA->getTypeStoreSize(SrcElTy);

  unsigned DstSize = ~0u;
  const Type *DstElTy = cast<PointerType>(DstAddr->getType())->getElementType();
  if (DstElTy->isSized()) DstSize = AA->getTypeStoreSize(DstElTy);

  if (L && L->isLoopInvariant(SrcAddr) && L->isLoopInvariant(DstAddr)) {
    // FIXME: What about nested loops?
    // Loop Invariant, let AA decide.
    if (!AA->isNoAlias(SrcAddr, SrcSize, DstAddr, DstSize))
      return createDep(SrcLoad, DstLoad, SrcBeforeDest);
    else
      return DepInfo();
  }
  
  GetElementPtrInst *SrcGEP = dyn_cast<GetElementPtrInst>(SrcAddr),
                    *DstGEP = dyn_cast<GetElementPtrInst>(DstAddr);

  if (!(SrcGEP && DstGEP))
    return createDep(SrcLoad, DstLoad, SrcBeforeDest);
  
  Value *SGPtr = SrcGEP->getPointerOperand(),
        *DGPtr = DstGEP->getPointerOperand();

  switch(AA->alias(SGPtr, SrcSize, DGPtr, DstSize)) {
  case AliasAnalysis::MustAlias:
    // We can only handle two access have the same element size.
    if (SrcSize == DstSize)
      return advancedDepAnalysis(SrcGEP, DstGEP, SrcLoad, DstLoad,
                                BB, SrcBeforeDest, SrcSize);
    // FIXME: Handle gep with difference size.
    // Fall thoungh.
  case AliasAnalysis::MayAlias:
    return createDep(SrcLoad, DstLoad, SrcBeforeDest);
  default:
    return DepInfo();
  }
  
  llvm_unreachable("What alias result we got?");
  return DepInfo();
}

MemDepInfo::DepInfo
MemDepInfo::advancedDepAnalysis(GetElementPtrInst *SrcAddr,
                                GetElementPtrInst *DstAddr,
                                bool SrcLoad, bool DstLoad,
                                BasicBlock &BB, bool SrcBeforeDest,
                                unsigned ElSizeInByte) {
  const SCEV *SSAddr = SE->getSCEVAtScope(SrcAddr, LI->getLoopFor(&BB)),
             *SDAddr = SE->getSCEVAtScope(DstAddr, LI->getLoopFor(&BB));
  DEBUG(dbgs() << *SSAddr << " and " << *SDAddr << '\n');
  // Use SCEV to compute the dependencies distance.
  const SCEV *Distance = SE->getMinusSCEV(SSAddr, SDAddr);
  // TODO: Get range.
  if (const SCEVConstant *C = dyn_cast<SCEVConstant>(Distance)) {
    int ItDistance = C->getValue()->getSExtValue();
    if (ItDistance >= 0)
      // The pointer distance is in Byte, but we need to get the distance in
      // Iteration.
      return createDep(SrcLoad, DstLoad, SrcBeforeDest,
                      ItDistance / ElSizeInByte);
    else
      return DepInfo();
  }

  return createDep(SrcLoad, DstLoad, SrcBeforeDest);
}

MemDepInfo::DepInfo MemDepInfo::createDep(bool SrcLoad, bool DstLoad,
                                          bool SrcBeforeDest, int Diff) {
   if (!SrcBeforeDest && (Diff == 0))
     Diff = 1;
   
   assert(Diff >= 0 && "Do not create a dependence with diff small than 0!");
   assert(!(SrcLoad && DstLoad) && "Do not create a RAR dep!");

   // WAW
   if (!SrcLoad && !DstLoad )
     return DepInfo(HWMemDep::OutputDep, Diff);

   if (!SrcLoad && DstLoad)
     SrcBeforeDest = !SrcBeforeDest;

   return DepInfo(SrcBeforeDest ? HWMemDep::AntiDep : HWMemDep::TrueDep, Diff);
}

bool MemDepInfo::runOnFunction(Function &F) {
  AA = &getAnalysis<AliasAnalysis>();
  TD = &getAnalysis<TargetData>();
  SE = &getAnalysis<ScalarEvolution>();
  LI = &getAnalysis<LoopInfo>();
  return  false;
}

void esyn::MemDepInfo::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<AliasAnalysis>();
  AU.addRequired<TargetData>();
  AU.addRequired<ScalarEvolution>();
  AU.addRequired<LoopInfo>();
  AU.setPreservesAll();
}
