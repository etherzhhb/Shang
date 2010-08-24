//===----------- InstLowering.cpp - Lower instructions for VBE ----*- C++ -*-===//
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
// This file implement the instruction lowering pass.
//
//===----------------------------------------------------------------------===//

#include "HWAtomPasses.h"

#include "llvm/Pass.h"
#include "llvm/Instructions.h"
#include "llvm/BasicBlock.h"
#include "llvm/Support/IRBuilder.h"
#include "llvm/Target/TargetData.h"
#include "llvm/Support/GetElementPtrTypeIterator.h"
#include "llvm/Support/MathExtras.h"

#define DEBUG_TYPE "vbe-inst-lower"
#include "llvm/Support/Debug.h"

using namespace llvm;

namespace {
struct InstLowering : public BasicBlockPass {
  TargetData *TD;

  static char ID;
  InstLowering() : BasicBlockPass(&ID) {}
  bool runOnBasicBlock(BasicBlock &BB);

  void lowerGEP(GetElementPtrInst *GEP);
};
}

bool InstLowering::runOnBasicBlock(BasicBlock &BB) {
  TD = getAnalysisIfAvailable<TargetData>();
  assert(TD && "TD not availabe!");

  std::vector<Instruction*> WorkList;
  for (BasicBlock::iterator I = BB.begin(), E = BB.end(); I != E; ++I)
    WorkList.push_back(I);
  
  for (std::vector<Instruction*>::iterator I = WorkList.begin(),
      E = WorkList.end(); I != E; ++I) {
    Instruction *Inst = *I;
    if (isa<GetElementPtrInst>(Inst))
      lowerGEP(cast<GetElementPtrInst>(Inst));
  }

  DEBUG(BB.dump());
  return true;
}

void InstLowering::lowerGEP(GetElementPtrInst *GEP) {
  //// No need to lower.
  //if (GEP->getNumIndices() < 2)
  //  return;
  
  Value *BaseAddr = GEP->getOperand(0);
  const Type *BaseTy = BaseAddr->getType();
  const Type *ElemTy = dyn_cast<SequentialType>(BaseTy)->getElementType();
  uint64_t ElemTyAllocSize = TD->getTypeAllocSize(ElemTy);

  uint64_t TotalConstOffs = 0;
  Value *LastIndex = 0;

  IRBuilder<> Builder(GEP->getParent(), GEP);

  for (GetElementPtrInst::op_iterator OI = GEP->op_begin() + 1,
       OE = GEP->op_end(); OI != OE; ++OI) {
    Value *Idx = *OI;
    BaseTy = dyn_cast<SequentialType>(BaseTy)->getElementType();
    // We can not handle.
    assert(BaseTy && "VBE can not handle now!");

    // If this is a constant subscript, handle it quickly.
    if (ConstantInt *CI = dyn_cast<ConstantInt>(Idx)) {
      if (CI->getZExtValue() == 0) continue;
      uint64_t Offs =
        TD->getTypeAllocSize(BaseTy) * cast<ConstantInt>(CI)->getSExtValue();
      TotalConstOffs += Offs; 
      continue;
    }

    // The offset.
    uint64_t TypeAllocSize = TD->getTypeAllocSize(BaseTy);
    // Dirty Hack:
    // Our address is base on Element size.
    TypeAllocSize /= ElemTyAllocSize;

    if (TypeAllocSize != 1) {
      if (isPowerOf2_64(TypeAllocSize)) {
        Idx = Builder.CreateShl(Idx, Log2_64(TypeAllocSize));
      } else {
        ConstantInt *IdxSize =
          ConstantInt::get(TD->getIntPtrType(BaseTy->getContext()),
                           TypeAllocSize);
        Idx = Builder.CreateMul(Idx, IdxSize);
      }
    }

    if (LastIndex == 0)
      LastIndex = Idx;
    else
      LastIndex = Builder.CreateAdd(LastIndex, Idx);
  }
  // Add the constant to the offset
  if (TotalConstOffs) {
    ConstantInt *ConstOffs
      = ConstantInt::get(TD->getIntPtrType(BaseTy->getContext()),
                         TotalConstOffs);
    LastIndex = Builder.CreateAdd(LastIndex, ConstOffs);
  }

  // Build Base addr to match the type.
  SmallVector<Value*, 4> ZeroIdx;
  ConstantInt *Zero =
    ConstantInt::get(TD->getIntPtrType(BaseTy->getContext()), 0);
  ZeroIdx.append(GEP->getNumIndices(), Zero);
  BaseAddr = Builder.CreateGEP(BaseAddr, ZeroIdx.begin(), ZeroIdx.end());

  // Get element pointer form base.
  Value *NewGEP = Builder.CreateGEP(BaseAddr, LastIndex);
  GEP->replaceAllUsesWith(NewGEP);
  GEP->eraseFromParent();
}

char InstLowering::ID = 0;

Pass *esyn::createInstLoweringPass() {
  return new InstLowering();
}
