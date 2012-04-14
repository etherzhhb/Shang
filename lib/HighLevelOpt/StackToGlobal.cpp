// StackToGlobal.cpp ---   convert AllocaInst to GlobalVariable ---- C++ -==//
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
// This pass convert AllocaInst to GlobalVariable
//
//===----------------------------------------------------------------------===//
#include "../../VIntrinsicsInfo.h"

#include "vtm/Passes.h"
#include "vtm/Utilities.h"
#include "vtm/VerilgoBackendMCTargetDesc.h"
#include "llvm/GlobalVariable.h"
#include "llvm/Pass.h"
#include "llvm/Module.h"
#include "llvm/Instructions.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/Support/Casting.h"
#define DEBUG_TYPE "StackToGlobal"
#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/ADT/Statistic.h"

#include <vector>
using namespace llvm;

STATISTIC(NumGlobalAlias, "Number of global alias created for allocas");

namespace {
  struct StackToGlobal : public ModulePass {
    static char ID;
    const TargetIntrinsicInfo &IntrInfo;
    unsigned AllocaCnt;

    StackToGlobal(const TargetIntrinsicInfo &I) : ModulePass(ID), IntrInfo(I),
      AllocaCnt(0) {
      initializeStackToGlobalPass(*PassRegistry::getPassRegistry());
    }

    StackToGlobal() : ModulePass(ID), IntrInfo(*new VIntrinsicInfo()) {
      llvm_unreachable("Cannot construct StackToGlobal like this!");
    };

    bool runOnModule(Module &M);

    bool handleAlloca(AllocaInst *AI, Function *F, Module &M);
  };
} // end anonymous.

bool StackToGlobal::handleAlloca(AllocaInst *AI, Function *F, Module &M) {
  PointerType *Ty = AI->getType();
  Type *AllocatedType = AI->getAllocatedType();
  // Create the global alias.
  GlobalVariable *GV = new GlobalVariable(M, AllocatedType, false,
                                          GlobalValue::InternalLinkage,
                                          Constant::getNullValue(AllocatedType),
                                          AI->getName() + utostr_32(++AllocaCnt)
                                          + "_g_alias");
  GV->setAlignment(AI->getAlignment());

  BasicBlock::iterator IP = llvm::next(BasicBlock::iterator(AI));

  // Create the function call to annotate the alias.
  Value *Args[] = { AI, GV };
  // We may need a cast.
  if (!Ty->getElementType()->isPrimitiveType()) {
    PointerType *PtrTy = PointerType::getIntNPtrTy(M.getContext(), 8,
                                                   Ty->getAddressSpace());
    Args[0] = CastInst::CreatePointerCast(AI, PtrTy, AI->getName()+"_cast", IP);
    Args[1] = ConstantExpr::getBitCast(GV, PtrTy);
  }

  Type *ArgTypes[] = { Args[0]->getType(), Args[1]->getType() };
  Function *AllocaAliasGlobal =
    IntrInfo.getDeclaration(&M, vtmIntrinsic::vtm_alloca_alias_global,
                            ArgTypes, 2);
  CallInst::Create(AllocaAliasGlobal, Args, "", IP);
  ++NumGlobalAlias;
  return true;
}

bool StackToGlobal::runOnModule(Module &M) {
  std::vector<AllocaInst*> WorkList;
  for (Module::iterator FI = M.begin(), E = M.end(); FI != E; ++FI){
    Function *F = FI;
    for(Function::iterator BBI = F->begin(), E = F->end(); BBI != E; ++BBI){
      BasicBlock *BB = BBI;
      for (BasicBlock::iterator II = BB->begin(), E = BB->end(); II != E; ++II){
        if (AllocaInst *AI = dyn_cast<AllocaInst>(&*II)) {
            //push back the AllocaInst to a vector.
              WorkList.push_back(AI);
        }
      }
    }
  }

//iterate the vector to replace the AllocaInst with the GlobalVariable.
  for(std::vector<AllocaInst*>::iterator I = WorkList.begin(), E = WorkList.end();
      I != E; ++I){
    AllocaInst* AI= *I;
    Type *Ty = AI->getAllocatedType();
    GlobalVariable *GV =
    new GlobalVariable(M, Ty, false, GlobalValue::InternalLinkage,
					             Constant::getNullValue(Ty),
                       AI->getName() + "_s2g");
    GV->setAlignment(std::max(8u, AI->getAlignment()));
    AI->replaceAllUsesWith(GV);
    AI->eraseFromParent();
  }
  return true;
}


char StackToGlobal::ID = 0;
INITIALIZE_PASS(StackToGlobal, "StackToGlobal",  "StackToGlobal", false, false)

Pass *llvm::createStackToGlobalPass(const TargetIntrinsicInfo &IntrInfo) {
  return new StackToGlobal(IntrInfo);
}
