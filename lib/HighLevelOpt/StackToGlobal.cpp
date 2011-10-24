// StackToGlobal.cpp ---   convert AllocaInst to GlobalVariable ---- C++ -==//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This pass convert AllocaInst to GlobalVariable
//
//===----------------------------------------------------------------------===//
#include "vtm/Passes.h"
#include "llvm/Pass.h"
#include "llvm/Module.h"
#include "llvm/Instructions.h"
#include "llvm/Support/Casting.h"

#define DEBUG_TYPE "StackToGlobal"

#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/GlobalVariable.h"

#include <vector>
using namespace llvm;

namespace {
  struct StackToGlobal : public ModulePass {
    static char ID;

    StackToGlobal(): ModulePass(ID) {
      initializeStackToGlobalPass(*PassRegistry::getPassRegistry());
    }

    void getAnalysisUsage(AnalysisUsage &AU) const {
      ModulePass::getAnalysisUsage(AU);
    }

    bool runOnModule(Module &M);
  };
} // end anonymous.

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
    const Type *Ty = AI->getAllocatedType();
    GlobalVariable *GV =
    new GlobalVariable(M, Ty, false, GlobalValue::InternalLinkage,
					       Constant::getNullValue(Ty), AI->getName() + "_s2g");
    AI->replaceAllUsesWith(GV);
    AI->eraseFromParent();
  }
  return true;
}


char StackToGlobal::ID = 0;
INITIALIZE_PASS_BEGIN(StackToGlobal, "StackToGlobal",
                        "StackToGlobal", false, false)
INITIALIZE_PASS_END(StackToGlobal, "StackToGlobal",
                    "StackToGlobal", false, false)

Pass *llvm::createStackToGlobalPass() {
  return new StackToGlobal();
}
