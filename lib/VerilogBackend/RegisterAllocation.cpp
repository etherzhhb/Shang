//===-------- RegisterAllocation.cpp - Allocation registers -----*- C++ -*-===//
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
// This file implement a pass that reduce unnecessary registers
//
//===----------------------------------------------------------------------===//

#include "HWAtomPasses.h"
#include "HWAtomInfo.h"

#define DEBUG_TYPE "vbe-reg-alloca"
#include "llvm/Support/Debug.h"

using namespace llvm;
using namespace esyn;

namespace {
struct RegAllocation : public FunctionPass {
  static char ID;
  explicit RegAllocation() : FunctionPass(&ID) {}
  bool runOnFunction(Function &F);
  bool runOnBasicBlock(BasicBlock &BB, HWAtomInfo &HI);
  void getAnalysisUsage(AnalysisUsage &AU) const;
};
}

bool RegAllocation::runOnFunction(Function &F) {
  HWAtomInfo &HI = getAnalysis<HWAtomInfo>();
  // Emit phi nodes
  for (Function::iterator I = F.begin(), E = F.end(); I != E; ++I) {
    BasicBlock &BB = *I;
    BasicBlock::iterator It = BB.begin();
    while (isa<PHINode>(It)) {
      PHINode *PN = cast<PHINode>(It++);
      // Emit the PHINode.
      HI.getConstant(*PN, &BB);
      // And its operand. 
      for (Instruction::op_iterator OI = PN->op_begin(), OE = PN->op_end();
        OI != OE; ++OI) {
          Value *V = *OI;
         
          // Emit the constants.
          if (isa<Constant>(V) || isa<Argument>(V))
            (void) HI.getConstant(*V, &BB);
          else if (isa<Instruction>(V)) {
            HWAtom *A = HI.getAtomFor(*V);
            HWARegister *R = HI.getRegister(*V, A);
            HI.updateAtomMap(*V, R);
          }
      }
    } 

    HWAPostBind &Exit = HI.getStateFor(BB).getExitRoot();
    for (unsigned i = Exit.getInstNumOps(), e = Exit.getNumDeps(); i != e; ++i) {
      HWAtom *Dep = Exit.getDep(i);
      const Type *Ty = Dep->getValue().getType();
      // Emit export register.
      if (!Ty->isVoidTy() && !Ty->isLabelTy()) {
        HWARegister *R = HI.getRegister(Dep->getValue(), Dep);
        HI.updateAtomMap(Dep->getValue(), R);
        Exit.setDep(i, R);
      }
    }
  }

  for (Function::iterator I = F.begin(), E = F.end(); I != E; ++I)
    runOnBasicBlock(*I, HI);

  return false;
}

bool RegAllocation::runOnBasicBlock(BasicBlock &BB, HWAtomInfo &HI) {
  ExecStage &State = HI.getStateFor(BB);
  HWAVRoot *EntryRoot = &State.getEntryRoot();

  SmallVector<HWAtom*, 32> Worklist(usetree_iterator::begin(EntryRoot),
                                    usetree_iterator::end(EntryRoot));

  while(!Worklist.empty()) {
    HWAOpInst *A = dyn_cast<HWAOpInst>(Worklist.back());
    Worklist.pop_back();
    
    if (A == 0)
      continue;

    DEBUG(A->print(dbgs()));
    DEBUG(dbgs() << " Visited\n");

    for (unsigned i = 0, e = A->getInstNumOps(); i != e; ++i) {
      HWAtom *dep = A->getDep(i);
      Value &V = *A->getIOperand(i);
      if (isa<HWAVRoot>(dep)) {
        if (isa<Constant>(V) || isa<Argument>(V))
          A->setDep(i, HI.getConstant(V, &BB));
        else
          A->setDep(i, HI.getAtomFor(V));

      } else if (HWAOpInst *DI = dyn_cast<HWAOpInst>(dep)) {
        if (DI->getSlot() + DI->getLatency() != A->getSlot() ||
            isa<HWAPreBind>(A)) {
          DEBUG(DI->print(dbgs()));
          DEBUG(dbgs() << " Registered\n");

          A->setDep(i, HI.getRegister(DI->getValue(), DI));
        }
      }

      if (((A->getOpcode() == Instruction::AShr) && (i == 0))
          || ((A->getOpcode() == Instruction::ICmp)
              && (A->getInst<ICmpInst>().isSigned())))      
        A->setDep(i, HI.getSigned(A->getDep(i)));
    }
  }

  //State.print(dbgs());
  
  return false;
}

void RegAllocation::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<HWAtomInfo>();
  AU.setPreservesAll();
}

char RegAllocation::ID = 0;

Pass *esyn::createRegisterAllocationPass() {
  return new RegAllocation();
}
