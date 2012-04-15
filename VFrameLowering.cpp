//=======- VFrameInfo.cpp - VTM Frame Information -----------*- C++ -*-=====//
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
// This file contains the VTM implementation of TargetFrameInfo class.
//
//===----------------------------------------------------------------------===//

#include "VIntrinsicsInfo.h"
#include "VFrameLowering.h"
#include "llvm/CodeGen/MachineFunction.h"

#include "vtm/Passes.h"

#include "llvm/Pass.h"
#include "llvm/Module.h"
#include "llvm/Instructions.h"
#include "llvm/Analysis/ValueTracking.h"
#include "llvm/Target/TargetData.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/SmallPtrSet.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/Support/InstIterator.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/CommandLine.h"
#define DEBUG_TYPE "vtm-frame-lowering"
#include "llvm/Support/Debug.h"
#include "llvm/ADT/Statistic.h"

using namespace llvm;
static cl::opt<bool> EnableBRAM("vtm-enable-bram",
                                cl::desc("Enable block RAM in design"),
                                cl::init(false));

STATISTIC(NumGlobalAlias, "Number of global alias created for allocas");
STATISTIC(NumBlockRAMs, "Number of block RAM created");

void VFrameInfo::emitPrologue(MachineFunction &MF) const {
}

void VFrameInfo::emitEpilogue(MachineFunction &MF,
                              MachineBasicBlock &MBB) const {
}


//===----------------------------------------------------------------------===//
// Utility function to change the address space of the
//
template <typename VisitFunc>
static bool visitPtrUseTree(Value *BasePtr, VisitFunc &Visitor) {
  typedef Instruction::use_iterator ChildIt;
  typedef SmallVector<std::pair<Value*, ChildIt>, 16> StackTy;
  SmallPtrSet<Value*, 8> Visited;

  StackTy Stack;
  Stack.push_back(std::make_pair(BasePtr, BasePtr->use_begin()));

  while (!Stack.empty()) {
    Value *CurVal = Stack.back().first;
    ChildIt &CurChildIt = Stack.back().second;

    // All children of the current instruction visited, visit the current
    // instruction.
    if (CurChildIt == CurVal->use_end()) {
      Stack.pop_back();
      continue;
    }

    Value *Child = *CurChildIt;
    ++CurChildIt;
    // Had us visited this node yet?
    if (!Visited.insert(Child)) continue;

    if (!Visitor(Child, CurVal)) return false;

    // Don't trace the loaded value.
    if (isa<LoadInst>(Child)) continue;

    Stack.push_back(std::make_pair(Child, Child->use_begin()));
  }

  return true;
}

namespace {
struct BlockRAMFormation : public ModulePass {
  static char ID;
  const TargetIntrinsicInfo &IntrInfo;
  unsigned CurAddrSpace, AllocaAliasCnt;
  enum { FirstBlockRAMAddressSpace = 1 };
  BlockRAMFormation(const TargetIntrinsicInfo &I)
    : ModulePass(ID), IntrInfo(I), CurAddrSpace(FirstBlockRAMAddressSpace),
      AllocaAliasCnt(0) {}

  BlockRAMFormation()
    : ModulePass(ID), IntrInfo(*new VIntrinsicInfo()), CurAddrSpace(0),
      AllocaAliasCnt(0) {
    llvm_unreachable("Cannot construct BlockRAMFormation like this!");
  }

  const char *getPassName() const { return "Block RAM Formation Pass"; }
  bool runOnModule(Module &M);
  bool runOnFunction(Function &F);

  void allocateGlobalAlias(AllocaInst *AI, Function *F);
};

struct PtrUseCollector {
  SmallVector<Value *, 8> Uses;

  bool operator()(Value *ValUser, const Value *V) {
    if (const Instruction *I = dyn_cast<Instruction>(ValUser)) {
      switch (I->getOpcode()) {
      case Instruction::GetElementPtr:
        Uses.push_back(ValUser);
        return true;
        // The pointer must use as pointer operand in load/store.
      case Instruction::Load: return cast<LoadInst>(I)->getPointerOperand() == V;
      case Instruction::Store:return cast<StoreInst>(I)->getPointerOperand() == V;
      }
    } else if (const ConstantExpr *C = dyn_cast<ConstantExpr>(ValUser)) {
      if (C->getOpcode() == Instruction::GetElementPtr) {
        Uses.push_back(ValUser);
        return true;
      }
    }

    return false;
  }
};
}

char BlockRAMFormation::ID = 0;

void BlockRAMFormation::allocateGlobalAlias(AllocaInst *AI, Function *F) {
  Module &M = *F->getParent();
  PointerType *Ty = AI->getType();
  Type *AllocatedType = AI->getAllocatedType();
  // Create the global alias.
  GlobalVariable *GV =
    new GlobalVariable(M, AllocatedType, false, GlobalValue::InternalLinkage,
                       Constant::getNullValue(AllocatedType),
                       AI->getName() + utostr_32(AllocaAliasCnt) + "_g_alias");
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
  ++AllocaAliasCnt;
}

static void mutateAddressSpace(Value *V, unsigned AS) {
  PointerType *Ty  = cast<PointerType>(V->getType());
  assert(Ty->getAddressSpace() == 0 && "V already in some address space!");
  Ty = PointerType::get(Ty->getElementType(), AS);
  V->mutateType(Ty);
}

bool BlockRAMFormation::runOnFunction(Function &F) {
  bool changed = false;
  PtrUseCollector Collector;
  for (inst_iterator I = inst_begin(F), E = inst_end(F); I != E; ++I) {
    AllocaInst *AI = dyn_cast<AllocaInst>(&*I);
    if (!AI) continue;

    changed |= true;
    // Can us handle the use tree of the allocated pointer?
    if (!EnableBRAM || !visitPtrUseTree(AI, Collector)) {
      // Otherwise, we need to allocate the object in global memory.
      allocateGlobalAlias(AI, &F);
      continue;
    }

    // Change the address space of the alloca, so the backend know the
    // load/store accessing this alloca are accessing block ram.
    mutateAddressSpace(AI, CurAddrSpace);
    while (!Collector.Uses.empty())
      mutateAddressSpace(Collector.Uses.pop_back_val(), CurAddrSpace);

    ++NumBlockRAMs;
    ++CurAddrSpace;
  }

  return changed;
}

bool BlockRAMFormation::runOnModule(Module &M) {
  bool changed = false;

  for (Module::iterator I = M.begin(), E = M.end(); I != E; ++I)
    changed |= runOnFunction(*I);

  return changed;
}

Pass *llvm::createBlockRAMFormation(const TargetIntrinsicInfo &IntrInfo) {
  return new BlockRAMFormation(IntrInfo);
}
