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
#include "llvm/Support/InstIterator.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/CommandLine.h"
#define DEBUG_TYPE "vtm-frame-lowering"
#include "llvm/Support/Debug.h"

using namespace llvm;
static cl::opt<bool> EnableBRAM("vtm-enable-bram",
                                cl::desc("Enable block RAM in design"),
                                cl::init(false));

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
struct BlockRAMFormation : public FunctionPass {
  static char ID;
  const TargetIntrinsicInfo &IntrInfo;
  unsigned CurAddrSpace;
  enum { FirstBlockRAMAddressSpace = 1 };
  BlockRAMFormation(const TargetIntrinsicInfo &I)
    : FunctionPass(ID), IntrInfo(I), CurAddrSpace(FirstBlockRAMAddressSpace) {}

  BlockRAMFormation()
    : FunctionPass(ID), IntrInfo(*new VIntrinsicInfo()), CurAddrSpace(0) {
    llvm_unreachable("Cannot construct BlockRAMFormation like this!");
  }

  const char *getPassName() const { return "Block RAM Formation Pass"; }
  // Try to localize the global variables.
  bool doInitialization(Module &M) { return false; }
  bool runOnFunction(Function &F);
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

    // Can us handle the use tree of the allocated pointer?
    if (!EnableBRAM || !visitPtrUseTree(AI, Collector)) continue;

    // Change the address space of the alloca, so the backend know the
    // load/store accessing this alloca are accessing block ram.
    mutateAddressSpace(AI, CurAddrSpace);
    while (!Collector.Uses.empty())
      mutateAddressSpace(Collector.Uses.pop_back_val(), CurAddrSpace);

    changed |= true;
    ++CurAddrSpace;
  }

  return changed;
}

Pass *llvm::createBlockRAMFormation(const TargetIntrinsicInfo &IntrInfo) {
  return new BlockRAMFormation(IntrInfo);
}
