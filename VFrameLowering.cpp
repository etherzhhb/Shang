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
#include "llvm/ADT/PointerIntPair.h"
#include "llvm/ADT/DenseMap.h"
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
                                cl::init(true));

STATISTIC(NumGlobalAlias, "Number of global alias created for allocas");
STATISTIC(NumBlockRAMs, "Number of block RAM created");
STATISTIC(NumLocalizedGV, "Number of GlobalVariable localized");


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
  TargetData *TD;
  unsigned CurAddrSpace, AllocaAliasCnt;
  enum { FirstBlockRAMAddressSpace = 1 };
  BlockRAMFormation(const TargetIntrinsicInfo &I)
    : ModulePass(ID), IntrInfo(I),TD(0),CurAddrSpace(FirstBlockRAMAddressSpace),
      AllocaAliasCnt(0) {}

  BlockRAMFormation()
    : ModulePass(ID), IntrInfo(*new VIntrinsicInfo()), TD(0), CurAddrSpace(0),
      AllocaAliasCnt(0) {
    llvm_unreachable("Cannot construct BlockRAMFormation like this!");
  }

  const char *getPassName() const { return "Block RAM Formation Pass"; }

  void getAnalysisUsage(AnalysisUsage &U) const {
    U.addRequired<TargetData>();
  }

  bool runOnModule(Module &M);
  bool runOnFunction(Function &F, Module &M);
  bool localizeGV(GlobalVariable *GV, Module &M);

  void allocateGlobalAlias(AllocaInst *AI, Module &M, unsigned AddressSpace = 0);
  void annotateBRAMInfo(unsigned BRAMNum, Type *AllocatedType, Constant *InitGV,
                        Instruction *InsertPos, Module &M);
};

struct AllocateUseCollector {
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

struct GVUseCollector {
  typedef SmallVector<Value*, 8> UsesVec;
  DenseMap<Function*, UsesVec> Uses;
  typedef DenseMap<Function*, UsesVec>::iterator iterator;
  // Modify information, Is the GV written in a function?
  DenseMap<const Function*, bool> ModInfo;

  bool operator()(Value *ValUser, const Value *V)  {
    if (Instruction *I = dyn_cast<Instruction>(ValUser)) {
      switch (I->getOpcode()) {
      case Instruction::GetElementPtr:
        Uses[I->getParent()->getParent()].push_back(ValUser);
        return true;
        // The pointer must use as pointer operand in load/store.
      case Instruction::Load:
        // Place holder, means there is a user in current function.
        Uses[I->getParent()->getParent()].push_back(0);
        return cast<LoadInst>(I)->getPointerOperand() == V;
      case Instruction::Store:
        // Place holder, means there is a user in current function.
        Uses[I->getParent()->getParent()].push_back(0);
        ModInfo[I->getParent()->getParent()] = true;
        return cast<StoreInst>(I)->getPointerOperand() == V;
      }
    } else if (const ConstantExpr *C = dyn_cast<ConstantExpr>(ValUser)) {
      if (C->getOpcode() == Instruction::GetElementPtr) {
        Uses[0].push_back(ValUser);
        return true;
      }
    }
    return false;
  }

  bool canBeLocalized() const {
    typedef DenseMap<Function*, UsesVec>::const_iterator it;
    unsigned NumReferredFunctions = 0;
    bool Written = false;

    for (it I = Uses.begin(), E = Uses.end(); I != E; ++I) {
      // Used by constant expression?
      if (I->first == 0) continue;

      ++NumReferredFunctions;
      Written |= ModInfo.lookup(I->first);
    }

    // We can localize the GV if it only accessed in one function, or it is not
    // written.
    return NumReferredFunctions == 1 || !Written;
  }
};
}

char BlockRAMFormation::ID = 0;

void BlockRAMFormation::allocateGlobalAlias(AllocaInst *AI, Module &M,
                                            unsigned AddressSpace) {
  PointerType *Ty = AI->getType();
  Type *AllocatedType = AI->getAllocatedType();
  // Create the global alias.
  GlobalVariable *GV =
    new GlobalVariable(M, AllocatedType, false, GlobalValue::InternalLinkage,
                       Constant::getNullValue(AllocatedType),
                       AI->getName() + utostr_32(AllocaAliasCnt) + "_g_alias",
                       0, false, AddressSpace);
  GV->setAlignment(AI->getAlignment());

  BasicBlock::iterator IP = AI->getParent()->getTerminator();

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
                            ArgTypes, array_lengthof(ArgTypes));
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

void BlockRAMFormation::annotateBRAMInfo(unsigned BRAMNum, Type *AllocatedType,
                                         Constant *InitGV, Instruction *InsertPos,
                                         Module &M) {
  // The element type of a scalar is the type of the scalar.
  Type *ElemTy = AllocatedType;
  unsigned NumElem = 1;
  // Try to expand multi-dimension array to single dimension array.
  while (const ArrayType *AT = dyn_cast<ArrayType>(ElemTy)) {
    ElemTy = AT->getElementType();
    NumElem *= AT->getNumElements();
  }

  if (!cast<PointerType>(InitGV->getType())->getElementType()->isPrimitiveType()) {
    Type *PtrType = PointerType::getIntNPtrTy(M.getContext(), 8, 0);
    InitGV = ConstantExpr::getBitCast(InitGV, PtrType);
  }

  Type *Int32Ty = IntegerType::get(M.getContext(), 32);
  Value *Args[] = { ConstantInt::get(Int32Ty, BRAMNum),
                    ConstantInt::get(Int32Ty, NumElem),
                    ConstantInt::get(Int32Ty, TD->getTypeStoreSize(ElemTy)),
                    InitGV };

  Type *ArgTypes[] = { Args[3]->getType() };
  Function *Annotate =
    IntrInfo.getDeclaration(&M, vtmIntrinsic::vtm_annotated_bram_info,
                            ArgTypes, array_lengthof(ArgTypes));
  CallInst::Create(Annotate, Args, "", InsertPos);
}

bool BlockRAMFormation::runOnFunction(Function &F, Module &M) {
  bool changed = false;
  AllocateUseCollector Collector;
  for (inst_iterator I = inst_begin(F), E = inst_end(F); I != E; ++I) {
    AllocaInst *AI = dyn_cast<AllocaInst>(&*I);
    if (!AI) continue;

    changed |= true;
    // Can us handle the use tree of the allocated pointer?
    if (!EnableBRAM || !visitPtrUseTree(AI, Collector)) {
      Collector.Uses.clear();
      // Otherwise, we need to allocate the object in global memory.
      allocateGlobalAlias(AI, M);
      continue;
    }

    allocateGlobalAlias(AI, M, CurAddrSpace);
    Constant *NullInitilizer =
      Constant::getNullValue(PointerType::getIntNPtrTy(F.getContext(), 8, 0));
    annotateBRAMInfo(CurAddrSpace, AI->getAllocatedType(),
                     NullInitilizer, AI->getParent()->getTerminator(),
                     *F.getParent());
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

bool BlockRAMFormation::localizeGV(GlobalVariable *GV, Module &M) {
  // Cannot localize if the GV may be modified by others module.
  if (!GV->hasInternalLinkage() && !GV->hasPrivateLinkage() &&
      (!GV->isConstant() || GV->isDeclaration()))
    return false;

  assert(GV->hasInitializer() && "Unexpected declaration!");
  // Not support BRAM initializing at the moment.
  if (!GV->getInitializer()->isNullValue())
    return false;

  GVUseCollector Collector;

  if (!visitPtrUseTree(GV, Collector)) return false;

  if (!Collector.canBeLocalized()) return false;

  // Align the address of localized GV.
  GV->setAlignment(std::max(8u, GV->getAlignment()));
  // Change the address space of the alloca, so the backend know the
  // load/store accessing this alloca are accessing block ram.
  mutateAddressSpace(GV, CurAddrSpace);
  for (GVUseCollector::iterator I = Collector.Uses.begin(),
       E = Collector.Uses.end(); I != E; ++I) {
    GVUseCollector::UsesVec &Uses = I->second;

    while (!Uses.empty())
      // No need to mutate address space if V is a place holder.
      if (Value *V = Uses.pop_back_val()) mutateAddressSpace(V, CurAddrSpace);

    if (I->first == 0) continue;
    
    Instruction *IP = I->first->getEntryBlock().getTerminator();
    annotateBRAMInfo(CurAddrSpace, GV->getType()->getElementType(), GV, IP, M);
  }

  ++CurAddrSpace;
  ++NumLocalizedGV;
  return true;
}

bool BlockRAMFormation::runOnModule(Module &M) {
  bool changed = false;
  TD = &getAnalysis<TargetData>();

  if (EnableBRAM)
    for (Module::global_iterator I = M.global_begin(), E = M.global_end();
         I != E; ++I)
      changed |= localizeGV(I, M);

  for (Module::iterator I = M.begin(), E = M.end(); I != E; ++I)
    changed |= runOnFunction(*I, M);

  return changed;
}

Pass *llvm::createBlockRAMFormation(const TargetIntrinsicInfo &IntrInfo) {
  return new BlockRAMFormation(IntrInfo);
}
