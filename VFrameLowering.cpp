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
#include "llvm/Support/CallSite.h"
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
STATISTIC(NumInitializedGV,
          "Number of GlobalVariable with initializer localized");


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
  bool replaceCallUser(GlobalVariable *GV);
  bool replaceCallUser(GlobalVariable *GV, Function *Callee, unsigned ArgNo,
                       ArrayRef<CallSite> Users);

  GlobalVariable *allocateGlobalAlias(AllocaInst *AI, Module &M,
                                      unsigned AddressSpace = 0);
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
        // Bitcast used by lifetime markers is allowed.
      case Instruction::BitCast: return onlyUsedByLifetimeMarkers(ValUser);
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

  static bool isLocalizedCandidate(GlobalVariable *GV) {
    // Cannot localize if the GV may be modified by others module.
    if (!GV->hasInternalLinkage() && !GV->hasPrivateLinkage() &&
        (!GV->isConstant() || GV->isDeclaration()))
      return false;

    assert(GV->hasInitializer() && "Unexpected declaration!");

    return true;
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

GlobalVariable *BlockRAMFormation::allocateGlobalAlias(AllocaInst *AI, Module &M,
                                                       unsigned AddressSpace) {
  Type *AllocatedType = AI->getAllocatedType();
  // Create the global alias.
  GlobalVariable *GV =
    new GlobalVariable(M, AllocatedType, false, GlobalValue::InternalLinkage,
                       Constant::getNullValue(AllocatedType),
                       AI->getName() + utostr_32(AllocaAliasCnt) + "_g_alias",
                       0, GlobalVariable::NotThreadLocal, AddressSpace);
  GV->setAlignment(AI->getAlignment());
  // Replace the alloca by the global variable.
  // Please note that this operation make the function become no reentrantable.
  AI->replaceAllUsesWith(GV);

  BasicBlock::iterator IP = AI->getParent()->getTerminator();
  Value *Arg = GV;

  if (!GV->getType()->isPrimitiveType()) {
    PointerType *PtrTy
      = PointerType::getIntNPtrTy(M.getContext(), 8, AddressSpace);
    Arg = ConstantExpr::getBitCast(GV, PtrTy);
  }

  Type *ArgTypes[] = { Arg->getType() };

  Function *Privatize =
    IntrInfo.getDeclaration(&M, vtmIntrinsic::vtm_privatize_global,
                            ArgTypes, array_lengthof(ArgTypes));
  CallInst::Create(Privatize, Arg, "", IP);

  AI->eraseFromParent();

  ++NumGlobalAlias;
  ++AllocaAliasCnt;
  return GV;
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
  for (inst_iterator I = inst_begin(F), E = inst_end(F); I != E; /*++I*/) {
    AllocaInst *AI = dyn_cast<AllocaInst>(&*I++);
    if (!AI) continue;

    BasicBlock *BB = AI->getParent();

    changed |= true;
    // Can us handle the use tree of the allocated pointer?
    if (!EnableBRAM || !visitPtrUseTree(AI, Collector)) {
      Collector.Uses.clear();
      // Otherwise, we need to allocate the object in global memory.
      allocateGlobalAlias(AI, M);
      continue;
    }

    // Change the address space of the alloca, so the backend know the
    // load/store accessing this alloca are accessing block ram.
    mutateAddressSpace(AI, CurAddrSpace);
    while (!Collector.Uses.empty())
      mutateAddressSpace(Collector.Uses.pop_back_val(), CurAddrSpace);

    GlobalVariable *GV = allocateGlobalAlias(AI, M, CurAddrSpace);
    Constant *NullInitilizer =
      Constant::getNullValue(PointerType::getIntNPtrTy(F.getContext(), 8, 0));
    annotateBRAMInfo(CurAddrSpace, GV->getType()->getElementType(),
                     NullInitilizer, BB->getTerminator(), M);

    ++NumBlockRAMs;
    ++CurAddrSpace;
  }

  return changed;
}

bool BlockRAMFormation::localizeGV(GlobalVariable *GV, Module &M) {
  if (!GVUseCollector::isLocalizedCandidate(GV)) return false;

  GVUseCollector Collector;

  if (!GV->getInitializer()->isNullValue())
    ++NumInitializedGV;

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

bool BlockRAMFormation::replaceCallUser(GlobalVariable *GV) {
  typedef DenseMap<std::pair<Function *, unsigned>, SmallVector<CallSite, 4> >
          CallUsersMap;
  CallUsersMap CallUsers;
  bool changed = false;

  typedef Value::use_iterator use_iterator;
  for (use_iterator I = GV->use_begin(), E = GV->use_end(); I != E; ++I) {
    CallInst *CI = dyn_cast<CallInst>(*I);

    if (CI == 0) continue;

    // Collect the CallSites.
    CallSite CS(CI);
    Function *Callee = CS.getCalledFunction();
    unsigned ArgNo = CS.getArgumentNo(I);
    CallUsers[std::make_pair(Callee, ArgNo)].push_back(CS);
  }

  typedef CallUsersMap::iterator iterator;
  for (iterator I = CallUsers.begin(), E = CallUsers.end(); I != E; ++I)
    changed |= replaceCallUser(GV, I->first.first, I->first.second, I->second);

  return changed;
}

bool BlockRAMFormation::replaceCallUser(GlobalVariable *GV,Function *Callee,
                                        unsigned ArgNo,
                                        ArrayRef<CallSite> Users) {
  // The user of the GV not cover all call of the Function.
  if (Users.size() != Callee->getNumUses()) return false;

  // Replace the argument by the GlobalVarialbe.
  Argument *Arg = llvm::next(Callee->arg_begin(), ArgNo);
  Arg->replaceAllUsesWith(GV);

  for (unsigned i = 0; i < Users.size(); ++i) {
    CallSite CS = Users[i];
    CS.setArgument(ArgNo, Constant::getNullValue(GV->getType()));
  }

  return true;
}

bool BlockRAMFormation::runOnModule(Module &M) {
  bool changed = false;
  TD = &getAnalysis<TargetData>();

  typedef Module::global_iterator global_iterator;

  if (EnableBRAM)
    for (global_iterator I = M.global_begin(), E = M.global_end(); I != E; ++I)
      changed |= localizeGV(I, M);

  for (Module::iterator I = M.begin(), E = M.end(); I != E; ++I)
    changed |= runOnFunction(*I, M);

  // If the argument of a function is always passed by a GV, replace the
  // argument by the GV in that function, by doing this, we can get better
  // alias analysis.
  bool LocalChange = true;

  // Iterate untill the module is stable.
  while (LocalChange) {
    LocalChange = false;
    for (global_iterator I = M.global_begin(), E = M.global_end(); I != E; ++I)
      changed |= LocalChange |= replaceCallUser(I);
  }

  return changed;
}

Pass *llvm::createBlockRAMFormation(const TargetIntrinsicInfo &IntrInfo) {
  return new BlockRAMFormation(IntrInfo);
}
