//==-ContoBrom.cpp ---This pass allocates BRoms to global constants===========//
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
// This file is a pass to transform the global constants' location from main
// memory to local BRom.
//
//===----------------------------------------------------------------------===//


#include "vtm/Passes.h"
#include "vtm/VerilgoBackendMCTargetDesc.h"

#include "llvm/Pass.h"
#include "llvm/Module.h"
#include "llvm/Function.h"
#include "llvm/Constants.h"
#include "llvm/GlobalVariable.h"
#include "llvm/Instructions.h"
#include "llvm/Target/TargetIntrinsicInfo.h"
#include "llvm/Target/TargetData.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/MachineFunction.h"
#define DEBUG_TYPE "ContoBrom"
#include "llvm/Support/Debug.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/raw_ostream.h"
#include <set>
#include <map>

using namespace llvm;

//===----------------------------------------------------------------------===//
// This pass allocates a BRom for each global constant in the function and
// replaces the load instruction by a call to vtm_access_bram intrinsic
// function, thus allowing a local BRom access to the global constant.

namespace {
 struct ContoBrom: public ModulePass {
   static char ID;
   Module* Mod;
   const TargetIntrinsicInfo &IntrinsicInfo;
   unsigned AllocatedBRamNum;

   // A set to track the GlobalVariable.
   std::set<Value*> VisitedGV;
   // The Two maps below is to record the AllocaBramfucntion and BramNum, they
   // are needed when we call the GetOrCreateBram function.
   std::map<GlobalVariable*, Function*> TheAllocaBramFnMap;
   std::map<GlobalVariable*, unsigned> BramNumMap;

   // Define the a small vector of the WorkStack.
   typedef SmallVector<std::pair<Value*, Value::use_iterator>, 16> WorkStackTy;
   // Create a WorkStack for depth first search use tree.
   WorkStackTy WorkStack;
   // Create a Depth First Search use Tree Vector for the LowerGVToBram.
   WorkStackTy DFSTVector;

   ContoBrom(const TargetIntrinsicInfo &II) : ModulePass(ID),
                                              IntrinsicInfo(II),
                                              AllocatedBRamNum(1){}

   bool runOnModule(Module &M);

   // Depth first search the GlobalVariable, return true if we can handle this
   // GlobalVariable.
   bool VisitGVUsers(GlobalVariable *GV);

   // Lower the GlobalVariable to the local block ram.
   void LowerGVToBram(GlobalVariable *GV);

   // Declare vtm_alloca_bram intrinsic function of GloabalVariable.
   void GetOrCreateBram(GlobalVariable *GV, Function *F, Instruction *I);

   // replace the GEPInst or LoadInst with intrinsic to access local Bram.
   void ReplaceLoadInstWithBRamAccess(LoadInst *LI);
  };

}//end of anonymous namespace

bool ContoBrom::runOnModule(Module &M) {
  Mod = &M;
  //iterate all over the module graph
  for (Module::global_iterator I = M.global_begin(), E = M.global_end();
    I != E; ++I) {
      GlobalVariable *GV = dyn_cast<GlobalVariable>(I);

      // Firstly, it should not be empty, but a GlobalVariable.
      if (!GV)
        continue;

      // Secondly, it should has internal linkage or private linkage.
      if (!(GV->hasInternalLinkage() || GV->hasPrivateLinkage()))
        continue;

      if (!GV->hasInitializer())
        continue;

      // If the Con is a multi-dimension constant array, continue.
      Constant *Con = cast<Constant>(GV->getInitializer());
      if (const ArrayType* AT = dyn_cast<ArrayType>(Con->getType())) {
        const Type* ET = AT->getElementType();
        if (isa<ArrayType>(ET))
          continue;
      }

      // Whether we can lower the GlobalVariable to Bram,
      // continue if we can not lower it.
      if (!VisitGVUsers(GV))
        continue;

      // Lower the GlobalVariable if we can lower it.
      LowerGVToBram(GV);
      DFSTVector.clear();
      WorkStack.clear();
  }
  return true;
}

bool ContoBrom::VisitGVUsers(GlobalVariable *GV){

  //Preserve the root node.
  WorkStack.push_back(std::make_pair(GV, GV->use_begin()));

  // Remember what we had visited.
  std::set<Value*> VisitedUses;

  // The following variables are for DFS(Depth First Search) for the GV user
  // tree.
  while(!WorkStack.empty()) {
    Value* Node = WorkStack.back().first;
    Value::use_iterator ChildIt = WorkStack.back().second;


    // Do we reach the end of the Nodes or Is the ChildNode a LoadInst?
    if (ChildIt == Node->use_end()) {
      WorkStack.pop_back();
      continue;
    }

    // Push back the Use Tree for LowerGVToBram.
    DFSTVector.push_back(std::make_pair(Node, ChildIt));

    // Depth first search the child of current node.
    Value *ChildNode = *ChildIt;
    ++WorkStack.back().second;

    if (isa<LoadInst>(ChildNode)){
      continue;
    }

    // If there are nodes that we can not handle, return false.
    if (!isa<GetElementPtrInst>(*ChildIt)) {
      DFSTVector.clear();
      return false;
    }

    // Had we visited this node?
    if (!VisitedUses.insert(ChildNode).second) continue;

    // If ChildNode is not visited, go on visit it and its children.
    WorkStack.push_back(std::make_pair(ChildNode, ChildNode->use_begin()));
  }

  // Now we can handle the GlobalVariable, return true.
  return true;
}

void ContoBrom::LowerGVToBram(GlobalVariable *GV){

  for(WorkStackTy::iterator I = DFSTVector.begin(), E = DFSTVector.end();
      I != E; ++I){

    // We need the function pointer the get the entry point when we use the
    // GetOrCreateBram function.
    Function *F = 0;

    // If the Instruction is GetElementPtrInst, allocate a Bram if the module
    // haven't alloca a Bram for the GlobalVariable.
    if (GetElementPtrInst *GEP = dyn_cast<GetElementPtrInst>(*I->second)){
      F = GEP->getParent()->getParent();
      GetOrCreateBram(GV, F, GEP);
      continue;
    }

    // If the Instruction is LoadInst, allocate a Bram if the module
    // haven't alloca a Bram for the GlobalVariable, and replace the LoadInst
    // with vtm_access_bram intrinsic function.
    if (LoadInst *LI = dyn_cast<LoadInst>(*I->second)){
      F = LI->getParent()->getParent();
      GetOrCreateBram(GV, F, LI);
      ReplaceLoadInstWithBRamAccess(LI);
    }
  }
}

void ContoBrom::GetOrCreateBram(GlobalVariable *GV, Function *F,
                                    Instruction *I){
  // dirty hack!!
  Value* V = GV->getInitializer();
  Constant* Con = cast<Constant>(V);
  // Gather the parameters of vtm_alloca_brom
  ArrayType *AT = cast<ArrayType>(Con->getType());
  Type *ET = AT->getElementType();
  uint16_t NumElems = AT->getNumElements();
  uint16_t ElemSizeInBytes = ET->getPrimitiveSizeInBits() / 8;
  Instruction* EntryPoint = F->begin()->begin();

  // allocate space for the alloca and get declaration
  // of the intrinsic function.
  unsigned AllocaAddrSpace = GV->getType()->getAddressSpace();

  // Transform the Element type into a pointer that
  // points to the Element
  Type* NewPtrTy = PointerType::get(ET, AllocaAddrSpace);
  Type* CAPtrTy = PointerType::get(ET, AllocaAddrSpace);

  // If we do not have alloca a Bram for the GV, Allocate a Bram for it.
  if (VisitedGV.insert(GV).second) {
  Type *ValTysAL[] = { NewPtrTy, CAPtrTy};
  Function* TheAllocaBramFn
    = IntrinsicInfo.getDeclaration(Mod, vtmIntrinsic::vtm_alloca_bram,
                                   ValTysAL, array_lengthof(ValTysAL));
  // record the AllocaBRamFunction and BramNum for the GV.
  TheAllocaBramFnMap[GV] = TheAllocaBramFn;
  BramNumMap[GV] = AllocatedBRamNum;
  // increase the AllocatedBramNum for the next GV.
  AllocatedBRamNum++;
  }
  // get the type of the context, construct the Args to the AllocaBromInst,
  // and creat a CallInst to call the intrinsic alloca function
  IntegerType *Int32TyAL = Type::getInt32Ty(Mod->getContext());
  CastInst* CIET = BitCastInst::CreatePointerCast(GV, CAPtrTy, 
                                                  "const_cast", EntryPoint);

  Value* ArgsAL[] = {CIET,
                     ConstantInt::get(Int32TyAL, BramNumMap[GV]),
                     ConstantInt::get(Int32TyAL, NumElems),
                     ConstantInt::get(Int32TyAL, ElemSizeInBytes)};
  Instruction* AllocaBram
    = CallInst::Create(TheAllocaBramFnMap[GV], ArgsAL,
                       I->getName(), EntryPoint);

  // If the Instruction is a LoadInst, set the Operand of the Load Instruction
  // when the operand is a GV. When the operand of the Load Instruction is a
  // GEP(it will be a cast Instruction), we do nothing to the instruction.
  if (isa<LoadInst>(I)){
    if(!isa<GlobalVariable>(I->getOperand(0))){
      return;
    }
    // Replace the GV with CastInst.
    I->setOperand(0, AllocaBram);
    return;
  }

  // The CallInst return a i32* pointer, but the GEP Instruction need its first
  // operand to be a array pointer [256 * i32]*. So we need to Cast pointer that
  // pointing array element to pointer that pointing array.
  Type *ArrayPtrTy = PointerType::get(AT, AllocaAddrSpace);
  CastInst* CIAT = BitCastInst::CreatePointerCast(AllocaBram, ArrayPtrTy,
                                                  "cast", EntryPoint);
  // Replace the GV with CastInst.
  I->setOperand(0, CIAT);
}

void ContoBrom::ReplaceLoadInstWithBRamAccess(LoadInst *LI){
  Value *Ptr = LI->getPointerOperand();
  Type *ValTys[] = { LI->getType(), Ptr->getType() };
  Function *TheLoadBRamFn
    = IntrinsicInfo.getDeclaration(Mod, vtmIntrinsic::vtm_access_bram,
                                   ValTys, array_lengthof(ValTys));
  IntegerType *Int32Ty = Type::getInt32Ty(Mod->getContext()),
              *Int1Ty = Type::getInt1Ty(Mod->getContext());

  Value *Args[] = { Ptr, UndefValue::get(ValTys[0]),
                    ConstantInt::get(Int1Ty, 0),
                    ConstantInt::get(Int32Ty, LI->getAlignment()),
                    ConstantInt::get(Int1Ty, LI->isVolatile()),
                    ConstantInt::get(Int32Ty, AllocatedBRamNum-1) };

  CallInst *NewLD = CallInst::Create(TheLoadBRamFn, Args, LI->getName(), LI);
  LI->replaceAllUsesWith(NewLD);
  LI->eraseFromParent();
}

char ContoBrom::ID = 0;
Pass *llvm::createContoBromPass(const TargetIntrinsicInfo &IntrinsicInfo) {
  return new ContoBrom(IntrinsicInfo);
}
