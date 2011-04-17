//=======- VFrameInfo.cpp - VTM Frame Information -----------*- C++ -*-=====//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
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
#include "llvm/ADT/SmallVector.h"
#include "llvm/Support/InstIterator.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#define DEBUG_TYPE "vtm-frame-lowering"
#include "llvm/Support/Debug.h"

using namespace llvm;

void VFrameInfo::emitPrologue(MachineFunction &MF) const {
}

void VFrameInfo::emitEpilogue(MachineFunction &MF,
                              MachineBasicBlock &MBB) const {
}


//===----------------------------------------------------------------------===//
// This pass translate all alloca instruction to llvm.vtm.alloca_bram intrinsics.
//
namespace {
struct LowerFrameInstrs : public FunctionPass {
  static char ID;
  Module *M;

  const TargetIntrinsicInfo &IntrinsicInfo;
  LowerFrameInstrs(const TargetIntrinsicInfo &II)
    : FunctionPass(ID), M(0), IntrinsicInfo(II) {}

  //template<class InstTy, class OpIt>
  //InstTy *CreateInst(InstTy *OldInst, OpIt OpBegin, OpIt OpEnd)

  void ReplaceUseTree(Instruction *From, Instruction *To);

  unsigned LowerAlloca(AllocaInst *AI, unsigned allocatedBRams);

  bool runOnFunction(Function &F);
};
}

char LowerFrameInstrs::ID = 0;

void LowerFrameInstrs::ReplaceUseTree(Instruction *From, Instruction *To) {
  std::vector<User*> Uses(From->use_begin(), From->use_end());

  while(!Uses.empty()) {
    Instruction *Instr = cast<Instruction>(Uses.back());
    Uses.pop_back();
    
    switch (Instr->getOpcode()) {
    case Instruction::GetElementPtr: {
      GetElementPtrInst *GEP = cast<GetElementPtrInst>(Instr);
      assert(From == GEP->getPointerOperand() &&  "What are we replacing?");

      SmallVector<Value*, 8> Ops(GEP->idx_begin(), GEP->idx_end());
      GetElementPtrInst *NewGEP =
        GetElementPtrInst::Create(To, Ops.begin(), Ops.end(),
                                  GEP->getName(), GEP);
      NewGEP->setIsInBounds(GEP->isInBounds());

      ReplaceUseTree(GEP, NewGEP);
    }  break;
    case Instruction::Store:{
      StoreInst *OldST = cast<StoreInst>(Instr);
      assert(From == OldST->getPointerOperand() && "What are we replacing?");

      // We use address space
      const Type *ValTys[] = { OldST->getValueOperand()->getType(),
                              To->getType() };
      Function *TheStoreBRamFn
        = IntrinsicInfo.getDeclaration(M, vtmIntrinsic::vtm_access_bram,
                                       ValTys, array_lengthof(ValTys));
      const Type *Int32Ty = Type::getInt32Ty(M->getContext()),
                 *Int1Ty = Type::getInt1Ty(M->getContext());

      Value *Args[] = { To, OldST->getValueOperand(),
                        ConstantInt::get(Int1Ty, 1),
                        ConstantInt::get(Int32Ty, OldST->getAlignment()),
                        ConstantInt::get(Int1Ty, OldST->isVolatile()) };

      (void) CallInst::Create(TheStoreBRamFn, Args, array_endof(Args),
                              "store_bram", OldST);
      // The old store is not use anymore.
      OldST->eraseFromParent();
    }  break;
    case Instruction::Load: {
      LoadInst *OldLD = cast<LoadInst>(Instr);
      assert(From == OldLD->getPointerOperand() && "What are we replacing?");

            // We use address space 
      const Type *ValTys[] = { OldLD->getType(), To->getType() };
      Function *TheLoadBRamFn
        = IntrinsicInfo.getDeclaration(M, vtmIntrinsic::vtm_access_bram,
                                       ValTys, array_lengthof(ValTys));
      const Type *Int32Ty = Type::getInt32Ty(M->getContext()),
                 *Int1Ty = Type::getInt1Ty(M->getContext());

      Value *Args[] = { To, UndefValue::get(ValTys[0]),
                        ConstantInt::get(Int1Ty, 0),
                        ConstantInt::get(Int32Ty, OldLD->getAlignment()),
                        ConstantInt::get(Int1Ty, OldLD->isVolatile()) };

      CallInst *NewLD = CallInst::Create(TheLoadBRamFn, Args, array_endof(Args),
                                         OldLD->getName(), OldLD);
      OldLD->replaceAllUsesWith(NewLD);
      OldLD->eraseFromParent();
    } break;
    case Instruction::BitCast: {
      BitCastInst *OldCast = cast<BitCastInst>(Instr);
      assert(From == OldCast->getOperand(0) && "What are we replacing?");
      assert(OldCast->getDestTy()->isPointerTy()
             && "Cannot handle other cast yet!");
      const PointerType *OldType = cast<PointerType>(OldCast->getDestTy());
      unsigned AddrSpace = cast<PointerType>(To->getType())->getAddressSpace();
      const PointerType *NewType = PointerType::get(OldType->getElementType(),
                                                    AddrSpace);
      // Try to use IRBuilder to fold the cast.
      Value *NewCast = BitCastInst::CreatePointerCast(To, NewType,
                                                      OldCast->getName(),
                                                      OldCast);
      ReplaceUseTree(OldCast, cast<Instruction>(NewCast));
    } break;
    // TODO: PHINode
    default:
      DEBUG(dbgs() << "Unknown user: " << *Instr  << '\n');
      llvm_unreachable("Only support load/store/gep!");
      return;
    }
  }

  // Erase From after it replaced.
  assert(From->use_empty() && "Tree not replaced!");
  From->eraseFromParent();
}

unsigned LowerFrameInstrs::LowerAlloca(AllocaInst *AI, unsigned allocatedBRams){
  M = AI->getParent()->getParent()->getParent();
  LLVMContext &Context = M->getContext();

  assert(AI->isStaticAlloca() && "Cannot support dynamic alloca yet!");
  unsigned NumElems = cast<ConstantInt>(AI->getArraySize())->getZExtValue();
  
  assert(isa<ArrayType>(AI->getAllocatedType()) && "Only support Array now!");
  const ArrayType *AT = cast<ArrayType>(AI->getAllocatedType());
  const Type *ET = AT->getElementType();
  unsigned ElemSizeInBytes = ET->getPrimitiveSizeInBits() / 8;
  assert(ElemSizeInBytes && "Non-primitive type are not supported!");
  NumElems *= AT->getNumElements();

  // We use address space 
  const Type *NewPtrTy = PointerType::get(ET, allocatedBRams);
  Function *TheAllocaBRamFn
    = IntrinsicInfo.getDeclaration(M, vtmIntrinsic::vtm_alloca_bram,
                                   &NewPtrTy, 1);

  const Type *Int32Ty = Type::getInt32Ty(Context);
  Value *Args[] = { ConstantInt::get(Int32Ty, NumElems),
                    ConstantInt::get(Int32Ty, ElemSizeInBytes) };
  
  Instruction *AllocaBRam = CallInst::Create(TheAllocaBRamFn,
                                             Args, array_endof(Args),
                                             AI->getName(), AI);
  // Cast pointer that pointing array element to pointer that pointing array.
  const Type *ArrayPtrTy = PointerType::get(AT, allocatedBRams);

  ReplaceUseTree(AI, BitCastInst::CreatePointerCast(AllocaBRam,
                                                    ArrayPtrTy, "cast", AI));

  return ++allocatedBRams;
}

bool LowerFrameInstrs::runOnFunction(Function &F) {
  unsigned allocatedBRams = 1;

  std::vector<AllocaInst*> Allocas;

  for (inst_iterator I = inst_begin(F), E = inst_end(F); I != E; ++I)
    if (AllocaInst *AI = dyn_cast<AllocaInst>(&*I))
      Allocas.push_back(AI);

  while (!Allocas.empty()) {
    allocatedBRams = LowerAlloca(Allocas.back(), allocatedBRams);
    Allocas.pop_back();
  }

  return allocatedBRams != 1;
}

Pass
  *llvm::createLowerFrameInstrsPass(const TargetIntrinsicInfo &IntrinsicInfo) {
  return new LowerFrameInstrs(IntrinsicInfo);
}
