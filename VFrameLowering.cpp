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
#include "llvm/Analysis/ValueTracking.h"
#include "llvm/Target/TargetData.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/Support/InstIterator.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/CommandLine.h"
#define DEBUG_TYPE "vtm-frame-lowering"
#include "llvm/Support/Debug.h"

using namespace llvm;

static cl::opt<unsigned>
UnderlyingObjectMaxLookUp("vtm-underlying-object-max-lookup",
  cl::desc("The Value of MaxLoopUp passed to GetUnderlyingObject"),
  cl::Hidden,
  cl::init(8));

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
  TargetData *TD;

  const TargetIntrinsicInfo &IntrinsicInfo;
  LowerFrameInstrs(const TargetIntrinsicInfo &II)
    : FunctionPass(ID), M(0), IntrinsicInfo(II) {}

  //template<class InstTy, class OpIt>
  //InstTy *CreateInst(InstTy *OldInst, OpIt OpBegin, OpIt OpEnd)
  VAllocaBRamInst *getReferredBRam(Value *V) const;
  void replaceBRamAccess(Function *F);

  unsigned lowerAlloca(AllocaInst *AI, unsigned allocatedBRams);

  bool runOnFunction(Function &F);
};
}

char LowerFrameInstrs::ID = 0;

VAllocaBRamInst *LowerFrameInstrs::getReferredBRam(Value *V) const {
  Instruction *Instr = dyn_cast<Instruction>(V);

  // Only an instruction may refer a block ram allocate instruction.
  if (!Instr) return 0;

  Value *Ptr = GetUnderlyingObject(Instr, TD,
                                   UnderlyingObjectMaxLookUp);

  return dyn_cast<VAllocaBRamInst>(Ptr);
}

void LowerFrameInstrs::replaceBRamAccess(Function *F) {
  SmallVector<Instruction*, 128> Insts;

  // Collect the instructions need to be replaced.
  for (inst_iterator I = inst_begin(F), E = inst_end(F); I != E; ++I) {
    Instruction &Inst = *I;
    if (isa<LoadInst>(Inst) || isa<StoreInst>(Inst))
      Insts.push_back(&Inst);
  }

  // Replace the instructions
  while (!Insts.empty()) {
    Instruction *Inst = Insts.pop_back_val();

    if (LoadInst *Load = dyn_cast<LoadInst>(Inst)) {
      Value *Ptr = Load->getPointerOperand();
      if (VAllocaBRamInst *Ram = getReferredBRam(Ptr)) {
        const Type *ValTys[] = { Load->getType(), Ptr->getType() };
        Function *TheLoadBRamFn
          = IntrinsicInfo.getDeclaration(M, vtmIntrinsic::vtm_access_bram,
          ValTys, array_lengthof(ValTys));
        const Type *Int32Ty = Type::getInt32Ty(M->getContext()),
          *Int1Ty = Type::getInt1Ty(M->getContext());

        Value *Args[] = { Ptr, UndefValue::get(ValTys[0]),
                          ConstantInt::get(Int1Ty, 0),
                          ConstantInt::get(Int32Ty, Load->getAlignment()),
                          ConstantInt::get(Int1Ty, Load->isVolatile()),
                          ConstantInt::get(Int32Ty,  Ram->getBRamId()) };

        CallInst *NewLD = CallInst::Create(TheLoadBRamFn,
                                           Args, array_endof(Args),
                                           Load->getName(), Load);
        Load->replaceAllUsesWith(NewLD);
        Load->eraseFromParent();
      }
      continue;
    }

    if (StoreInst *Store = dyn_cast<StoreInst>(Inst)) {
      Value *Ptr = Store->getPointerOperand();
      if (VAllocaBRamInst *Ram = getReferredBRam(Ptr)) {
        Value *ValueOperand = Store->getValueOperand();
        assert(getReferredBRam(ValueOperand) == 0
               && "Can not store block ram address!");
        const Type *ValTys[] = { ValueOperand->getType(), Ptr->getType() };
        Function *TheStoreBRamFn
          = IntrinsicInfo.getDeclaration(M, vtmIntrinsic::vtm_access_bram,
          ValTys, array_lengthof(ValTys));
        const Type *Int32Ty = Type::getInt32Ty(M->getContext()),
                   *Int1Ty = Type::getInt1Ty(M->getContext());

        Value *Args[] = { Ptr, ValueOperand,
                          ConstantInt::get(Int1Ty, 1),
                          ConstantInt::get(Int32Ty, Store->getAlignment()),
                          ConstantInt::get(Int1Ty, Store->isVolatile()),
                          ConstantInt::get(Int32Ty,  Ram->getBRamId()) };

        (void) CallInst::Create(TheStoreBRamFn, Args, array_endof(Args),
                                "store_bram", Store);
        // The old store is not use anymore.
        Store->eraseFromParent();
      }
      continue;
    }

    // Else check the operand of the instruction, find out the unsupport case.
  }
}

unsigned LowerFrameInstrs::lowerAlloca(AllocaInst *AI, unsigned allocatedBRams){
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
  unsigned AllocaAddrSpace = AI->getType()->getAddressSpace();
  const Type *NewPtrTy = PointerType::get(ET, AllocaAddrSpace);
  Function *TheAllocaBRamFn
    = IntrinsicInfo.getDeclaration(M, vtmIntrinsic::vtm_alloca_bram,
                                   &NewPtrTy, 1);

  const Type *Int32Ty = Type::getInt32Ty(Context);
  Value *Args[] = { ConstantInt::get(Int32Ty, allocatedBRams),
                    ConstantInt::get(Int32Ty, NumElems),
                    ConstantInt::get(Int32Ty, ElemSizeInBytes) };
  
  Instruction *AllocaBRam = CallInst::Create(TheAllocaBRamFn,
                                             Args, array_endof(Args),
                                             AI->getName(), AI);
  // Cast pointer that pointing array element to pointer that pointing array.
  const Type *ArrayPtrTy = PointerType::get(AT, AllocaAddrSpace);

  AI->replaceAllUsesWith(BitCastInst::CreatePointerCast(AllocaBRam, ArrayPtrTy,
                                                        "cast", AI));
  AI->eraseFromParent();

  return ++allocatedBRams;
}

bool LowerFrameInstrs::runOnFunction(Function &F) {
  TD = getAnalysisIfAvailable<TargetData>();
  M = F.getParent();
  unsigned allocatedBRams = 1;

  SmallVector<AllocaInst*, 16> Allocas;

  for (inst_iterator I = inst_begin(F), E = inst_end(F); I != E; ++I)
    if (AllocaInst *AI = dyn_cast<AllocaInst>(&*I))
      Allocas.push_back(AI);

  if (Allocas.empty()) return false;

  while (!Allocas.empty()) {
    allocatedBRams = lowerAlloca(Allocas.back(), allocatedBRams);
    Allocas.pop_back();
  }

  replaceBRamAccess(&F);

  return true;
}

Pass
  *llvm::createLowerFrameInstrsPass(const TargetIntrinsicInfo &IntrinsicInfo) {
  return new LowerFrameInstrs(IntrinsicInfo);
}
