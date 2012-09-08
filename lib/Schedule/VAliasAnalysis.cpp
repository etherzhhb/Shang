//===---- VAliasAnalysis.cpp - VTM Specific Alias Analysis  -----*- C++ -*-===//
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
// This file implement the Verilog Target Machine specific alias analysis.
//
//===----------------------------------------------------------------------===//
#include "vtm/Passes.h"
#include "vtm/VerilogBackendMCTargetDesc.h"

#include "llvm/GlobalVariable.h"
#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/Analysis/ScalarEvolution.h"
#include "llvm/Analysis/ScalarEvolutionExpressions.h"
#include "llvm/Analysis/ValueTracking.h"
#include "llvm/Target/TargetData.h"
#include "llvm/Target/TargetIntrinsicInfo.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/Pass.h"
#include "llvm/ADT/STLExtras.h"
using namespace llvm;

namespace {
struct VAliasAnalysis: public ImmutablePass, public AliasAnalysis {
  static char ID;
  const TargetIntrinsicInfo *II;

  explicit VAliasAnalysis(const TargetIntrinsicInfo *II = 0)
                       : ImmutablePass(ID), II(II) {
    initializeVAliasAnalysisPass(*PassRegistry::getPassRegistry());
    assert(II && "TargetIntrinsicInfo not available!"
           "(Constructed by default constructor?)");
  }

  virtual void *getAdjustedAnalysisPointer(AnalysisID ID) {
    if (ID == &AliasAnalysis::ID)
      return (AliasAnalysis*)this;
    return this;
  }

  virtual void initializePass() {
    InitializeAliasAnalysis(this);
  }

  void getAnalysisUsage(AnalysisUsage &AU) const {
    AU.addRequired<AliasAnalysis>();
    AU.addRequired<TargetData>();
  }

  bool isNoAliasArgAndGV(const Value *LHS, const Value *RHS) {
    const Argument *Arg = dyn_cast<Argument>(LHS);

    if (!Arg) return false;

    // Try to retrieve annotation intrinsic.
    const CallInst *Privatizer = 0;
    typedef Value::const_use_iterator use_iterator;
    for (use_iterator I = RHS->use_begin(), E = RHS->use_end(); I != E; ++I) {
      Privatizer = dyn_cast<CallInst>(*I);

      if (Privatizer == 0) {
        if (const ConstantExpr *C = dyn_cast<ConstantExpr>(*I))
          if (C->getOpcode() == Instruction::BitCast
              && isNoAliasArgAndGV(LHS, C))
          return true;

        continue;
      }

      ImmutableCallSite CS(Privatizer);
      const Function *F = CS.getCalledFunction();
      if (!F || !F->isDeclaration()) return false;

      unsigned ID = II->getIntrinsicID(const_cast<Function*>(F));

      if (ID != vtmIntrinsic::vtm_privatize_global) {
        // Not the privatize intrinsic, reset the result.
        Privatizer = 0;
        continue;
      }

      // Intrinsic found.
      break;
    }

    // The privatize intrinsic not found.
    if (Privatizer  == 0) return false;

    // Since the privatized global variable is converted from the alloca, we can
    // follow the rule in BasicAliasAnalysis: Argument not alias with AllocaInst.
    return Arg->getParent() == Privatizer->getParent()->getParent();
  }

  AliasResult alias(const Location &LocA, const Location &LocB) {
    // Figure out what objects these things are pointing to if we can.
    const Value *O1 = GetUnderlyingObject(LocA.Ptr->stripPointerCasts(), TD);
    const Value *O2 = GetUnderlyingObject(LocB.Ptr->stripPointerCasts(), TD);

    PointerType *ATy = cast<PointerType>(O1->getType()),
                *BTy = cast<PointerType>(O2->getType());

    // Pointers pointing to different address space never alias.
    if (ATy->getAddressSpace() != BTy->getAddressSpace())
      return NoAlias;

    // Handle the privatize global variables, which are converted from allocas.
    if (isNoAliasArgAndGV(O1, O2) || isNoAliasArgAndGV(O2, O1))
      return NoAlias;

    // TODO: Constant expression.

    return AliasAnalysis::alias(LocA, LocB);
  }
};

}

char VAliasAnalysis::ID = 0;

INITIALIZE_AG_PASS(VAliasAnalysis, AliasAnalysis, "vaa",
                   "VTM Specific Alias Analyais", false, true, false)

Pass *llvm::createVAliasAnalysisPass(const TargetIntrinsicInfo *II) {
  return new VAliasAnalysis(II);
}

namespace llvm {
std::pair<const Value*, int64_t>
extractPointerAndOffset(const Value *V, int64_t Offset) {
  // Try to strip pointer casts.
  if (const ConstantExpr *E = dyn_cast<ConstantExpr>(V)) {
    if (E->getOpcode() == Instruction::IntToPtr) {
      if (ConstantExpr *Int = dyn_cast<ConstantExpr>(E->getOperand(0))) {
        switch (Int->getOpcode()) {
        default: break;
        // IntToPtr(PtrToInt A)
        case Instruction::PtrToInt:
          return extractPointerAndOffset(Int->getOperand(0), Offset);
        // IntToPtr(PtrToInt A + Offset)
        case Instruction::Add: {
          const Constant *LHS = Int->getOperand(0),
                         *RHS = Int->getOperand(1);
          if (const ConstantExpr *Ptr = dyn_cast<ConstantExpr>(LHS))
            if (Ptr->getOpcode() == Instruction::PtrToInt)
              if (const ConstantInt *COffset = dyn_cast<ConstantInt>(RHS))
                return extractPointerAndOffset(Ptr->getOperand(0),
                                               Offset+COffset->getSExtValue());
          break;
        }
        }
      }
    } else if (E->getOpcode() == Instruction::BitCast &&
               isa<PointerType>(E->getOperand(0)->getType()))
      return extractPointerAndOffset(E->getOperand(0), Offset);
  }

  return std::make_pair(V, Offset);
}

static const SCEV *getMachineMemOperandSCEV(const Value *V, int64_t Offset,
                                            ScalarEvolution *SE){
  tie(V, Offset) = extractPointerAndOffset(V, Offset);
  // Get the SCEV.
  const SCEV *S = SE->getSCEV(const_cast<Value*>(V));
  if (Offset)
    S = SE->getAddExpr(S, SE->getConstant(S->getType(), Offset, true));

  return S;
}

const SCEV *getMachineMemOperandSCEV(MachineMemOperand* V, ScalarEvolution *SE){
  return getMachineMemOperandSCEV(V->getValue(), V->getOffset(), SE);
}

const SCEV *getAddressDeltaSCEV(MachineMemOperand *LHS, MachineMemOperand *RHS,
                                ScalarEvolution *SE) {
  const SCEV *LHSSCEV = getMachineMemOperandSCEV(LHS, SE);
  const SCEV *RHSSCEV = getMachineMemOperandSCEV(RHS, SE);
  return SE->getMinusSCEV(LHSSCEV, RHSSCEV);
}

int64_t getAddressDelta(MachineMemOperand *LHS, MachineMemOperand *RHS,
                        ScalarEvolution *SE) {
  const SCEVConstant *Delta
    = cast<SCEVConstant>(getAddressDeltaSCEV(LHS, RHS, SE));
  return Delta->getValue()->getSExtValue();
}

static Value *GetBaseValue(const SCEV *S) {
  if (const SCEVAddRecExpr *AR = dyn_cast<SCEVAddRecExpr>(S)) {
    // In an addrec, assume that the base will be in the start, rather
    // than the step.
    return GetBaseValue(AR->getStart());
  } else if (const SCEVAddExpr *A = dyn_cast<SCEVAddExpr>(S)) {
    // If there's a pointer operand, it'll be sorted at the end of the list.
    const SCEV *Last = A->getOperand(A->getNumOperands()-1);
    if (Last->getType()->isPointerTy())
      return GetBaseValue(Last);
  } else if (const SCEVUnknown *U = dyn_cast<SCEVUnknown>(S)) {
    // This is a leaf node.
    return U->getValue();
  }
  // No Identified object found.
  return 0;
}

AliasAnalysis::AliasResult
MachineMemOperandAlias(MachineMemOperand* V1, MachineMemOperand *V2,
                       AliasAnalysis *AA, ScalarEvolution *SE) {
  uint64_t V1Size = V1->getSize(), V2Size = V2->getSize();
  // If either of the memory references is empty, it doesn't matter what the
  // pointer values are. This allows the code below to ignore this special
  // case.
  if (V1Size == 0 || V2Size == 0) return AliasAnalysis::NoAlias;

  Value *V1Ptr = const_cast<Value *>(V1->getValue()),
        *V2Ptr = const_cast<Value *>(V2->getValue());
  int64_t V1Offset = V1->getOffset(), V2Offset = V2->getOffset();
  // FIXME: already implemented in VAliasAnalysis.
  if (cast<PointerType>(V1Ptr->getType())->getAddressSpace() !=
      cast<PointerType>(V2Ptr->getType())->getAddressSpace())
    return AliasAnalysis::NoAlias;

  // AliasAnalysis can handle memory location with zero offset.
  // Ask AliasAnalysis for help.
  if (V1Offset == V2Offset) return AA->alias(V1Ptr, V1Size, V2Ptr, V2Size);

  // This is ScalarEvolutionAliasAnalysis. Get the SCEVs!
  const SCEV *V1S = getMachineMemOperandSCEV(V1, SE);
  const SCEV *V2S = getMachineMemOperandSCEV(V2, SE);

  // If they evaluate to the same expression, it's a MustAlias.
  if (V1S == V2S) return AliasAnalysis::MustAlias;
  // If something is known about the difference between the two addresses,
  // see if it's enough to prove a NoAlias.
  if (SE->getEffectiveSCEVType(V1S->getType()) ==
      SE->getEffectiveSCEVType(V2S->getType())) {
    unsigned BitWidth = SE->getTypeSizeInBits(V1S->getType());
    APInt ASizeInt(BitWidth, V1Size);
    APInt BSizeInt(BitWidth, V2Size);

    // Compute the difference between the two pointers.
    const SCEV *BA = SE->getMinusSCEV(V2S, V1S);

    // Test whether the difference is known to be great enough that memory of
    // the given sizes don't overlap. This assumes that ASizeInt and BSizeInt
    // are non-zero, which is special-cased above.
    if (ASizeInt.ule(SE->getUnsignedRange(BA).getUnsignedMin()) &&
        (-BSizeInt).uge(SE->getUnsignedRange(BA).getUnsignedMax()))
      return AliasAnalysis::NoAlias;

    // Folding the subtraction while preserving range information can be tricky
    // (because of INT_MIN, etc.); if the prior test failed, swap AS and BS
    // and try again to see if things fold better that way.

    // Compute the difference between the two pointers.
    const SCEV *AB = SE->getMinusSCEV(V1S, V2S);

    // Test whether the difference is known to be great enough that memory of
    // the given sizes don't overlap. This assumes that ASizeInt and BSizeInt
    // are non-zero, which is special-cased above.
    if (BSizeInt.ule(SE->getUnsignedRange(AB).getUnsignedMin()) &&
        (-ASizeInt).uge(SE->getUnsignedRange(AB).getUnsignedMax()))
      return AliasAnalysis::NoAlias;
  }

  if (AA->isMustAlias(AliasAnalysis::Location(V1Ptr, V1Size),
                      AliasAnalysis::Location(V2Ptr, V2Size))) {
    assert(V1Offset != V2Offset
           && "MachineMemOperand with same offset should not reach here!");
    return AliasAnalysis::NoAlias;
  }

  // Cannot go any further, cause the AliasAnalysis is not offset aware.
  return AliasAnalysis::MayAlias;
}

int getLoopDepDist(bool SrcBeforeDest, int Distance){
  if (!SrcBeforeDest && (Distance == 0)) Distance = 1;

  assert(Distance >= 0 && "Do not create a dependence with diff small than 0!");
  return Distance;
}
}
