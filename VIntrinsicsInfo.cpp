//===- VIntrinsicsInfo.cpp - Intrinsic Information -00-------*- C++ -*-===//
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
// This file contains the VTM implementation of TargetIntrinsicInfo.
//
//===----------------------------------------------------------------------===//

#include "VIntrinsicsInfo.h"

#include "llvm/DerivedTypes.h"
#include "llvm/Function.h"
#include "llvm/Module.h"
#include "llvm/Type.h"
#include "llvm/CodeGen/ValueTypes.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/DataTypes.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/ADT/StringMap.h"
#include <cstring>

using namespace llvm;
namespace vtmIntrinsic {
//enum ID {
//  last_non_mblaze_intrinsic = Intrinsic::num_intrinsics-1,
//#define GET_INTRINSIC_ENUM_VALUES
//#include "VerilogBackendGenIntrinsics.inc"
//#undef GET_INTRINSIC_ENUM_VALUES
//  , num_vtm_intrinsics
//};
#define GET_LLVM_INTRINSIC_FOR_GCC_BUILTIN
#include "VerilogBackendGenIntrinsics.inc"
#undef GET_LLVM_INTRINSIC_FOR_GCC_BUILTIN
}

//===----------------------------------------------------------------------===//
static inline unsigned lookupNameHelper(const char *Name, unsigned Len) {
    if (Len < 5 || Name[4] != '.' || Name[0] != 'l' || Name[1] != 'l'
        || Name[2] != 'v' || Name[3] != 'm')
      return 0;  // All intrinsics start with 'llvm.'

#define GET_FUNCTION_RECOGNIZER
#include "VerilogBackendGenIntrinsics.inc"
#undef GET_FUNCTION_RECOGNIZER
    return 0;
}

//===----------------------------------------------------------------------===//
// IntrinsicInfo Implementation.
std::string VIntrinsicInfo::getName(unsigned IntrID, Type **Tys,
                                    unsigned numTys) const {
    static const char *const names[] = {
#define GET_INTRINSIC_NAME_TABLE
#include "VerilogBackendGenIntrinsics.inc"
#undef GET_INTRINSIC_NAME_TABLE
    };

  if (IntrID < Intrinsic::num_intrinsics)
    return 0;
  assert(IntrID < vtmIntrinsic::num_vtm_intrinsics && "Invalid intrinsic ID");

  std::string Result(names[IntrID - Intrinsic::num_intrinsics]);

  if (numTys == 0) return Result;

  for (unsigned i = 0; i < numTys; ++i) {
    if (PointerType* PTyp = dyn_cast<PointerType>(Tys[i])) {
      Result += ".p" + llvm::utostr(PTyp->getAddressSpace());
      Result += EVT::getEVT(PTyp->getElementType()).getEVTString();
    } else if (Tys[i])
      Result += "." + EVT::getEVT(Tys[i]).getEVTString();
  }

  return Result;
}

unsigned VIntrinsicInfo::lookupName(const char *Name, unsigned Len) const {
  return lookupNameHelper(Name, Len);
}

unsigned VIntrinsicInfo::lookupGCCName(const char *Name) const {
    return vtmIntrinsic::getIntrinsicForGCCBuiltin("vtm",Name);
}

bool VIntrinsicInfo::isOverloaded(unsigned IntrID) const {
  if (IntrID == 0)
    return false;

  unsigned id = IntrID - Intrinsic::num_intrinsics + 1;
#define GET_INTRINSIC_OVERLOAD_TABLE
#include "VerilogBackendGenIntrinsics.inc"
#undef GET_INTRINSIC_OVERLOAD_TABLE
}

/// This defines the "getAttributes(ID id)" method.
#define GET_INTRINSIC_ATTRIBUTES
#include "VerilogBackendGenIntrinsics.inc"
#undef GET_INTRINSIC_ATTRIBUTES

static FunctionType *getType(LLVMContext &Context, unsigned id,
                             ArrayRef<Type*> Tys) {
  Type *ResultTy = NULL;
  std::vector<Type*> ArgTys;
  bool IsVarArg = false;

#define GET_INTRINSIC_GENERATOR
#include "VerilogBackendGenIntrinsics.inc"
#undef GET_INTRINSIC_GENERATOR

  return FunctionType::get(ResultTy, ArgTys, IsVarArg);
}

Function *VIntrinsicInfo::getDeclaration(Module *M, unsigned IntrID,
                                         Type **Tys, unsigned numTys) const {
  AttrListPtr AList = getAttributes((vtmIntrinsic::ID)IntrID);

  return cast<Function>(
    M->getOrInsertFunction(getName(IntrID, Tys, numTys),
                           getType(M->getContext(), IntrID,
                           ArrayRef<Type*>(Tys, numTys)),
                           AList));
}
