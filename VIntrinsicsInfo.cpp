//===- VIntrinsicsInfo.cpp - Intrinsic Information -00-------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
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
#include "llvm/CodeGen/ValueTypes.h "
#include "llvm/Support/raw_ostream.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/StringExtras.h"
#include <cstring>

namespace llvm {
  namespace vtmIntrinsic {
#define GET_LLVM_INTRINSIC_FOR_GCC_BUILTIN
#include "VGenIntrinsics.inc"
#undef GET_LLVM_INTRINSIC_FOR_GCC_BUILTIN
  }
}

using namespace llvm;

std::string VIntrinsicInfo::getName(unsigned IntrID, const Type **Tys,
                                    unsigned numTys) const {
    static const char *const names[] = {
#define GET_INTRINSIC_NAME_TABLE
#include "VGenIntrinsics.inc"
#undef GET_INTRINSIC_NAME_TABLE
    };

  if (IntrID < Intrinsic::num_intrinsics)
    return 0;
  assert(IntrID < vtmIntrinsic::num_vtm_intrinsics && "Invalid intrinsic ID");

  std::string Result(names[IntrID - Intrinsic::num_intrinsics]);

  if (numTys == 0) return Result;

  for (unsigned i = 0; i < numTys; ++i) {
    if (const PointerType* PTyp = dyn_cast<PointerType>(Tys[i])) {
      Result += ".p" + llvm::utostr(PTyp->getAddressSpace());
      Result += EVT::getEVT(PTyp->getElementType()).getEVTString();
    } else if (Tys[i])
      Result += "." + EVT::getEVT(Tys[i]).getEVTString();
  }

  return Result;
}

unsigned VIntrinsicInfo::lookupName(const char *Name, unsigned Len) const {
    if (Len < 5 || Name[4] != '.' || Name[0] != 'l' || Name[1] != 'l'
      || Name[2] != 'v' || Name[3] != 'm')
      return 0;  // All intrinsics start with 'llvm.'

#define GET_FUNCTION_RECOGNIZER
#include "VGenIntrinsics.inc"
#undef GET_FUNCTION_RECOGNIZER
    return 0;
}

unsigned VIntrinsicInfo::
  lookupGCCName(const char *Name) const {
    return vtmIntrinsic::getIntrinsicForGCCBuiltin("vtm",Name);
}

bool VIntrinsicInfo::isOverloaded(unsigned IntrID) const {
  // Overload Table
  const bool OTable[] = {
#define GET_INTRINSIC_OVERLOAD_TABLE
#include "VGenIntrinsics.inc"
#undef GET_INTRINSIC_OVERLOAD_TABLE
  };
  if (IntrID == 0)
    return false;
  else
    return OTable[IntrID - Intrinsic::num_intrinsics];
}

/// This defines the "getAttributes(ID id)" method.
#define GET_INTRINSIC_ATTRIBUTES
#include "VGenIntrinsics.inc"
#undef GET_INTRINSIC_ATTRIBUTES

static const FunctionType *getType(LLVMContext &Context, unsigned id,
                                   const Type **Tys, unsigned numTys) {
  const Type *ResultTy = NULL;
  std::vector<const Type*> ArgTys;
  bool IsVarArg = false;

#define GET_INTRINSIC_GENERATOR
#include "VGenIntrinsics.inc"
#undef GET_INTRINSIC_GENERATOR

  return FunctionType::get(ResultTy, ArgTys, IsVarArg);
}

Function *VIntrinsicInfo::getDeclaration(Module *M, unsigned IntrID,
                                         const Type **Tys, unsigned numTys) const {
  AttrListPtr AList = getAttributes((vtmIntrinsic::ID) IntrID);

  return cast<Function>(
    M->getOrInsertFunction(getName(IntrID, Tys, numTys),
                           getType(M->getContext(), IntrID, Tys, numTys),
                           AList));
}
