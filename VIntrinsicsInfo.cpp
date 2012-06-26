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

#include "llvm/Intrinsics.h"
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

namespace {
/// IIT_Info - These are enumerators that describe the entries returned by the
/// getIntrinsicInfoTableEntries function.
///
/// NOTE: This must be kept in synch with the copy in TblGen/IntrinsicEmitter!
enum IIT_Info {
  // Common values should be encoded with 0-15.
  IIT_Done = 0,
  IIT_I1   = 1,
  IIT_I8   = 2,
  IIT_I16  = 3,
  IIT_I32  = 4,
  IIT_I64  = 5,
  IIT_F32  = 6,
  IIT_F64  = 7,
  IIT_V2   = 8,
  IIT_V4   = 9,
  IIT_V8   = 10,
  IIT_V16  = 11,
  IIT_V32  = 12,
  IIT_MMX  = 13,
  IIT_PTR  = 14,
  IIT_ARG  = 15,

  // Values from 16+ are only encodable with the inefficient encoding.
  IIT_METADATA = 16,
  IIT_EMPTYSTRUCT = 17,
  IIT_STRUCT2 = 18,
  IIT_STRUCT3 = 19,
  IIT_STRUCT4 = 20,
  IIT_STRUCT5 = 21,
  IIT_EXTEND_VEC_ARG = 22,
  IIT_TRUNC_VEC_ARG = 23,
  IIT_ANYPTR = 24
};

#define GET_INTRINSIC_GENERATOR_GLOBAL
#include "VerilogBackendGenIntrinsics.inc"
#undef GET_INTRINSIC_GENERATOR_GLOBAL
}

static void DecodeIITType(unsigned &NextElt, ArrayRef<unsigned char> Infos,
                      SmallVectorImpl<Intrinsic::IITDescriptor> &OutputTable) {
  IIT_Info Info = IIT_Info(Infos[NextElt++]);
  unsigned StructElts = 2;
  using namespace Intrinsic;

  switch (Info) {
  case IIT_Done:
    OutputTable.push_back(IITDescriptor::get(IITDescriptor::Void, 0));
    return;
  case IIT_MMX:
    OutputTable.push_back(IITDescriptor::get(IITDescriptor::MMX, 0));
    return;
  case IIT_METADATA:
    OutputTable.push_back(IITDescriptor::get(IITDescriptor::Metadata, 0));
    return;
  case IIT_F32:
    OutputTable.push_back(IITDescriptor::get(IITDescriptor::Float, 0));
    return;
  case IIT_F64:
    OutputTable.push_back(IITDescriptor::get(IITDescriptor::Double, 0));
    return;
  case IIT_I1:
    OutputTable.push_back(IITDescriptor::get(IITDescriptor::Integer, 1));
    return;
  case IIT_I8:
    OutputTable.push_back(IITDescriptor::get(IITDescriptor::Integer, 8));
    return;
  case IIT_I16:
    OutputTable.push_back(IITDescriptor::get(IITDescriptor::Integer,16));
    return;
  case IIT_I32:
    OutputTable.push_back(IITDescriptor::get(IITDescriptor::Integer, 32));
    return;
  case IIT_I64:
    OutputTable.push_back(IITDescriptor::get(IITDescriptor::Integer, 64));
    return;
  case IIT_V2:
    OutputTable.push_back(IITDescriptor::get(IITDescriptor::Vector, 2));
    DecodeIITType(NextElt, Infos, OutputTable);
    return;
  case IIT_V4:
    OutputTable.push_back(IITDescriptor::get(IITDescriptor::Vector, 4));
    DecodeIITType(NextElt, Infos, OutputTable);
    return;
  case IIT_V8:
    OutputTable.push_back(IITDescriptor::get(IITDescriptor::Vector, 8));
    DecodeIITType(NextElt, Infos, OutputTable);
    return;
  case IIT_V16:
    OutputTable.push_back(IITDescriptor::get(IITDescriptor::Vector, 16));
    DecodeIITType(NextElt, Infos, OutputTable);
    return;
  case IIT_V32:
    OutputTable.push_back(IITDescriptor::get(IITDescriptor::Vector, 32));
    DecodeIITType(NextElt, Infos, OutputTable);
    return;
  case IIT_PTR:
    OutputTable.push_back(IITDescriptor::get(IITDescriptor::Pointer, 0));
    DecodeIITType(NextElt, Infos, OutputTable);
    return;
  case IIT_ANYPTR: {  // [ANYPTR addrspace, subtype]
    OutputTable.push_back(IITDescriptor::get(IITDescriptor::Pointer,
                                             Infos[NextElt++]));
    DecodeIITType(NextElt, Infos, OutputTable);
    return;
  }
  case IIT_ARG: {
    unsigned ArgInfo = (NextElt == Infos.size() ? 0 : Infos[NextElt++]);
    OutputTable.push_back(IITDescriptor::get(IITDescriptor::Argument, ArgInfo));
    return;
  }
  case IIT_EXTEND_VEC_ARG: {
    unsigned ArgInfo = (NextElt == Infos.size() ? 0 : Infos[NextElt++]);
    OutputTable.push_back(IITDescriptor::get(IITDescriptor::ExtendVecArgument,
                                             ArgInfo));
    return;
  }
  case IIT_TRUNC_VEC_ARG: {
    unsigned ArgInfo = (NextElt == Infos.size() ? 0 : Infos[NextElt++]);
    OutputTable.push_back(IITDescriptor::get(IITDescriptor::TruncVecArgument,
                                             ArgInfo));
    return;
  }
  case IIT_EMPTYSTRUCT:
    OutputTable.push_back(IITDescriptor::get(IITDescriptor::Struct, 0));
    return;
  case IIT_STRUCT5: ++StructElts; // FALL THROUGH.
  case IIT_STRUCT4: ++StructElts; // FALL THROUGH.
  case IIT_STRUCT3: ++StructElts; // FALL THROUGH.
  case IIT_STRUCT2: {
    OutputTable.push_back(IITDescriptor::get(IITDescriptor::Struct,StructElts));

    for (unsigned i = 0; i != StructElts; ++i)
      DecodeIITType(NextElt, Infos, OutputTable);
    return;
  }
  }
  llvm_unreachable("unhandled");
}

static Type *DecodeFixedType(ArrayRef<Intrinsic::IITDescriptor> &Infos,
                             ArrayRef<Type*> Tys, LLVMContext &Context) {
  using namespace Intrinsic;
  IITDescriptor D = Infos.front();
  Infos = Infos.slice(1);

  switch (D.Kind) {
  case IITDescriptor::Void: return Type::getVoidTy(Context);
  case IITDescriptor::MMX: return Type::getX86_MMXTy(Context);
  case IITDescriptor::Metadata: return Type::getMetadataTy(Context);
  case IITDescriptor::Float: return Type::getFloatTy(Context);
  case IITDescriptor::Double: return Type::getDoubleTy(Context);

  case IITDescriptor::Integer:
    return IntegerType::get(Context, D.Integer_Width);
  case IITDescriptor::Vector:
    return VectorType::get(DecodeFixedType(Infos, Tys, Context),D.Vector_Width);
  case IITDescriptor::Pointer:
    return PointerType::get(DecodeFixedType(Infos, Tys, Context),
                            D.Pointer_AddressSpace);
  case IITDescriptor::Struct: {
    Type *Elts[5];
    assert(D.Struct_NumElements <= 5 && "Can't handle this yet");
    for (unsigned i = 0, e = D.Struct_NumElements; i != e; ++i)
      Elts[i] = DecodeFixedType(Infos, Tys, Context);
    return StructType::get(Context, ArrayRef<Type*>(Elts,D.Struct_NumElements));
  }

  case IITDescriptor::Argument:
    return Tys[D.getArgumentNumber()];
  case IITDescriptor::ExtendVecArgument:
    return VectorType::getExtendedElementVectorType(cast<VectorType>(
                                                  Tys[D.getArgumentNumber()]));

  case IITDescriptor::TruncVecArgument:
    return VectorType::getTruncatedElementVectorType(cast<VectorType>(
                                                  Tys[D.getArgumentNumber()]));
  }
  llvm_unreachable("unhandled");
}

static FunctionType *getType(LLVMContext &Context, unsigned id,
                             ArrayRef<Type*> Tys) {
  SmallVector<Intrinsic::IITDescriptor, 8> Table;

  // Check to see if the intrinsic's type was expressible by the table.
  unsigned TableVal = IIT_Table[id - Intrinsic::num_intrinsics];

  // Decode the TableVal into an array of IITValues.
  SmallVector<unsigned char, 8> IITValues;
  ArrayRef<unsigned char> IITEntries;
  unsigned NextElt = 0;
  if ((TableVal >> 31) != 0) {
    // This is an offset into the IIT_LongEncodingTable.
    IITEntries = IIT_LongEncodingTable;

    // Strip sentinel bit.
    NextElt = (TableVal << 1) >> 1;
  } else {
    // Decode the TableVal into an array of IITValues.  If the entry was encoded
    // into a single word in the table itself, decode it now.
    do {
      IITValues.push_back(TableVal & 0xF);
      TableVal >>= 4;
    } while (TableVal);

    IITEntries = IITValues;
    NextElt = 0;
  }

  // Okay, decode the table into the output vector of IITDescriptors.
  DecodeIITType(NextElt, IITEntries, Table);
  while (NextElt != IITEntries.size() && IITEntries[NextElt] != 0)
    DecodeIITType(NextElt, IITEntries, Table);

  ArrayRef<Intrinsic::IITDescriptor> TableRef = Table;
  Type *ResultTy = DecodeFixedType(TableRef, Tys, Context);

  SmallVector<Type*, 8> ArgTys;
  while (!TableRef.empty())
    ArgTys.push_back(DecodeFixedType(TableRef, Tys, Context));

  return FunctionType::get(ResultTy, ArgTys, false);
}

Function *VIntrinsicInfo::getDeclaration(Module *M, unsigned IntrID,
                                         Type **Tys, unsigned numTys) const {
  AttrListPtr AList = getAttributes((vtmIntrinsic::ID)IntrID);

  FunctionType *FuncTy = getType(M->getContext(), IntrID,
                                 ArrayRef<Type*>(Tys, numTys));
  return cast<Function>(M->getOrInsertFunction(getName(IntrID, Tys, numTys),
                                               FuncTy, AList));
}
