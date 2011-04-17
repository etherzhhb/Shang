//===- VTMIntrinsicInfo.h - VTM Intrinsic Information -----*- C++ -*-===//
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
#ifndef VTM_INTRINSICS_H
#define VTM_INTRINSICS_H

#include "llvm/Intrinsics.h"
#include "llvm/Target/TargetIntrinsicInfo.h"

namespace llvm {
  namespace vtmIntrinsic {
    enum ID {
      last_non_vtm_intrinsic = Intrinsic::num_intrinsics - 1,
#define GET_INTRINSIC_ENUM_VALUES
#include "VGenIntrinsics.inc"
#undef GET_INTRINSIC_ENUM_VALUES
      , num_vtm_intrinsics
    };
  }
  // TODO: IntrinsicInst for VTM.

  class VIntrinsicInfo : public TargetIntrinsicInfo {
  public:
    std::string getName(unsigned IntrID, const Type **Tys = 0,
                        unsigned numTys = 0) const;
  
    unsigned lookupName(const char *Name, unsigned Len) const;
  
    unsigned lookupGCCName(const char *Name) const;
  
    bool isOverloaded(unsigned IID) const;
  
    Function *getDeclaration(Module *M, unsigned ID, const Type **Tys = 0,
                             unsigned numTys = 0) const;
  };

}

#endif
