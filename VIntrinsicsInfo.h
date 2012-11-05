//===- VTMIntrinsicInfo.h - VTM Intrinsic Information -----*- C++ -*-===//
//
//                      The Shang HLS frameowrk                               //
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

#include "vtm/VerilogBackendMCTargetDesc.h"

#include "llvm/IntrinsicInst.h"
#include "llvm/Target/TargetIntrinsicInfo.h"

namespace llvm {
  class VIntrinsicInfo : public TargetIntrinsicInfo {
  public:
    std::string getName(unsigned IntrID, Type **Tys = 0,
                        unsigned numTys = 0) const;

    unsigned lookupName(const char *Name, unsigned Len) const;

    unsigned lookupGCCName(const char *Name) const;

    bool isOverloaded(unsigned IID) const;

    Function *getDeclaration(Module *M, unsigned ID, Type **Tys = 0,
                             unsigned numTys = 0) const;
  };

}

#endif
