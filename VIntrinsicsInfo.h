//===- VTMIntrinsicInfo.h - VTM Intrinsic Information -----*- C++ -*-===//
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
#ifndef VTM_INTRINSICS_H
#define VTM_INTRINSICS_H

#include "vtm/VerilgoBackendMCTargetDesc.h"

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
