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
  /// VIntrinsicInst - A useful wrapper class for inspecting calls to intrinsic
  /// functions.  This allows the standard isa/dyncast/cast functionality to
  /// work with calls to intrinsic functions.
  class VIntrinsicInst : public CallInst {
    VIntrinsicInst();                      // DO NOT IMPLEMENT
    VIntrinsicInst(const VIntrinsicInst&);  // DO NOT IMPLEMENT
    void operator=(const VIntrinsicInst&); // DO NOT IMPLEMENT
  public:
    /// getIntrinsicID - Return the intrinsic ID of this intrinsic.
    ///
    vtmIntrinsic::ID getIntrinsicID() const;

    // Methods for support type inquiry through isa, cast, and dyn_cast:
    static inline bool classof(const VIntrinsicInst *) { return true; }
    static bool classof(const CallInst *I);

    static inline bool classof(const Value *V) {
      return isa<CallInst>(V) && classof(cast<CallInst>(V));
    }
  };

  ///VAllocaBramInst - This represents the llvm.vtm.alloca.bram instruction.
  ///
  class VAllocaBRamInst : public VIntrinsicInst {
  public:
    // Methods for support type inquiry through isa, cast, and dyn_cast:
    static inline bool classof(const VAllocaBRamInst *) {return true;}
    static inline bool classof(const VIntrinsicInst *I) {
      return I->getIntrinsicID() == vtmIntrinsic::vtm_alloca_bram;
    }
    static inline bool classof(const Value *V) {
      return isa<VIntrinsicInst>(V) && classof(cast<VIntrinsicInst>(V));
    }

    Value *getPointerOperand() const;
    unsigned getBRamId() const;
    unsigned getNumElement() const;
    unsigned getElementSizeInBytes() const;
  };

  /// VAccessBramInst - This represents the llvm.vtm.access.bram instruction.
  ///
  class VAccessBRamInst : public VIntrinsicInst {
  public:
    // Methods for support type inquiry through isa, cast, and dyn_cast:
    static inline bool classof(const VAllocaBRamInst *) { return true; }
    static inline bool classof(const VIntrinsicInst *I) {
      return I->getIntrinsicID() == vtmIntrinsic::vtm_access_bram;
    }
    static inline bool classof(const Value *V) {
      return isa<VIntrinsicInst>(V) && classof(cast<VIntrinsicInst>(V));
    }

    unsigned getBRamId() const;
    Value *getPointerOperand() const;
    Value *getValueOperand() const;
    bool isStore() const;
    unsigned getAlignment() const;
    bool isVolatile() const;
  };

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
