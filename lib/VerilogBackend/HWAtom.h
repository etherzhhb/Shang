//===------------- HWAtom.h - Translate LLVM IR to HWAtom  -------*- C++ -*-===//
//
//                            The Verilog Backend
//
// Copyright: 2010 by Hongbin Zheng. all rights reserved.
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
// This file define the HWAtom class, which represent the basic atom operation
// in hardware.
// This file also define the HWAtomInfo class, which construction the HWAtom
// represent from LLVM IR.
//
//===----------------------------------------------------------------------===//
//

#ifndef VBE_HARDWARE_ATOM_H
#define VBE_HARDWARE_ATOM_H

#include "llvm/Pass.h"
#include "llvm/Function.h"
#include "llvm/Instructions.h"
#include "llvm/Intrinsics.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/FoldingSet.h"
#include "llvm/Support/Allocator.h"
#include "llvm/Support/InstVisitor.h"
#include "llvm/Support/raw_os_ostream.h"
#include "llvm/Support/ErrorHandling.h"

#include "vbe/ResourceConfig.h"

using namespace llvm;

namespace esyn {


enum HWAtomTypes {
  atomSignedPrefix,   // Represent the Signed marker, use in Ashr
  atomBitCast,        // Trunc, Z/SExt, PtrToInt, IntToPtr
                      // Data communication atom
  atomAssignRegister, // Assign value to register, use in infinite scheduler
                      // Schedeable atoms
                      // Operate resource
                      // on pre-allocate resource,
  atomOpPreAllRes,    // operate infinite resource 
  atomOpPostAllRes    // on post-allocated limited resource
                      // all infinite resource is pre-allocate
};

/// @brief Base Class of all hardware atom. 
class HWAtom : public FoldingSetNode {
  /// FastID - A reference to an Interned FoldingSetNodeID for this node.
  /// The ScalarEvolution's BumpPtrAllocator holds the data.
  FoldingSetNodeIDRef FastID;

  // The HWAtom baseclass this node corresponds to
  const unsigned short HWAtomType;

private:
  HWAtom(const HWAtom &);            // DO NOT IMPLEMENT
  void operator=(const HWAtom &);  // DO NOT IMPLEMENT

protected:
  // The corresponding LLVM Instruction
  Instruction &Inst;

  virtual ~HWAtom();
public:
  explicit HWAtom(const FoldingSetNodeIDRef ID,
    unsigned HWAtomTy, Instruction &I) :
  FastID(ID), HWAtomType(HWAtomTy), Inst(I) {}

  unsigned getHWAtomType() const { return HWAtomType; }

  /// Profile - FoldingSet support.
  void Profile(FoldingSetNodeID& ID) { ID = FastID; }

  Instruction &getInst() const { return Inst; }

  /// print - Print out the internal representation of this atom to the
  /// specified stream.  This should really only be used for debugging
  /// purposes.
  virtual void print(raw_ostream &OS) const = 0;

  /// dump - This method is used for debugging.
  ///
  void dump() const;
};

/// @brief Inline operation
class HWAInline : public HWAtom {
  HWAtom *Op;
protected:
  explicit HWAInline(const FoldingSetNodeIDRef ID, enum HWAtomTypes T,
    HWAtom *O) : HWAtom(ID, T, O->getInst()), Op(O) {}
public:
  HWAtom *getOperand() const { return Op; }

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWAInline *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomSignedPrefix || atomBitCast
      || atomAssignRegister;
  }
};

/// @brief The signed wire marker
class HWASigned : public HWAInline {
public:
  explicit HWASigned(const FoldingSetNodeIDRef ID, HWAtom *O)
    : HWAInline(ID, atomSignedPrefix, O){}

  void print(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWASigned *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomSignedPrefix;
  }
};

/// @brief The register atom will break the WAR dependence
class HWARegeister : public HWAInline {
public:
  explicit HWARegeister(const FoldingSetNodeIDRef ID, HWAtom *O)
    : HWAInline(ID, atomAssignRegister, O) {}

  void print(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWARegeister *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomAssignRegister;
  } 
};

class HWABitCast : public HWAInline {
  Instruction &CastInst;
public:
  explicit HWABitCast(const FoldingSetNodeIDRef ID, HWAtom *O,
    Instruction &Inst)
    : HWAInline(ID, atomBitCast, O), CastInst(Inst) {}

  void print(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWABitCast *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomBitCast;
  }
};

/// @brief The Schedulable Hardware Atom
class HWASchedable : public HWAtom {
  /// First of all, we schedule all atom base on dependence
  HWAtom *const *Deps;
  size_t NumDeps;

  // the scheduled slot is manage by the scheduler
  // unsigned slot

  // The resoure that this atom using.
  HWResource &UsingRes;

protected:
  explicit HWASchedable(const FoldingSetNodeIDRef ID, enum HWAtomTypes T,
     Instruction &Inst, HWAtom *const *deps, size_t numDep, HWResource &Res)
    : HWAtom(ID, T, Inst), Deps(deps), NumDeps(numDep), UsingRes(Res) {}
public:
  /// @name Dependences
  //{
  size_t getNumDeps() const { return NumDeps; }
  HWAtom *getDep(unsigned i) const {
    assert(i < NumDeps && "Operand index out of range!");
    return Deps[i];
  }

  typedef HWAtom *const *op_iterator;
  op_iterator dep_begin() { return Deps; }
  op_iterator dep_end() { return Deps + NumDeps; }
  //}

  /// @name The using resource
  //{
  HWResource &getUsingResource() const {
    return UsingRes;
  }
  // Get the latancy of this atom
  unsigned getLatency() const {
    return UsingRes.getLatency();
  }
  //}

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWASchedable *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() ==
      atomOpPreAllRes || atomOpPostAllRes;
  }
};

/// @brief The atom that operates post-allocate resources
class HWAOpPostAllRes : public HWASchedable {
public:
  explicit HWAOpPostAllRes(const FoldingSetNodeIDRef ID,
    Instruction &Inst, HWAtom *const *deps, size_t numDep, HWResource &Res)
    : HWASchedable(ID, atomOpPostAllRes, Inst, deps, numDep, Res) {}
};

/// @brief The atom that operate in infinite resource
class HWAOpPreAllRes : public HWASchedable {
public:
  explicit HWAOpPreAllRes(const FoldingSetNodeIDRef ID,
    Instruction &Inst, HWAtom *const *deps, size_t numDep, HWResource &Res)
    : HWASchedable(ID, atomOpPreAllRes, Inst, deps, numDep, Res) {}
};

/// @brief Hardware atom construction pass
///
class HWAtomInfo : public FunctionPass, public InstVisitor<HWAtomInfo> {

  /// @name InstVisitor interface
  //{
  friend class InstVisitor<HWAtomInfo>;
  void visitReturnInst(ReturnInst &I){}
  void visitBranchInst(BranchInst &I){}
  void visitSwitchInst(SwitchInst &I){}
  void visitIndirectBrInst(IndirectBrInst &I){}
  void visitInvokeInst(InvokeInst &I) {
    llvm_unreachable("HWAtomFilterPass pass didn't work!");
  }

  void visitUnwindInst(UnwindInst &I) {
    llvm_unreachable("HWAtomFilterPass pass didn't work!");
  }
  void visitUnreachableInst(UnreachableInst &I){}

  void visitPHINode(PHINode &I){}
  void visitBinaryOperator(Instruction &I){}
  void visitICmpInst(ICmpInst &I){}
  void visitFCmpInst(FCmpInst &I){}

  void visitCastInst (CastInst &I){}
  void visitSelectInst(SelectInst &I){}
  void visitCallInst (CallInst &I){}
  void visitInlineAsm(CallInst &I){}
  bool visitBuiltinCall(CallInst &I, Intrinsic::ID ID, bool &WroteCallee){}

  void visitAllocaInst(AllocaInst &I){}
  void visitLoadInst  (LoadInst   &I){}
  void visitStoreInst (StoreInst  &I){}
  void visitGetElementPtrInst(GetElementPtrInst &I){}
  void visitVAArgInst (VAArgInst &I){}

  void visitInsertElementInst(InsertElementInst &I){}
  void visitExtractElementInst(ExtractElementInst &I){}
  void visitShuffleVectorInst(ShuffleVectorInst &SVI){}

  void visitInsertValueInst(InsertValueInst &I){}
  void visitExtractValueInst(ExtractValueInst &I){}

  void visitInstruction(Instruction &I) {
#ifndef NDEBUG
    errs() << "HWAtomInfo does not know about " << I;
#endif
    llvm_unreachable(0);
  }
  //}

  // Allocator
  BumpPtrAllocator HWAtomAllocator;

  // 
  FoldingSet<HWAtom> UniqiueHWAtoms;
  
  // Maping Instruction to HWAtoms
  typedef std::vector<HWAtom*> HWAtomVecType;
  typedef DenseMap<const Instruction*, HWAtomVecType> AtomMapType;
  AtomMapType InstToHWAtoms;
  

  void clear();
public:

  /// @name FunctionPass interface
  //{
  static char ID;
  HWAtomInfo() : FunctionPass(&ID) {}

  bool runOnFunction(Function &F);
  void releaseMemory();
  void getAnalysisUsage(AnalysisUsage &) const;
  virtual void print(raw_ostream &O, const Module *M) const;
  //}
};

} // end namespace

#endif