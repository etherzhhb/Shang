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

namespace llvm {
class LoopInfo;
}
using namespace llvm;

namespace esyn {

enum HWAtomTypes {
  atomSignedPrefix,   // Represent the Signed marker, use in Ashr
  atomWireOp,        // Trunc, Z/SExt, PtrToInt, IntToPtr
                      // Data communication atom
  atomRegister,       // Assign value to register, use in infinite scheduler
                      // Schedeable atoms
                      // Operate resource
                      // on pre-allocate resource,
  atomOpPreAllRes,    // operate infinite resource 
  atomOpPostAllRes,   // on post-allocated limited resource
                      // all infinite resource is pre-allocate
  atomStateBegin,     // state transfer
  atomStateEnd,
  atomEmitNextLoop
};

/// @brief Base Class of all hardware atom. 
class HWAtom : public FoldingSetNode {
  /// FastID - A reference to an Interned FoldingSetNodeID for this node.
  /// The ScalarEvolution's BumpPtrAllocator holds the data.
  FoldingSetNodeIDRef FastID;

  // The HWAtom baseclass this node corresponds to
  const unsigned short HWAtomType;

  /// First of all, we schedule all atom base on dependence
  HWAtom *const *Deps;
  size_t NumDeps;

private:
  HWAtom(const HWAtom &);            // DO NOT IMPLEMENT
  void operator=(const HWAtom &);  // DO NOT IMPLEMENT

protected:
  // The corresponding LLVM Instruction
  Value &Val;

  virtual ~HWAtom();
public:
  explicit HWAtom(const FoldingSetNodeIDRef ID,
    unsigned HWAtomTy, Value &V, 
    HWAtom *const *deps, size_t numDeps) :
  FastID(ID), HWAtomType(HWAtomTy), Val(V), Deps(deps), NumDeps(numDeps) {}

  unsigned getHWAtomType() const { return HWAtomType; }

  /// Profile - FoldingSet support.
  void Profile(FoldingSetNodeID& ID) { ID = FastID; }

  Value &getValue() const { return Val; }

  /// @name Dependences
  //{
  size_t getNumDeps() const { return NumDeps; }
  HWAtom *getDep(unsigned i) const {
    assert(i < NumDeps && "Operand index out of range!");
    return Deps[i];
  }

  typedef HWAtom *const *dep_iterator;
  dep_iterator dep_begin() { return Deps; }
  dep_iterator dep_end() { return Deps + NumDeps; }

  typedef const HWAtom *const *const_dep_iterator;
  const_dep_iterator dep_begin() const { return Deps; }
  const_dep_iterator dep_end() const { return Deps + NumDeps; }
  //}

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
protected:
  explicit HWAInline(const FoldingSetNodeIDRef ID, enum HWAtomTypes T,
    Value &V, HWAtom *const *O) : HWAtom(ID, T, V, O, 1) {}
public:

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWAInline *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomSignedPrefix ||
      A->getHWAtomType() == atomWireOp ||
      A->getHWAtomType() == atomRegister;
  }
};

/// @brief The signed wire marker
class HWASigned : public HWAInline {
public:
  explicit HWASigned(const FoldingSetNodeIDRef ID, HWAtom *const *O)
    : HWAInline(ID, atomSignedPrefix, O[0]->getValue(), O){}

  void print(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWASigned *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomSignedPrefix;
  }
};

/// @brief The register atom will break the WAR dependence
class HWARegister : public HWAInline {
public:
  explicit HWARegister(const FoldingSetNodeIDRef ID, Instruction &I,
    HWAtom *const *O) 
    : HWAInline(ID, atomRegister, I, O) {}

  void print(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWARegister *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomRegister;
  } 
};

class HWAWireOp : public HWAInline {
public:
  explicit HWAWireOp(const FoldingSetNodeIDRef ID, Instruction &Inst,
    HWAtom *const *O) : HWAInline(ID, atomWireOp, Inst, O) {}

  void print(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWAWireOp *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomWireOp;
  }
};

class HWAStateTrans : public HWAtom {
public:
  explicit HWAStateTrans(const FoldingSetNodeIDRef ID, enum HWAtomTypes T,
    Value &Val, HWAtom *const *deps, size_t numDep)
    : HWAtom(ID, T, Val, deps, numDep) {}

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWAStateTrans *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomStateBegin ||
           A->getHWAtomType() == atomStateEnd;
  }
};

class HWAStateBegin : public HWAStateTrans {
  typedef std::vector<HWAtom*> HWAtomVecType;

  // Atoms in this state
  HWAtomVecType Atoms;
public:
  explicit HWAStateBegin(const FoldingSetNodeIDRef ID, BasicBlock &BB)
    : HWAStateTrans(ID, atomStateBegin, BB, 0, 0) {}

  void addNewAtom(HWAtom *Atom) { Atoms.push_back(Atom); }

  typedef HWAtomVecType::iterator iterator;
  iterator begin() { return Atoms.begin(); }
  iterator end() { return Atoms.end(); }

  typedef HWAtomVecType::const_iterator const_iterator;
  const_iterator begin() const { return Atoms.begin(); }
  const_iterator end() const { return Atoms.end(); }

  void print(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWAStateBegin *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomStateBegin;
  }
};

class HWAStateEnd : public HWAStateTrans {
public:
  explicit HWAStateEnd(const FoldingSetNodeIDRef ID,
    TerminatorInst &Term, HWAtom *const *deps, size_t numDep)
    : HWAStateTrans(ID, atomStateEnd, Term, deps, numDep) {}

  void print(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWAStateEnd *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomStateEnd;
  }
};


/// @brief The Schedulable Hardware Atom
class HWASchedable : public HWAtom {
  // the scheduled slot is manage by the scheduler
  // unsigned slot

protected:
  explicit HWASchedable(const FoldingSetNodeIDRef ID, enum HWAtomTypes T,
    Instruction &Inst, HWAtom *const *deps, size_t numDep)
    : HWAtom(ID, T, Inst, deps, numDep) {}
public:

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWASchedable *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomOpPreAllRes ||
      A->getHWAtomType() == atomOpPostAllRes ||
      A->getHWAtomType() == atomEmitNextLoop;
  }
};

class HWAEmitNextLoop : public HWASchedable {
public:
  explicit HWAEmitNextLoop(const FoldingSetNodeIDRef ID, enum HWAtomTypes T,
    Instruction &Inst, HWAtom *const *deps, size_t numDep)
    : HWASchedable(ID, T, Inst, deps, numDep) {}

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWAEmitNextLoop *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomEmitNextLoop;
  }
};

class HWAOpRes : public HWASchedable {

  // The resoure that this atom using.
  HWResource &UsingRes;

protected:
  explicit HWAOpRes(const FoldingSetNodeIDRef ID, enum HWAtomTypes T,
    Instruction &Inst, HWAtom *const *deps, size_t numDep, HWResource &Res)
    : HWASchedable(ID, T, Inst, deps, numDep), UsingRes(Res) {}
public:
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
  static inline bool classof(const HWAOpRes *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomOpPostAllRes ||
      A->getHWAtomType() == atomOpPreAllRes;
  }
};

/// @brief The atom that operates post-allocate resources
class HWAOpPostAllRes : public HWAOpRes {
public:
  explicit HWAOpPostAllRes(const FoldingSetNodeIDRef ID,
    Instruction &Inst, HWAtom *const *deps, size_t numDep, HWResource &Res)
    : HWAOpRes(ID, atomOpPostAllRes, Inst, deps, numDep, Res) {}

  void print(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWAOpPostAllRes *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomOpPostAllRes;
  }
};

/// @brief The atom that operate in infinite resource
class HWAOpPreAllRes : public HWAOpRes {
public:
  explicit HWAOpPreAllRes(const FoldingSetNodeIDRef ID,
    Instruction &Inst, HWAtom *const *deps, size_t numDep, HWResource &Res)
    : HWAOpRes(ID, atomOpPreAllRes, Inst, deps, numDep, Res) {}

  void print(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWAOpPreAllRes *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomOpPreAllRes;
  }
};

/// @brief Hardware atom construction pass
///
class HWAtomInfo : public FunctionPass, public InstVisitor<HWAtomInfo> {

  /// @name InstVisitor interface
  //{
  friend class InstVisitor<HWAtomInfo>;
  void visitTerminatorInst(TerminatorInst &I);

  void visitReturnInst(ReturnInst &I) {
    visitTerminatorInst(I);
  }
  void visitBranchInst(BranchInst &I) {
    visitTerminatorInst(I);
  }
  void visitSwitchInst(SwitchInst &I){
    llvm_unreachable("Instruction not support yet!");
  }
  void visitIndirectBrInst(IndirectBrInst &I){
    llvm_unreachable("Instruction not support yet!");
  }
  void visitInvokeInst(InvokeInst &I) {
    llvm_unreachable("HWAtomFilterPass pass didn't work!");
  }

  void visitUnwindInst(UnwindInst &I) {
    llvm_unreachable("HWAtomFilterPass pass didn't work!");
  }
  void visitUnreachableInst(UnreachableInst &I){}

  void visitPHINode(PHINode &I);
  void visitBinaryOperator(Instruction &I);
  void visitICmpInst(ICmpInst &I);
  void visitFCmpInst(FCmpInst &I){
    llvm_unreachable("Instruction not support yet!");
  }

  void visitCastInst (CastInst &I);
  void visitSelectInst(SelectInst &I);
  void visitCallInst (CallInst &I){
    llvm_unreachable("Instruction not support yet!");}
  void visitInlineAsm(CallInst &I){
    llvm_unreachable("Instruction not support yet!");
  }
  bool visitBuiltinCall(CallInst &I, Intrinsic::ID ID, bool &WroteCallee) {
    llvm_unreachable("Instruction not support yet!");
  }

  void visitAllocaInst(AllocaInst &I) {
    llvm_unreachable("Instruction not support yet!");
  }
  void visitLoadInst  (LoadInst   &I);
  void visitStoreInst (StoreInst  &I);
  void visitGetElementPtrInst(GetElementPtrInst &I);
  void visitVAArgInst (VAArgInst &I){
    llvm_unreachable("Instruction not support yet!");
  }

  void visitInsertElementInst(InsertElementInst &I){
    llvm_unreachable("Instruction not support yet!");
  }
  void visitExtractElementInst(ExtractElementInst &I){
    llvm_unreachable("Instruction not support yet!");
  }
  void visitShuffleVectorInst(ShuffleVectorInst &SVI){
    llvm_unreachable("Instruction not support yet!");
  }

  void visitInsertValueInst(InsertValueInst &I){
    llvm_unreachable("Instruction not support yet!");
  }
  void visitExtractValueInst(ExtractValueInst &I){
    llvm_unreachable("Instruction not support yet!");
  }

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
  
  HWAStateBegin *getStateBegin(BasicBlock &BB);

  HWAStateEnd *getStateEnd(TerminatorInst &Term,
                           SmallVectorImpl<HWAtom*> &Deps);

  HWARegister *getRegister(Instruction &I, HWAtom *Using);

  HWASigned *getSigned(HWAtom *Using);

  HWAWireOp *getWireOp(Instruction &I, HWAtom *Using);

  HWAOpRes *getOpRes(Instruction &I,
                            SmallVectorImpl<HWAtom*> &Deps,
                            HWResource &Res) {   
    if (Res.isInfinite())
      return getOpPreAllRes(I, Deps, Res);
    else
      return getOpPostAllRes(I, Deps, Res);
  }

  HWAOpPostAllRes *getOpPostAllRes(Instruction &I,
                                   SmallVectorImpl<HWAtom*> &Deps,
                                   HWResource &Res);

  HWAOpPreAllRes *getOpPreAllRes(Instruction &I,
                                 SmallVectorImpl<HWAtom*> &Deps,
                                 HWResource &Res);


  // Maping Instruction to HWAtoms
  typedef DenseMap<const Instruction*, HWAtom*> AtomMapType;
  AtomMapType InstToHWAtoms;



  HWAtom *getAtomFor(Instruction &Inst) const {
    AtomMapType::const_iterator At = InstToHWAtoms.find(&Inst);
    assert(At != InstToHWAtoms.end() && "Can not get the Atom!");
    return  At->second;
  }

  typedef DenseMap<const BasicBlock*, HWAStateBegin*> StateMapType;
  StateMapType BBToStates;
  HWAtom *ControlRoot;
  HWAStateBegin *CurState;

  HWAStateBegin *getCurState() {
    return CurState;
  }

  void updateStateTo(BasicBlock &BB) {
    CurState = getStateBegin(BB);
    BBToStates.insert(
      std::make_pair<const BasicBlock*, HWAStateBegin*>(&BB, CurState));
    SetControlRoot(CurState);
  }

  HWAtom *getControlRoot() {
    return ControlRoot;
  }

  HWAtom *getAtomInState(Instruction &Inst, BasicBlock *BB) {
    if (BB == Inst.getParent())
      return getAtomFor(Inst);
    else
      // Create the register
      return getRegister(Inst, CurState);
  }
  void addOperandDeps(Instruction &I, SmallVectorImpl<HWAtom*> &Deps) {
    BasicBlock *ParentBB = I.getParent();
    HWAStateBegin *CurState = getStateFor(*ParentBB);
    for (ReturnInst::op_iterator OI = I.op_begin(), OE = I.op_end();
        OI != OE; ++OI) {
      if (Instruction *OpI = dyn_cast<Instruction>(OI))
        // Restrict the dependences in the current state
        Deps.push_back(getAtomInState(*OpI, ParentBB));
    }
  }

  void SetControlRoot(HWAtom *NewRoot) {
    ControlRoot = NewRoot;
  }

  HWAStateBegin *getStateFor(BasicBlock &BB) const {
    StateMapType::const_iterator At = BBToStates.find(&BB);
    assert(At != BBToStates.end() && "Can not get the State!");
    return  At->second;
  }

  void clear();

  // The loop Info
  LoopInfo *LI;
  ResourceConfig *RC;
public:

  /// @name FunctionPass interface
  //{
  static char ID;
  HWAtomInfo() : FunctionPass(&ID), ControlRoot(0), CurState(0), LI(0), RC(0) {}

  bool runOnFunction(Function &F);
  void releaseMemory();
  void getAnalysisUsage(AnalysisUsage &AU) const;
  virtual void print(raw_ostream &O, const Module *M) const;
  //}
};

} // end namespace

#endif