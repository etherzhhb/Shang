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
  atomWireOp,         // Trunc, Z/SExt, PtrToInt, IntToPtr
                      // Data communication atom
  atomRegister,       // Assign value to register, use in infinite scheduler
                      // Schedeable atoms
  atomOpRes,          // Operate shared resource
  atomOpInst,         // Operate on inline instruction
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

  // The time slot that this atom scheduled to.
  unsigned SchedSlot;

  virtual ~HWAtom();

public:
  explicit HWAtom(const FoldingSetNodeIDRef ID,
    unsigned HWAtomTy, Value &V, 
    HWAtom *const *deps, size_t numDeps) 
    : FastID(ID), HWAtomType(HWAtomTy), Val(V), Deps(deps),
    NumDeps(numDeps), SchedSlot(UINT32_MAX >> 1) {}

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

  virtual void reset() { SchedSlot = UINT32_MAX  >> 1; }

  void scheduledTo(unsigned slot) { SchedSlot = slot; }
  unsigned getSlot() const { return SchedSlot; }

  virtual bool isOperationFinish(unsigned CurSlot) const = 0;

  bool isAllDepsOpFin(unsigned CurSlot) const {
    for (const_dep_iterator I = dep_begin(), E = dep_end(); I != E; ++I) {
      if (!(*I)->isOperationFinish(CurSlot))
        return false;
    }
    return true;
  }

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

  virtual bool isOperationFinish(unsigned CurSlot) const {
    return SchedSlot <= CurSlot;
  }

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

// FIXME: this atom is not necessary
class HWAStateEnd : public HWAtom {
public:
  explicit HWAStateEnd(const FoldingSetNodeIDRef ID,
    TerminatorInst &Term, HWAtom *const *deps, size_t numDep)
    : HWAtom(ID, atomStateEnd, Term, deps, numDep) {}

  virtual bool isOperationFinish(unsigned CurSlot) const {
    llvm_unreachable("Unexpect to check state end!");
  }

  void print(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWAStateEnd *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomStateEnd;
  }
};


class HWAState : public HWAtom {
  typedef std::vector<HWAtom*> HWAtomVecType;

  HWAStateEnd *StateEnd;

  // Atoms in this state
  HWAtomVecType Atoms;

public:
  explicit HWAState(const FoldingSetNodeIDRef ID, BasicBlock &BB)
    : HWAtom(ID, atomStateBegin, BB, 0, 0), StateEnd(0) {}

  virtual bool isOperationFinish(unsigned CurSlot) const {
    return true;
  }

  void addNewAtom(HWAtom *Atom) { Atoms.push_back(Atom); }

  typedef HWAtomVecType::iterator iterator;
  iterator begin() { return Atoms.begin(); }
  iterator end() { return Atoms.end(); }

  typedef HWAtomVecType::const_iterator const_iterator;
  const_iterator begin() const { return Atoms.begin(); }
  const_iterator end() const { return Atoms.end(); }

  void resetAll() {
    for (iterator I = begin(), E = end(); I != E; ++I)
      (*I)->reset();
  }

  void endWith(HWAStateEnd &stateEnd) {
    StateEnd = &stateEnd;
  }

  HWAStateEnd *getStateEnd() { return StateEnd; }
  const HWAStateEnd *getStateEnd() const { return StateEnd; }

  typedef std::multimap<unsigned, HWAtom*> ScheduleMapType;

  void getScheduleMap(ScheduleMapType &Atoms) const;

  void print(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWAState *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomStateBegin;
  }
};


/// @brief The Schedulable Hardware Atom
class HWASchedable : public HWAtom {
  unsigned Latency;

protected:
  explicit HWASchedable(const FoldingSetNodeIDRef ID, enum HWAtomTypes T,
    Instruction &Inst, unsigned latency, HWAtom *const *deps, size_t numDep)
    : HWAtom(ID, T, Inst, deps, numDep), Latency(latency) {}
public:
  // Get the latency of this atom
  unsigned getLatency() const {
    return Latency;
  }

  virtual bool isOperationFinish(unsigned CurSlot) const {
    return SchedSlot + Latency <= CurSlot;
  }

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWASchedable *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomOpRes ||
      A->getHWAtomType() == atomEmitNextLoop ||
      A->getHWAtomType() == atomOpInst;
  }
};

class HWAEmitNextLoop : public HWASchedable {
public:
  explicit HWAEmitNextLoop(const FoldingSetNodeIDRef ID, enum HWAtomTypes T,
    Instruction &Inst, HWAtom *const *deps, size_t numDep)
    : HWASchedable(ID, T, Inst, 0, deps, numDep) {}

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWAEmitNextLoop *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomEmitNextLoop;
  }
};

class HWAOpInst : public HWASchedable {
public:
  explicit HWAOpInst(const FoldingSetNodeIDRef ID, Instruction &Inst,
    unsigned latency, HWAtom *const *deps, size_t numDep)
    : HWASchedable(ID, atomOpInst, Inst, latency, deps, numDep) {}

  void print(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWAOpInst *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomOpInst;
  }
};

class HWAOpRes : public HWASchedable {
  // The resoure that this atom using.
  HWResource &Used;
protected:
  // The instance of allocate resource
  unsigned ResId;

public:
  explicit HWAOpRes(const FoldingSetNodeIDRef ID, Instruction &Inst,
    unsigned latency, HWAtom *const *deps, size_t numDep,
    HWResource &Res, unsigned Instance)
    : HWASchedable(ID, atomOpRes, Inst, latency, deps, numDep),
      Used(Res), ResId(Instance) {
    // Remember we used this resource.
    Used.addUsingAtom(this);
  }

  /// @name The using resource
  //{
  HWResource &getUsedResource() const { return Used; }

  unsigned getResourceId() const { return ResId; }
  //}

  void print(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWAOpRes *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomOpRes;
  }
};

/// @brief Resource Table
class HWResTable {
  /// mapping allocated instences to atom
  typedef std::set<HWResource*> ResourceSetType;
  ///
  ResourceSetType ResSet;
  ///
  ResourceConfig &RC;


public:
  explicit HWResTable(ResourceConfig &rc) : RC(rc) {}
  ~HWResTable();

  HWResource *initResource(std::string Name);

  void clear();

  typedef ResourceSetType::iterator iterator;
  typedef ResourceSetType::const_iterator const_iterator;

  iterator begin() { return ResSet.begin(); }
  const_iterator begin() const { return ResSet.begin(); }

  iterator end() { return ResSet.end(); }
  const_iterator end() const { return ResSet.end(); }

  /// Get the least busy resource of a given kind
};

} // end namespace

#endif
