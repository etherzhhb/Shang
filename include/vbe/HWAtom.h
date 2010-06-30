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
#include "llvm/Assembly/Writer.h"
#include "llvm/ADT/GraphTraits.h"
#include "llvm/ADT/DepthFirstIterator.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/FoldingSet.h"
#include "llvm/Support/Allocator.h"
#include "llvm/Support/InstVisitor.h"
#include "llvm/Support/raw_os_ostream.h"
#include "llvm/Support/ErrorHandling.h"

#include "vbe/ResourceConfig.h"

#include <list>

using namespace llvm;

namespace esyn {

enum HWAtomTypes {
  atomConst,          // The constant Atom
  atomSignedPrefix,   // Represent the Signed marker, use in Ashr
  atomWireOp,         // Trunc, Z/SExt, PtrToInt, IntToPtr
                      // Data communication atom
  atomRegister,       // Assign value to register, use in infinite scheduler
                      // Schedeable atoms
  atomPreBind,      // Operate on pre bind resource
  atomPostBind,     // Operate on post binding resource
  atomVRoot       // Virtual Root
};

/// @brief Base Class of all hardware atom. 
class HWAtom : public FoldingSetNode {
  /// FastID - A reference to an Interned FoldingSetNodeID for this node.
  /// The ScalarEvolution's BumpPtrAllocator holds the data.
  FoldingSetNodeIDRef FastID;

  // The HWAtom baseclass this node corresponds to
  const unsigned short HWAtomType;

  /// First of all, we schedule all atom base on dependence
  HWAtom **Deps;
  size_t NumDeps;

  // The atoms that using this atom.
  std::list<HWAtom*> UseList;

  void addToUseList(HWAtom *User) {
    UseList.push_back(User);
  }

  void removeFromList(HWAtom *User) {
    std::list<HWAtom*>::iterator at = std::find(UseList.begin(), UseList.end(),
                                                User);
    assert(at != UseList.end() && "Not in use list!");
    UseList.erase(at);
  }

  HWAtom(const HWAtom &);            // DO NOT IMPLEMENT
  void operator=(const HWAtom &);  // DO NOT IMPLEMENT

protected:
  // The corresponding LLVM Instruction
  Value &Val;

  // The time slot that this atom scheduled to.
  unsigned SchedSlot;

  virtual ~HWAtom();

public:
  explicit HWAtom(const FoldingSetNodeIDRef ID, unsigned HWAtomTy, Value &V,
    HWAtom **deps, size_t numDeps);

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

  void setDep(unsigned idx, HWAtom *NewDep) {
    assert(idx < NumDeps && "Operand index out of range!");
    // Update use list
    Deps[idx]->removeFromList(this);
    NewDep->addToUseList(this);
    // Setup the dependence list.
    Deps[idx] = NewDep;
  }

  typedef HWAtom *const *dep_iterator;
  dep_iterator dep_begin() { return Deps; }
  dep_iterator dep_end() { return Deps + NumDeps; }

  typedef const HWAtom *const *const_dep_iterator;
  const_dep_iterator dep_begin() const { return Deps; }
  const_dep_iterator dep_end() const { return Deps + NumDeps; }
  //}

  /// @name Use
  //{
  typedef std::list<HWAtom*>::iterator use_iterator;
  typedef std::list<HWAtom*>::const_iterator const_use_iterator;

  use_iterator use_begin() { return UseList.begin(); }
  const_use_iterator use_begin() const { return UseList.begin(); }
  
  use_iterator use_end() { return UseList.end(); }
  const_use_iterator use_end() const { return UseList.end(); }

  HWAtom *use_back() { return UseList.back(); }
  HWAtom *use_back() const { return UseList.back(); }

  bool use_empty() { return UseList.empty(); }
  //}

  virtual void reset() { SchedSlot = UINT32_MAX  >> 1; }

  void scheduledTo(unsigned slot) { SchedSlot = slot; }
  unsigned getSlot() const { return SchedSlot; }

  /// print - Print out the internal representation of this atom to the
  /// specified stream.  This should really only be used for debugging
  /// purposes.
  virtual void print(raw_ostream &OS) const = 0;

  /// dump - This method is used for debugging.
  ///
  void dump() const;
};

}

namespace llvm {

template<> struct GraphTraits<Inverse<esyn::HWAtom*> > {
  typedef esyn::HWAtom NodeType;
  typedef esyn::HWAtom::dep_iterator ChildIteratorType;
  static NodeType *getEntryNode(NodeType* N) { return N; }
  static inline ChildIteratorType child_begin(NodeType *N) {
    return N->dep_begin();
  }
  static inline ChildIteratorType child_end(NodeType *N) {
    return N->dep_end();
  }
};

template<> struct GraphTraits<Inverse<const esyn::HWAtom*> > {
  typedef const esyn::HWAtom NodeType;
  typedef esyn::HWAtom::const_dep_iterator ChildIteratorType;
  static NodeType *getEntryNode(NodeType* N) { return N; }
  static inline ChildIteratorType child_begin(NodeType *N) {
    return N->dep_begin();
  }
  static inline ChildIteratorType child_end(NodeType *N) {
    return N->dep_end();
  }
};

template<> struct GraphTraits<esyn::HWAtom*> {
  typedef esyn::HWAtom NodeType;
  typedef esyn::HWAtom::use_iterator ChildIteratorType;
  static NodeType *getEntryNode(NodeType* N) { return N; }
  static inline ChildIteratorType child_begin(NodeType *N) {
    return N->use_begin();
  }
  static inline ChildIteratorType child_end(NodeType *N) {
    return N->use_end();
  }
};

template<> struct GraphTraits<const esyn::HWAtom*> {
  typedef const esyn::HWAtom NodeType;
  typedef esyn::HWAtom::const_use_iterator ChildIteratorType;
  static NodeType *getEntryNode(NodeType* N) { return N; }
  static inline ChildIteratorType child_begin(NodeType *N) {
    return N->use_begin();
  }
  static inline ChildIteratorType child_end(NodeType *N) {
    return N->use_end();
  }
};

}

// FIXME: Move to a seperate header.
namespace esyn {
// Virtual Root
class HWAVRoot : public HWAtom {
public:
  explicit HWAVRoot(const FoldingSetNodeIDRef ID, BasicBlock &BB)
    : HWAtom(ID, atomVRoot, BB, 0, 0) {}

  BasicBlock &getBasicBlock() { return cast<BasicBlock>(getValue()); }

  typedef df_iterator<HWAtom*, SmallPtrSet<HWAtom*, 8>, false,
    GraphTraits<HWAtom*> > tree_iterator;
  typedef df_iterator<const HWAtom*, SmallPtrSet<const HWAtom*, 8>, false,
    GraphTraits<const HWAtom*> > const_tree_iterator;

  tree_iterator begin() {
    return tree_iterator::begin(this);
  }
  tree_iterator end() {
    return tree_iterator::end(this);
  }

  const_tree_iterator begin() const { 
    return const_tree_iterator::begin(this);
  }
  const_tree_iterator end() const {
    return const_tree_iterator::end(this);
  }

  void print(raw_ostream &OS) const;

  static inline bool classof(const HWAVRoot *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomVRoot;
  }
};

/// @brief Constant node
class HWAConst : public HWAtom {
public:
  explicit HWAConst(const FoldingSetNodeIDRef ID, Value &V, HWAtom **E)
    : HWAtom(ID, atomConst, V, E, 1) {}

  void print(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWAConst *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomConst;
  }
};

/// @brief Inline operation
class HWAInline : public HWAtom {
protected:
  explicit HWAInline(const FoldingSetNodeIDRef ID, enum HWAtomTypes T,
    Value &V, HWAtom **O) : HWAtom(ID, T, V, O, 1) {}
public:

  // The referenced value.
  HWAtom *getRefVal() { return getDep(0); }

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
  explicit HWASigned(const FoldingSetNodeIDRef ID, HWAtom **O)
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
  explicit HWARegister(const FoldingSetNodeIDRef ID, Value &V,
    HWAtom **O) 
    : HWAInline(ID, atomRegister, V, O) {}

  void print(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWARegister *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomRegister;
  } 
};

/// @brief The Schedulable Hardware Atom
class HWAOpInst : public HWAtom {
  // The latency of this atom
  unsigned Latency;
  // Effective operand number
  unsigned InstNumOps;
protected:
  unsigned SubClassData;

  explicit HWAOpInst(const FoldingSetNodeIDRef ID, enum HWAtomTypes T,
    Instruction &Inst, unsigned latency, HWAtom **deps, size_t numDep,
    size_t OpNum, unsigned subClassData = 0)
    : HWAtom(ID, T, Inst, deps, numDep), Latency(latency),
    InstNumOps(OpNum), SubClassData(subClassData) {}
public:
  // Get the latency of this atom
  unsigned getLatency() const {
    return Latency;
  }

  template<class InstTy>
  InstTy &getInst() { return cast<InstTy>(getValue()); }

  size_t getInstNumOps () const { return InstNumOps; }

  HWAtom *getOperand(unsigned idx) {
    assert(idx < InstNumOps && "index Out of range!");
    assert(&(getDep(idx)->getValue()) == getInst<Instruction>().getOperand(idx)
      && "HWPostBind operands broken!");
    return getDep(idx);
  }

  // Help the scheduler to identify difference operation class
  virtual enum HWResource::ResTypes getResClass() const {
    return HWResource::Other;
  }

  // Return the opcode of the instruction.
  unsigned getOpcode() const {
    return cast<Instruction>(getValue()).getOpcode();
  }

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWAOpInst *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomPreBind ||
      A->getHWAtomType() == atomPostBind;
  }
};

class HWAPostBind : public HWAOpInst {
public:
  explicit HWAPostBind(const FoldingSetNodeIDRef ID, Instruction &Inst,
    unsigned latency, HWAtom **deps, size_t numDep, size_t OpNum,
      enum HWResource::ResTypes OpClass)
    : HWAOpInst(ID, atomPostBind, Inst, latency,
                  deps, numDep, OpNum, OpClass) {}
  enum HWResource::ResTypes getResClass() const {
    return (HWResource::ResTypes)SubClassData;
  }

  void print(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWAPostBind *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomPostBind;
  }
};

class HWAPreBind : public HWAOpInst {
public:
  explicit HWAPreBind(const FoldingSetNodeIDRef ID, Instruction &Inst,
    unsigned latency, HWAtom **deps, size_t numDep, size_t OpNum,
    HWResource &Res, unsigned Instance = 0)
    : HWAOpInst(ID, atomPreBind, Inst, latency, deps, numDep, OpNum,
    HWResource::createResId(Res.getResourceType(), Instance)) {}

  /// @name The using resource
  //{
  // Help the scheduler to identify difference resource unit.
  HWResource::ResIdType getResourceId() const {
    return SubClassData;
  }

  enum HWResource::ResTypes getResClass() const {
    return HWResource::extractResType(getResourceId());
  }
  unsigned getAllocatedInstance() const {
    return HWResource::extractInstanceId(getResourceId());
  }
  //}

  void print(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWAPreBind *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomPreBind;
  }
};

// Execute state.
class ExecStage {
  typedef std::vector<HWAtom*> HWAtomVecType;

  HWAVRoot &EntryRoot;
  HWAPostBind &ExitRoot;

public:
  explicit ExecStage(HWAVRoot *entry, HWAPostBind *exit)
    : EntryRoot(*entry), ExitRoot(*exit) {}


  /// @name Roots
  //{
  HWAVRoot &getEntryRoot() const { return EntryRoot; }
  HWAPostBind &getExitRoot() const { return ExitRoot; }

  HWAVRoot &getEntryRoot() { return EntryRoot; }
  HWAPostBind &getExitRoot() { return ExitRoot; }
  //}

  // Return the corresponding basiclbocl of this Execute stage.
  BasicBlock *getBasicBlock() { return &EntryRoot.getBasicBlock(); }
  BasicBlock *getBasicBlock() const { return &EntryRoot.getBasicBlock(); }

  // Successor tree iterator, travel the tree from entry node.
  typedef HWAVRoot::tree_iterator entry_iterator;
  typedef HWAVRoot::const_tree_iterator const_entry_iterator;

  entry_iterator entry_begin() { return EntryRoot.begin(); }
  const_entry_iterator entry_begin() const {
    return ((const HWAVRoot&)EntryRoot).begin();
  }

  entry_iterator entry_end() { return EntryRoot.end(); }
  const_entry_iterator entry_end() const {
    return ((const HWAVRoot&)EntryRoot).end();
  }

  // Predecessor tree iterator, travel the tree from exit node.
  typedef df_iterator<HWAtom*, SmallPtrSet<HWAtom*, 8>, false,
    GraphTraits<Inverse<HWAtom*> > > exit_iterator;

  typedef df_iterator<const HWAtom*, SmallPtrSet<const HWAtom*, 8>, false,
    GraphTraits<Inverse<const HWAtom*> > > const_exit_iterator;

  exit_iterator exit_begin() { return exit_iterator::begin(&ExitRoot); }
  const_exit_iterator exit_begin() const {
    return const_exit_iterator::begin(&ExitRoot);
  }

  exit_iterator exit_end() { return exit_iterator::end(&ExitRoot); }
  const_exit_iterator exit_end()  const {
    return const_exit_iterator::end(&ExitRoot);
  }

  void resetAll() {
    for (entry_iterator I = entry_begin(), E = entry_end(); I != E; ++I)
      (*I)->reset();
  }

  typedef std::multimap<unsigned, HWAtom*> ScheduleMapType;

  void getScheduleMap(ScheduleMapType &Atoms) const;

  void print(raw_ostream &OS) const;
};

} // end namespace

#endif
