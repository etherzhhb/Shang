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
  atomValDep,          // The constant Atom
  atomMemDep,
  atomCtrlDep,

  atomPreBind,      // Operate on pre bind resource
  atomPostBind,     // Operate on post binding resource
  atomVRoot       // Virtual Root
};

template <class GraphType>
class DAG {};

template<class IteratorType, class NodeType>
class DAGIterator : public std::iterator<std::forward_iterator_tag,
  NodeType*, ptrdiff_t> {
  IteratorType I, E;
public:
  inline DAGIterator(IteratorType i, IteratorType e);

  bool operator==(const DAGIterator RHS) const { 
    return (I == RHS.I && E == RHS.E);
  }
  bool operator!=(const DAGIterator RHS) const { 
    return !operator==(RHS);
  }

  inline const DAGIterator &operator=(const DAGIterator &RHS) {
    I = RHS.I;
    E = RHS.E;
    return *this;
  }

  inline NodeType* operator*() const {
    return *I;
  }
  inline NodeType* operator->() const { return operator*(); }

  inline DAGIterator& operator++();                // Preincrement

  inline DAGIterator operator++(int) {              // Postincrement
    DAGIterator tmp = *this;
    ++*this;
    return tmp; 
  }
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
  unsigned char NumDeps;

  // The atoms that using this atom.
  std::list<HWAtom*> UseList;

  void addToUseList(HWAtom *User) {
    UseList.push_back(User);
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
  HWAtom(const FoldingSetNodeIDRef ID, unsigned HWAtomTy, Value &V,
    HWAtom **deps, size_t numOps);

  unsigned getHWAtomType() const { return HWAtomType; }

  /// Profile - FoldingSet support.
  void Profile(FoldingSetNodeID& ID) { ID = FastID; }

  Value &getValue() const { return Val; }

  static bool willCauseCycle(const HWAtom *Edge);

  /// @name Operands
  //{
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
  typedef const HWAtom *const *const_dep_iterator;
  typedef DAGIterator<dep_iterator, HWAtom> dag_dep_iterator;
  typedef DAGIterator<const_dep_iterator, const HWAtom> dag_const_dep_iterator;

  dep_iterator dep_begin() { return Deps; }
  dep_iterator dep_end() { return Deps + NumDeps; }
  const_dep_iterator dep_begin() const { return Deps; }
  const_dep_iterator dep_end() const { return Deps + NumDeps; }

  dag_dep_iterator dag_dep_begin() {
    return dag_dep_iterator(dep_begin(), dep_end());
  }
  dag_dep_iterator dag_dep_end() {
    return dag_dep_iterator(dep_end(), dep_end());
  }
  dag_const_dep_iterator dag_dep_begin() const {
    return dag_const_dep_iterator(dep_begin(), dep_end());
  }
  dag_const_dep_iterator dag_dep_end() const {
    return dag_const_dep_iterator(dep_end(), dep_end());
  }

  size_t getNumDeps() const { return NumDeps; }
  size_t getNumDAGDeps() const {
    return std::distance(dag_dep_begin(), dag_dep_end());
  }

  // If this Depend on A? return the position if found, return dep_end otherwise.
  const_dep_iterator getDepIdx(HWAtom *A) const {
    return std::find(dep_begin(), dep_end(), A);
  }
  dep_iterator getDepIdx(HWAtom *A) {
    return std::find(dep_begin(), dep_end(), A);
  }
  // If the current atom depend on A?
  bool isDepOn(HWAtom *A) const { return getDepIdx(A) != dep_end(); }
  //}

  /// @name Use
  //{
  typedef std::list<HWAtom*>::iterator use_iterator;
  typedef std::list<HWAtom*>::const_iterator const_use_iterator;
  typedef DAGIterator<use_iterator, HWAtom> dag_use_iterator;
  typedef DAGIterator<const_use_iterator, const HWAtom> dag_const_use_iterator;

  use_iterator use_begin() { return UseList.begin(); }
  const_use_iterator use_begin() const { return UseList.begin(); }
  use_iterator use_end() { return UseList.end(); }
  const_use_iterator use_end() const { return UseList.end(); }

  dag_use_iterator dag_use_begin() {
    return dag_use_iterator(use_begin(), use_end());
  }
  dag_use_iterator dag_use_end() {
    return dag_use_iterator(use_end(), use_end());
  }
  dag_const_use_iterator dag_use_begin() const {
    return dag_const_use_iterator(use_begin(), use_end());
  }
  dag_const_use_iterator dag_use_end() const {
    return dag_const_use_iterator(use_end(), use_end());
  }

  size_t getNumDAGUses() const {
    return std::distance(dag_use_begin(), dag_use_end());
  }


  HWAtom *use_back() { return UseList.back(); }
  HWAtom *use_back() const { return UseList.back(); }


  void removeFromList(HWAtom *User) {
    std::list<HWAtom*>::iterator at = std::find(UseList.begin(), UseList.end(),
      User);
    assert(at != UseList.end() && "Not in use list!");
    UseList.erase(at);
  }

  void replaceAllUseBy(HWAtom *A);

  bool use_empty() { return UseList.empty(); }
  size_t getNumUses() const { return UseList.size(); }
  //}

  void resetSchedule() { SchedSlot = 0; }
  unsigned getSlot() const { return SchedSlot; }
  bool isScheduled() const { return SchedSlot != 0; }
  void scheduledTo(unsigned slot);

  // Get the latency of this atom
  virtual unsigned getLatency() const { return 0; }

  /// print - Print out the internal representation of this atom to the
  /// specified stream.  This should really only be used for debugging
  /// purposes.
  virtual void print(raw_ostream &OS) const = 0;

  /// dump - This method is used for debugging.
  ///
  void dump() const;
};


template<class IteratorType, class NodeType>
inline DAGIterator<IteratorType, NodeType>::DAGIterator(IteratorType i,
                                                        IteratorType e)
                                                        : I(i), E(e) {
  while(I != E && HWAtom::willCauseCycle(*I))
    ++I;
}

template<class IteratorType, class NodeType>
inline DAGIterator<IteratorType, NodeType>&
DAGIterator<IteratorType, NodeType>::operator++() {
  do
    ++I;
  while(I != E && HWAtom::willCauseCycle(*I));

  return *this;
}

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


template<> struct GraphTraits<Inverse<esyn::DAG<esyn::HWAtom*> > > {
  typedef esyn::HWAtom NodeType;
  typedef esyn::HWAtom::dag_dep_iterator ChildIteratorType;
  static NodeType *getEntryNode(NodeType* N) { return N; }
  static inline ChildIteratorType child_begin(NodeType *N) {
    return N->dag_dep_begin();
  }
  static inline ChildIteratorType child_end(NodeType *N) {
    return N->dag_dep_end();
  }
};
template<> struct GraphTraits<Inverse<esyn::DAG<const esyn::HWAtom*> > > {
  typedef const esyn::HWAtom NodeType;
  typedef esyn::HWAtom::dag_const_dep_iterator ChildIteratorType;
  static NodeType *getEntryNode(NodeType* N) { return N; }
  static inline ChildIteratorType child_begin(NodeType *N) {
    return N->dag_dep_begin();
  }
  static inline ChildIteratorType child_end(NodeType *N) {
    return N->dag_dep_end();
  }
};
template<> struct GraphTraits<esyn::DAG<esyn::HWAtom*> > {
  typedef esyn::HWAtom NodeType;
  typedef esyn::HWAtom::dag_use_iterator ChildIteratorType;
  static NodeType *getEntryNode(NodeType* N) { return N; }
  static inline ChildIteratorType child_begin(NodeType *N) {
    return N->dag_use_begin();
  }
  static inline ChildIteratorType child_end(NodeType *N) {
    return N->dag_use_end();
  }
};
template<> struct GraphTraits<esyn::DAG<const esyn::HWAtom*> > {
  typedef const esyn::HWAtom NodeType;
  typedef esyn::HWAtom::dag_const_use_iterator ChildIteratorType;
  static NodeType *getEntryNode(NodeType* N) { return N; }
  static inline ChildIteratorType child_begin(NodeType *N) {
    return N->dag_use_begin();
  }
  static inline ChildIteratorType child_end(NodeType *N) {
    return N->dag_use_end();
  }
};

}

// FIXME: Move to a seperate header.
namespace esyn {

// Use tree iterator.
typedef df_iterator<HWAtom*, SmallPtrSet<HWAtom*, 8>, false,
  GraphTraits<esyn::DAG<HWAtom*> > > usetree_iterator;
typedef df_iterator<const HWAtom*, SmallPtrSet<const HWAtom*, 8>, false,
  GraphTraits<esyn::DAG<const HWAtom*> > > const_usetree_iterator;

// Predecessor tree iterator, travel the tree from exit node.
typedef df_iterator<HWAtom*, SmallPtrSet<HWAtom*, 8>, false,
  GraphTraits<Inverse<esyn::DAG<HWAtom*> > > > deptree_iterator;

typedef df_iterator<const HWAtom*, SmallPtrSet<const HWAtom*, 8>, false,
  GraphTraits<Inverse<esyn::DAG<const HWAtom*> > > > const_deptree_iterator;

/// @brief Inline operation
class HWADepEdge : public HWAtom {
protected:
  HWADepEdge(const FoldingSetNodeIDRef ID, enum HWAtomTypes T,
    Value &V, HWAtom **O) : HWAtom(ID, T, V, O, 1) {}
public:

  // The referenced value.
  HWAtom *getSrc() { return getDep(0); }

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWADepEdge *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomValDep ||
      A->getHWAtomType() == atomMemDep ||
      A->getHWAtomType() == atomCtrlDep;
  }
};

class HWReg {
  std::set<Value*> Vals;
  const Type *Ty;
  unsigned Num;

public:
  explicit HWReg(unsigned num, Value &V)
    : Ty(V.getType()), Num(num) {
    Vals.insert(&V);
  }

  const Type *getType() const { return Ty; }

  void addValue(Value &V) {
    assert(Ty == V.getType() && "Can merge difference type!");
    Vals.insert(&V);
  }

  unsigned getRegNum() const { return Num; }

  typedef std::set<Value*>::iterator iterator;
  typedef std::set<Value*>::const_iterator const_iterator;

  iterator begin() { return Vals.begin(); }
  const_iterator begin() const { return Vals.begin(); }

  iterator end() { return Vals.begin(); }
  const_iterator end() const { return Vals.begin(); }
}; 

/// @brief Constant node
class HWAValDep : public HWADepEdge {
  bool Signed;
  HWReg *R;
public:
  HWAValDep(const FoldingSetNodeIDRef ID, Value &V, HWAtom **E,
    bool isSigned, HWReg *Reg)
    : HWADepEdge(ID, atomValDep, V, E), Signed(isSigned), R(Reg) {}

  HWReg *getReg() const { return R; }

  void print(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWAValDep *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomValDep;
  }
};

class HWACtrlDep : public HWADepEdge {
public:
  HWACtrlDep(const FoldingSetNodeIDRef ID, Value &V, HWAtom **E)
    : HWADepEdge(ID, atomCtrlDep, V, E) {}

  void print(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWACtrlDep *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomCtrlDep;
  }
};

class HWAMemDep : public HWADepEdge {
public:
  enum MemDepTypes {
    TrueDep, AntiDep, OutputDep
  };
private:
  enum MemDepTypes DepType;
  // Iteration distance.
  unsigned ItDst;
public:
  HWAMemDep(const FoldingSetNodeIDRef ID, Value &V, HWAtom **E,
    enum MemDepTypes DT, unsigned Dst)
    : HWADepEdge(ID, atomMemDep, V, E), DepType(DT), ItDst(Dst) {}

  enum MemDepTypes getDepType() const { return DepType; }
  unsigned getItDst() const { return ItDst; }

  void print(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWACtrlDep *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomMemDep;
  }
};

// Virtual Root
class HWAVRoot : public HWAtom {
public:
  HWAVRoot(const FoldingSetNodeIDRef ID, BasicBlock &BB)
    : HWAtom(ID, atomVRoot, BB, 0, 0) {}

  BasicBlock &getBasicBlock() { return cast<BasicBlock>(getValue()); }


  usetree_iterator begin() {
    return usetree_iterator::begin(this);
  }
  usetree_iterator end() {
    return usetree_iterator::end(this);
  }

  const_usetree_iterator begin() const { 
    return const_usetree_iterator::begin(this);
  }
  const_usetree_iterator end() const {
    return const_usetree_iterator::end(this);
  }

  void print(raw_ostream &OS) const;

  static inline bool classof(const HWAVRoot *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomVRoot;
  }
};

// Create the ative atom class for register, and operate on instruction

/// @brief The Schedulable Hardware Atom
class HWAOpInst : public HWAtom {
  // The latency of this atom
  unsigned Latency;
  unsigned NumOps;
protected:
  unsigned SubClassData;

  HWAOpInst(const FoldingSetNodeIDRef ID, enum HWAtomTypes T,
    Instruction &Inst, unsigned latency, HWAtom **deps, size_t numDep,
    size_t OpNum, unsigned subClassData = 0)
    : HWAtom(ID, T, Inst, deps, numDep), Latency(latency), NumOps(OpNum),
    SubClassData(subClassData) {}
public:
  // Get the latency of this atom
  unsigned getLatency() const {
    return Latency;
  }

  template<class InstTy>
  InstTy &getInst() { return cast<InstTy>(getValue()); }

  template<class InstTy>
  const InstTy &getInst() const { return cast<InstTy>(getValue()); }

  Value *getIOperand(unsigned idx) {
    return getInst<Instruction>().getOperand(idx);
  }

  Value *getIOperand(unsigned idx) const {
    return getInst<Instruction>().getOperand(idx);
  }

  size_t getInstNumOps () const { return NumOps; }

  HWAtom *getOperand(unsigned idx) {
    assert(idx < NumOps && "index Out of range!");
    assert(&(getDep(idx)->getValue()) == getInst<Instruction>().getOperand(idx)
      && "HWPostBind operands broken!");
    return getDep(idx);
  }

  // Help the scheduler to identify difference operation class
  virtual enum HWResource::ResTypes getResClass() const {
    return HWResource::Trivial;
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
    enum HWResource::ResTypes OpClass, unsigned Instance = 0)
    : HWAOpInst(ID, atomPreBind, Inst, latency, deps, numDep, OpNum,
    HWResource::createResId(OpClass, Instance)) {}

  HWAPreBind(const FoldingSetNodeIDRef ID, HWAPostBind &PostBind,
    unsigned Instance);

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
class FSMState {
  typedef std::vector<HWAtom*> HWAtomVecType;

  HWAVRoot &EntryRoot;
  HWAOpInst &ExitRoot;

public:
  explicit FSMState(HWAVRoot *entry, HWAOpInst *exit)
    : EntryRoot(*entry), ExitRoot(*exit) {}


  /// @name Roots
  //{
  HWAVRoot &getEntryRoot() const { return EntryRoot; }
  HWAOpInst &getExitRoot() const { return ExitRoot; }

  HWAVRoot &getEntryRoot() { return EntryRoot; }
  HWAOpInst &getExitRoot() { return ExitRoot; }
  //}

  // Return the corresponding basiclbocl of this Execute stage.
  BasicBlock *getBasicBlock() { return &EntryRoot.getBasicBlock(); }
  BasicBlock *getBasicBlock() const { return &EntryRoot.getBasicBlock(); }

  // Successor tree iterator, travel the tree from entry node.
  usetree_iterator usetree_begin() { return EntryRoot.begin(); }
  const_usetree_iterator usetree_begin() const {
    return ((const HWAVRoot&)EntryRoot).begin();
  }

  usetree_iterator usetree_end() { return EntryRoot.end(); }
  const_usetree_iterator usetree_end() const {
    return ((const HWAVRoot&)EntryRoot).end();
  }

  deptree_iterator deptree_begin() { return deptree_iterator::begin(&ExitRoot); }
  const_deptree_iterator deptree_begin() const {
    return const_deptree_iterator::begin(&ExitRoot);
  }

  deptree_iterator deptree_end() { return deptree_iterator::end(&ExitRoot); }
  const_deptree_iterator deptree_end()  const {
    return const_deptree_iterator::end(&ExitRoot);
  }

  void resetSchedule() {
    for (usetree_iterator I = usetree_begin(), E = usetree_end(); I != E; ++I)
      (*I)->resetSchedule();
  }

  typedef std::multimap<unsigned, HWAtom*> ScheduleMapType;

  void getScheduleMap(ScheduleMapType &Atoms) const;

  void print(raw_ostream &OS) const;
};

} // end namespace

#endif
