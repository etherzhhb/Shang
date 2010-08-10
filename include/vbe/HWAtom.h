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
#include "llvm/ADT/PointerIntPair.h"
#include "llvm/ADT/PointerUnion.h"
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
  atomWrReg,      // Write to local storage, i.e. register.
  atomRdReg,     // Import local storage form predecessor BB.
  atomPreBind,    // Operate on pre bind resource
  atomPostBind,   // Operate on post binding resource
  atomDelay,      // The delay atom.
  atomVRoot       // Virtual Root
};

enum HWEdgeTypes {
  edgeConst,
  edgeValDep,
  edgeMemDep,
  edgeCtrlDep,
};

class HWAtom;
class HWAOpInst;
class FSMState;
template<class IteratorType, class NodeType> class HWAtomDepIterator;
class HWAtomInfo;

/// @brief Inline operation
class HWEdge {
  const unsigned short EdgeType;
  HWAtom *Src;
  // Iterate distance.
  unsigned ItDst : 15;
  bool IsBackEdge : 1;

  HWEdge(const HWEdge &);            // DO NOT IMPLEMENT
  void operator=(const HWEdge &);    // DO NOT IMPLEMENT

  friend class HWAtomDepIterator<SmallVectorImpl<HWEdge*>::iterator, HWAtom>;
  friend class
    HWAtomDepIterator<SmallVectorImpl<HWEdge*>::const_iterator, const HWAtom>;
  friend class HWAtom;
  void setSrc(HWAtom *NewSrc) { Src = NewSrc; }
protected:
  HWEdge(enum HWEdgeTypes T, HWAtom *src, unsigned Dst, bool isBackEdge = false)
    : EdgeType(T), Src(src), ItDst(Dst), IsBackEdge(isBackEdge) {}
public:
  unsigned getEdgeType() const { return EdgeType; }

  // The referenced value.
  HWAtom *getSrc() const { return Src; }
  HWAtom* operator->() const { return getSrc(); }
  //HWAtom* operator*() const { return getSrc(); }

  unsigned getItDst() const { return ItDst; }
  bool isBackEdge() const { return IsBackEdge; }

  virtual void print(raw_ostream &OS) const = 0;
};

template<class IteratorType, class NodeType>
class HWAtomDepIterator : public std::iterator<std::forward_iterator_tag,
  NodeType*, ptrdiff_t> {
    IteratorType I;   // std::vector<MSchedGraphEdge>::iterator or const_iterator
    typedef HWAtomDepIterator<IteratorType, NodeType> Self;
public:
  HWAtomDepIterator(IteratorType i) : I(i) {}

  bool operator==(const Self RHS) const { return I == RHS.I; }
  bool operator!=(const Self RHS) const { return I != RHS.I; }

  const Self &operator=(const Self &RHS) {
    I = RHS.I;
    return *this;
  }

  NodeType* operator*() const {
    return (*I)->getSrc();
  }
  NodeType* operator->() const { return operator*(); }

  Self& operator++() {                // Preincrement
    ++I;
    return *this;
  }
  HWAtomDepIterator operator++(int) { // Postincrement
    HWAtomDepIterator tmp = *this;
    ++*this;
    return tmp; 
  }

  HWEdge *getEdge() { return *I; }
  const HWEdge *getEdge() const { return *I; }

  static Self findSccEdge(NodeType *Src, NodeType *Dst) {
    return std::find(Self(Dst->edge_begin()), Self(Dst->edge_end()), Src);
  }
};

class HWRegister {
  const Type *Ty;
  int Num;
  // The life time of this register, Including EndSlot.
  unsigned short StartSlot, EndSlot;
public:
  explicit HWRegister(unsigned num, const Type *T,
                 unsigned startSlot, unsigned endSlot)
    : Ty(T), Num(num), StartSlot(startSlot), EndSlot(endSlot) {}

  const Type *getType() const { return Ty; }
  unsigned getStartSlot() const { return StartSlot; }
  unsigned getEndSlot() const { return EndSlot; }
  const std::pair<unsigned short, unsigned short> getLifeTime() const {
    return std::make_pair(StartSlot, EndSlot);
  }

  // TODO: provide merge function.
  // HWReg* mergeReg(const HWReg &Other)

  unsigned getRegNum() const {
    assert(Num > 0 && "Not an ordinary Register!");
    return Num;
  }

  HWFUnitID getFUnit() const {
    assert(Num < 0 && "Not a Function unit Register!");
    return HWFUnitID(-Num);
  }

  bool isFuReg() const { return Num < 0; }

  //typedef std::set<Value*>::iterator iterator;
  //typedef std::set<Value*>::const_iterator const_iterator;

  //iterator begin() { return Vals.begin(); }
  //const_iterator begin() const { return Vals.begin(); }

  //iterator end() { return Vals.begin(); }
  //const_iterator end() const { return Vals.begin(); }
}; 

/// @brief Constant node
class HWConst : public HWEdge {
  Constant *C; 
public:
  HWConst(HWAtom *Src, Constant *Const)
    : HWEdge(edgeConst, Src, 0), C(Const) {}

  Constant *getConstant() const { return C; }

  void print(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWConst *A) { return true; }
  static inline bool classof(const HWEdge *A) {
    return A->getEdgeType() == edgeConst;
  }
};

/// @brief Value Dependence Edge.
class HWValDep : public HWEdge {
public:
  enum ValDepTypes{
    Normal, Import, Export, PHI
  };
  HWValDep(HWAtom *Src, bool isSigned, enum ValDepTypes T);

  bool isSigned() const { return IsSigned; }
  enum ValDepTypes getDepType() const { return DepType;}

  void print(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWValDep *A) { return true; }
  static inline bool classof(const HWEdge *A) {
    return A->getEdgeType() == edgeValDep;
  }

private:
  bool IsSigned;
  enum ValDepTypes DepType;
};

class HWCtrlDep : public HWEdge {
public:
  HWCtrlDep(HWAtom *Src) : HWEdge(edgeCtrlDep, Src, 0) {}

  void print(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWCtrlDep *A) { return true; }
  static inline bool classof(const HWEdge *A) {
    return A->getEdgeType() == edgeCtrlDep;
  }
};

class HWMemDep : public HWEdge {
public:
  enum MemDepTypes {
    TrueDep, AntiDep, OutputDep, NoDep
  };
private:
  enum MemDepTypes DepType;
public:
  HWMemDep(HWAtom *Src, bool isBackEdge, enum MemDepTypes DT, unsigned Dist)
    : HWEdge(edgeMemDep, Src, Dist, isBackEdge), DepType(DT) {}

  enum MemDepTypes getDepType() const { return DepType; }

  void print(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWCtrlDep *A) { return true; }
  static inline bool classof(const HWEdge *A) {
    return A->getEdgeType() == edgeMemDep;
  }
};

/// @brief Base Class of all hardware atom. 
class HWAtom : public FoldingSetNode {
  /// FastID - A reference to an Interned FoldingSetNodeID for this node.
  /// The ScalarEvolution's BumpPtrAllocator holds the data.
  FoldingSetNodeIDRef FastID;

  // The HWAtom baseclass this node corresponds to
  const unsigned short HWAtomType;
  const unsigned short Latancy;
  // The time slot that this atom scheduled to.
  // TODO: typedef SlotType
  unsigned short SchedSlot;
  unsigned short InstIdx;
  FSMState *Parant;

  /// First of all, we schedule all atom base on dependence
  SmallVector<HWEdge*, 4> Deps;

  // The atoms that using this atom.
  std::list<HWAtom*> UseList;

  void addToUseList(HWAtom *User) {
    UseList.push_back(User);
  }

  HWAtom(const HWAtom &);            // DO NOT IMPLEMENT
  void operator=(const HWAtom &);  // DO NOT IMPLEMENT

  void setDep(HWAtomDepIterator<SmallVectorImpl<HWEdge*>::iterator, HWAtom> I,
    HWAtom *NewDep) {
      assert(I != dep_end() && "I out of range!");
      I->removeFromList(this);
      NewDep->addToUseList(this);
      // Setup the dependence list.
      I.getEdge()->setSrc(NewDep);
  }
protected:
  // The corresponding LLVM Instruction
  Value &Val;

  virtual ~HWAtom();
  
  friend class FSMState;

  void resetSchedule() { SchedSlot = 0; }
  void setParent(FSMState *State) { Parant = State; }

public:
  template <class It>
  HWAtom(const FoldingSetNodeIDRef ID, unsigned HWAtomTy, Value &V,
    It depbegin, It depend, unsigned latancy, unsigned short Idx)  : FastID(ID),
    HWAtomType(HWAtomTy), Deps(depbegin, depend), Val(V), SchedSlot(0),
    Latancy(latancy), InstIdx(Idx) {
    for (dep_iterator I = dep_begin(), E = dep_end(); I != E; ++I) {
      //Deps.push_back(*I);
      (*I)->addToUseList(this);
    }
  }

  static const unsigned short MaxSlot = ~0;
  unsigned short getIdx() const { return InstIdx; }

  FSMState *getParent() { return Parant; }
  FSMState *getParent() const { return Parant; }

  // Add a new depencence edge to the atom.
  void addDep(HWEdge *E) {
    E->getSrc()->addToUseList(this);
    Deps.push_back(E);
  }

  HWAtom(const FoldingSetNodeIDRef ID, unsigned HWAtomTy, Value &V, 
         unsigned latancy, unsigned short Idx) ;

  HWAtom(const FoldingSetNodeIDRef ID, unsigned HWAtomTy, Value &V, HWEdge *Dep0,
         unsigned latancy, unsigned short Idx);

  unsigned getHWAtomType() const { return HWAtomType; }

  SmallVectorImpl<HWEdge*>::iterator edge_begin() { return Deps.begin(); }
  SmallVectorImpl<HWEdge*>::iterator edge_end() { return Deps.end(); }

  SmallVectorImpl<HWEdge*>::const_iterator edge_begin() const { return Deps.begin(); }
  SmallVectorImpl<HWEdge*>::const_iterator edge_end() const { return Deps.end(); }

  /// Profile - FoldingSet support.
  void Profile(FoldingSetNodeID& ID) { ID = FastID; }

  Value &getValue() const { return Val; }

  /// @name Operands
  //{
  HWEdge &getDep(unsigned i) const { return *Deps[i]; }

  typedef HWAtomDepIterator<SmallVectorImpl<HWEdge*>::iterator, HWAtom>
    dep_iterator;
  typedef HWAtomDepIterator<SmallVectorImpl<HWEdge*>::const_iterator, const HWAtom>
    const_dep_iterator;

  dep_iterator dep_begin() { return Deps.begin(); }
  dep_iterator dep_end() { return Deps.end(); }
  const_dep_iterator dep_begin() const { return Deps.begin(); }
  const_dep_iterator dep_end() const { return Deps.end(); }

  size_t getNumDeps() const { return Deps.size(); }

  // If the current atom depend on A?
  bool isDepOn(const HWAtom *A) const { return getDepIt(A) != dep_end(); }

  void replaceDep(HWAtom *From, HWAtom *To) {
    setDep(getDepIt(From), To);
  }

  void setDep(unsigned idx, HWAtom *NewDep) {
    // Update use list
    Deps[idx]->getSrc()->removeFromList(this);
    NewDep->addToUseList(this);
    // Setup the dependence list.
    Deps[idx]->setSrc(NewDep);
  }

  // If this Depend on A? return the position if found, return dep_end otherwise.
  const_dep_iterator getDepIt(const HWAtom *A) const {
    for (const_dep_iterator I = dep_begin(), E = dep_end(); I != E; ++I)
      if ((*I) == A)
        return I;

    return dep_end();
  }
  dep_iterator getDepIt(const HWAtom *A) {
    for (dep_iterator I = dep_begin(), E = dep_end(); I != E; ++I)
      if ((*I) == A)
        return I;
    
    return dep_end();
  }

  HWEdge *getEdgeFrom(const HWAtom *A) {
    assert(isDepOn(A) && "Current atom not depend on A!");
    return getDepIt(A).getEdge();
  }
  HWEdge *getEdgeFrom(const HWAtom *A) const {
    assert(isDepOn(A) && "Current atom not depend on A!");
    return getDepIt(A).getEdge();
  }

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

  unsigned getSlot() const { return SchedSlot; }
  unsigned getFinSlot() const { return SchedSlot + Latancy; }
  bool isScheduled() const { return SchedSlot != 0; }
  void scheduledTo(unsigned slot);

  // Get the latency of this atom
  unsigned getLatency() const { return Latancy; }

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

// Use tree iterator.
typedef df_iterator<HWAtom*, SmallPtrSet<HWAtom*, 8>, false,
  GraphTraits<HWAtom*> > usetree_iterator;
typedef df_iterator<const HWAtom*, SmallPtrSet<const HWAtom*, 8>, false,
  GraphTraits<const HWAtom*> > const_usetree_iterator;

// Predecessor tree iterator, travel the tree from exit node.
typedef df_iterator<HWAtom*, SmallPtrSet<HWAtom*, 8>, false,
  GraphTraits<Inverse<HWAtom*> > > deptree_iterator;

typedef df_iterator<const HWAtom*, SmallPtrSet<const HWAtom*, 8>, false,
  GraphTraits<Inverse<const HWAtom*> > > const_deptree_iterator;

// Write local storage.
class HWAWrReg : public HWAtom {
  HWRegister *Reg;
public:
  HWAWrReg(const FoldingSetNodeIDRef ID, HWEdge &Edge, HWRegister *reg,
    unsigned short Slot) : HWAtom(ID, atomWrReg, Edge->getValue(), &Edge,
    1, Edge->getIdx()), Reg(reg) {
    scheduledTo(Slot);
    setParent(Edge->getParent());
  }

  HWRegister *getReg() const { return Reg;  }

  static inline bool classof(const HWAWrReg *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomWrReg;
  }

  void print(raw_ostream &OS) const;
};

class HWADelay : public HWAtom {
public:
  HWADelay(const FoldingSetNodeIDRef ID, HWCtrlDep &Edge, unsigned Delay,
           unsigned Idx) : HWAtom(ID, atomDelay, Edge->getValue(), &Edge,
           Delay, Idx) {}

  static inline bool classof(const HWADelay *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomDelay;
  }

  void print(raw_ostream &OS) const;
};

// Import local storage from predecessor basicblock.
class HWARdReg : public HWAtom {
  HWRegister *Reg;
public:
  HWARdReg(const FoldingSetNodeIDRef ID, HWEdge &Edge, HWRegister *reg, Value &V)
    : HWAtom(ID, atomRdReg, V, &Edge, 0, Edge->getIdx()), Reg(reg) {
      scheduledTo(Edge->getFinSlot());
      setParent(Edge->getParent());
  }

  HWRegister *getReg() const { return Reg;  }

  bool isPHINode() const { return isa<PHINode>(getValue()); }

  static inline bool classof(const HWARdReg *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomRdReg;
  }

  void print(raw_ostream &OS) const;
};
// Create the ative atom class for register, and operate on instruction

/// @brief The Schedulable Hardware Atom
class HWAOpInst : public HWAtom {
  unsigned NumOps;
  HWFUnit FUnit;

protected:
  template <class It>
  HWAOpInst(const FoldingSetNodeIDRef ID, enum HWAtomTypes T,
    Instruction &Inst, It depbegin, It depend, size_t OpNum, HWFUnit UID,
    unsigned short Idx) : HWAtom(ID, T, Inst, depbegin, depend,
    UID.getLatency(), Idx), NumOps(OpNum), FUnit(UID) {}
public:
  HWFUnit getFunUnit() const { return FUnit; }
  HWFUnitID getFunUnitID() const { return FUnit.getFUnitID(); }

  enum HWResource::ResTypes getResClass() const {
    return FUnit.getResType();
  }

  bool isTrivial() const {
    return getResClass() == HWResource::Trivial;
  }

  unsigned getUnitNum() const { return FUnit.getUnitNum(); }

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

  HWEdge &getValDep(unsigned idx) {
    assert(idx < NumOps && "index Out of range!");
    return getDep(idx);
  }

  HWAtom *getOperand(unsigned idx) const {
    assert(idx < NumOps && "index Out of range!");
    //assert(&(getDep(idx)->getSrc()->getValue()) == getInst<Instruction>().getOperand(idx)
    //  && "HWPostBind operands broken!");
    return getDep(idx).getSrc();
  }

  // Return the opcode of the instruction.
  unsigned getOpcode() const {
    return cast<Instruction>(getValue()).getOpcode();
  }

  void print(raw_ostream &OS) const;
  
  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWAOpInst *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomPreBind ||
           A->getHWAtomType() == atomPostBind;
  }
};

class HWAPostBind : public HWAOpInst {
public:
  template <class It>
  HWAPostBind(const FoldingSetNodeIDRef ID, Instruction &Inst,
               It depbegin, It depend, size_t OpNum, HWFUnit FUID,
               unsigned short Idx) : HWAOpInst(ID, atomPostBind, Inst,
               depbegin, depend, OpNum, FUID, Idx) {}

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWAPostBind *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomPostBind;
  }
};

class HWAPreBind : public HWAOpInst {
public:
  template <class It>
  HWAPreBind(const FoldingSetNodeIDRef ID, Instruction &Inst,
             It depbegin, It depend, unsigned OpNum, HWFUnit FUID,
             unsigned short Idx) : HWAOpInst(ID, atomPreBind, Inst,
             depbegin, depend, OpNum, FUID, Idx) {}

  HWAPreBind(const FoldingSetNodeIDRef ID, HWAPostBind &PostBind,
             HWFUnit FUID);

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWAPreBind *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomPreBind;
  }
};

// Virtual Root
class FSMState  : public HWAtom {
  HWAOpInst *ExitRoot;

  // The registers that store the source value of PHINodes.
  typedef std::map<const Instruction*, HWAtom*> PHISrcMapType;
  PHISrcMapType PHISrc;

  // Modulo for modulo schedule.
  unsigned short II;
  bool HaveSelfLoop;

  void setExitRoot(HWAOpInst *Exit) {
    ExitRoot = Exit;

    for (usetree_iterator I = usetree_begin(), E = usetree_end(); I != E; ++I)
      (*I)->setParent(this);
  }
  void setHaveSelfLoop(bool haveSelfLoop) { HaveSelfLoop = haveSelfLoop; }

  friend class HWAtomInfo;
public:
  FSMState(const FoldingSetNodeIDRef ID, BasicBlock &BB, unsigned short Idx)
    : HWAtom(ID, atomVRoot, BB, 0, Idx) , HaveSelfLoop(false), II(0) {
  }
  ~FSMState() { PHISrc.clear(); }
  
  /// @name Roots
  //{
  HWAOpInst *getExitRoot() const { return ExitRoot; }
  HWAOpInst *getExitRoot() { return ExitRoot; }
  //}

  // Return the corresponding basiclbocl of this Execute stage.
  BasicBlock *getBasicBlock() { return &cast<BasicBlock>(getValue()); }
  BasicBlock *getBasicBlock() const { return &cast<BasicBlock>(getValue()); }

  // Successor tree iterator, travel the tree from entry node.
  usetree_iterator usetree_begin() {
    return usetree_iterator::begin(this);
  }
  usetree_iterator usetree_end() {
    return usetree_iterator::end(this);
  }

  const_usetree_iterator usetree_begin() const { 
    return const_usetree_iterator::begin(this);
  }
  const_usetree_iterator usetree_end() const {
    return const_usetree_iterator::end(this);
  }

  deptree_iterator deptree_begin() { return deptree_iterator::begin(ExitRoot); }
  const_deptree_iterator deptree_begin() const {
    return const_deptree_iterator::begin(ExitRoot);
  }

  deptree_iterator deptree_end() { return deptree_iterator::end(ExitRoot); }
  const_deptree_iterator deptree_end()  const {
    return const_deptree_iterator::end(ExitRoot);
  }

  void resetSchedule() {
    for (usetree_iterator I = usetree_begin(), E = usetree_end(); I != E; ++I)
      (*I)->resetSchedule();
  }

  void updatePHISrc(const Instruction *Inst, HWAtom *A) {
    PHISrc[Inst] = A;
  }

  HWAtom *getPHISrc(const Instruction *Inst) {
    PHISrcMapType::iterator At = PHISrc.find(Inst);
    if (At == PHISrc.end())
      return 0;

    return At->second;
  }

  unsigned getEndSlot() const { return getExitRoot()->getSlot(); }

  unsigned getTotalSlot() const {
    return getEndSlot() - getSlot();
  }

  // II for Modulo schedule

  void setII(unsigned ii) { II = ii; }
  unsigned getII() const { return II; }
  unsigned getIISlot() const { return getSlot() + II; }
  bool haveSelfLoop() const { return HaveSelfLoop; }

  typedef std::multimap<unsigned, HWAtom*> ScheduleMapType;

  void getScheduleMap(ScheduleMapType &Atoms) const;

  void print(raw_ostream &OS) const;

  void dump() const;


  static inline bool classof(const FSMState *A) { return true; }
  static inline bool classof(const HWAtom *A) {
    return A->getHWAtomType() == atomVRoot;
  }
};

} // end namespace

#endif
