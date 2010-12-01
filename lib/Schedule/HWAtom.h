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

#include "vtm/FUs.h"

#include "llvm/Assembly/Writer.h"
#include "llvm/ADT/GraphTraits.h"
#include "llvm/ADT/DepthFirstIterator.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/PointerIntPair.h"
#include "llvm/ADT/PointerUnion.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/Support/raw_os_ostream.h"

#include <list>

namespace llvm {
class BitLevelInfo;
class ForceDirectedSchedulingBase;
class FuncUnitId;
class HWAtom;
class FSMState;
struct VTargetMachine;

class MachineBasicBlock;
class MachineInstr;
class MachineOperand;

/// @brief Inline operation
class HWEdge {
public:
  enum HWEdgeTypes {
    edgeValDep,
    edgeMemDep,
    edgeCtrlDep
  };
private:
  const unsigned short EdgeType;
  HWAtom *Src;
  // The latancy of this edge.
  unsigned Latancy;
  // Iterate distance.
  unsigned ItDst : 31;
  bool IsBackEdge : 1;
  
  HWEdge(const HWEdge &);            // DO NOT IMPLEMENT
  void operator=(const HWEdge &);    // DO NOT IMPLEMENT

  friend class HWAtom;
  void setSrc(HWAtom *NewSrc) { Src = NewSrc; }
protected:
  HWEdge(enum HWEdgeTypes T, HWAtom *src, unsigned latancy, unsigned Dst,
         bool isBackEdge = false)
    : EdgeType(T), Src(src), Latancy(latancy), ItDst(Dst),
    IsBackEdge(isBackEdge) {
    assert(!isBackEdge || Dst != 0
           && "Back edge must have a non-zero iterate distance!");
  }
public:
  unsigned getLatency() const { return Latancy; }

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
};

/// @brief Value Dependence Edge.
class HWValDep : public HWEdge {
public:
  enum ValDepTypes{
    Normal, Import, Export, PHI
  };
  HWValDep(HWAtom *Src, unsigned latancy, bool isSigned, enum ValDepTypes T)
    : HWEdge(edgeValDep, Src, latancy,  0), IsSigned(isSigned), DepType(T) {}

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
  HWCtrlDep(HWAtom *Src, unsigned latancy)
    : HWEdge(edgeCtrlDep, Src, latancy, 0) {}

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
  HWMemDep(HWAtom *Src, unsigned latancy, bool isBackEdge,
           enum MemDepTypes DT, unsigned Dist)
    : HWEdge(edgeMemDep, Src, latancy, Dist, isBackEdge), DepType(DT) {}

  enum MemDepTypes getDepType() const { return DepType; }

  void print(raw_ostream &OS) const;

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWCtrlDep *A) { return true; }
  static inline bool classof(const HWEdge *A) {
    return A->getEdgeType() == edgeMemDep;
  }
};

/// @brief Base Class of all hardware atom. 
class HWAtom {
  // The time slot that this atom scheduled to.
  unsigned Latency;
  // TODO: typedef SlotType
  unsigned short SchedSlot;
  unsigned short InstIdx;
  unsigned FUNum;

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

  /// The corresponding Instructions - We may store several instruction inside
  /// the same schedule unit, so we can clamp them in a same slot.
  SmallVector<MachineInstr*, 2> Instrs;
  
public:
  static const unsigned short MaxSlot = ~0 >> 1;

  // Create the entry node.
  HWAtom(unsigned short Idx) : Latency(0), SchedSlot(0), InstIdx(Idx), FUNum(0) {
  }

  HWAtom(MachineInstr **I, unsigned NumInstrs, unsigned short latancy,
         unsigned short Idx, unsigned fuid = 0)
    : Latency(latancy), SchedSlot(0), InstIdx(Idx), FUNum(fuid),
    Instrs(I, I + NumInstrs) {}

  ~HWAtom() {
    std::for_each(Deps.begin(), Deps.end(), deleter<HWEdge>);
  }

  unsigned short getIdx() const { return InstIdx; }

  // Add a new depencence edge to the atom.
  void addDep(HWEdge *E) {
    E->getSrc()->addToUseList(this);
    Deps.push_back(E);
  }

  SmallVectorImpl<HWEdge*>::iterator edge_begin() { return Deps.begin(); }
  SmallVectorImpl<HWEdge*>::iterator edge_end() { return Deps.end(); }

  SmallVectorImpl<HWEdge*>::const_iterator edge_begin() const { return Deps.begin(); }
  SmallVectorImpl<HWEdge*>::const_iterator edge_end() const { return Deps.end(); }

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
  bool dep_empty() const { return Deps.empty(); }
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
  void dropAllReferences();
  void replaceAllUseBy(HWAtom *A);

  bool use_empty() { return UseList.empty(); }
  size_t getNumUses() const { return UseList.size(); }
  //}

  unsigned getSlot() const { return SchedSlot; }
  unsigned getFinSlot() const { return SchedSlot + Latency; }
  bool isScheduled() const { return SchedSlot != 0; }
  void scheduledTo(unsigned slot);
  void resetSchedule() { SchedSlot = 0; }

  // If this Schedule Unit is just the place holder for the Entry node.
  bool isEntry() const { return Instrs.empty(); }

  // Dirty Hack: Only return the first instruction.
  MachineInstr *getInstr() const {
    if (isEntry()) return 0;
      
    return Instrs.front();
  }

  typedef SmallVector<MachineInstr*, 2>::iterator instr_iterator;
  
  instr_iterator instr_begin() { return Instrs.begin(); }
  instr_iterator instr_end()   { return Instrs.end(); }

  typedef SmallVector<MachineInstr*, 2>::const_iterator const_instr_iterator;
  const_instr_iterator instr_begin() const { return Instrs.begin(); }
  const_instr_iterator instr_end()   const { return Instrs.end(); }

  unsigned getOpcode() const;
  VFUs::FUTypes getFUType() const;
  unsigned getFUNum() const { return FUNum; }
  FuncUnitId getFUId() const {
    return FuncUnitId(getFUType(), getFUNum());
  }

  /// print - Print out the internal representation of this atom to the
  /// specified stream.  This should really only be used for debugging
  /// purposes.
  virtual void print(raw_ostream &OS) const;

  /// dump - This method is used for debugging.
  ///
  void dump() const;
};

template<> struct GraphTraits<Inverse<HWAtom*> > {
  typedef HWAtom NodeType;
  typedef HWAtom::dep_iterator ChildIteratorType;
  static NodeType *getEntryNode(NodeType* N) { return N; }
  static inline ChildIteratorType child_begin(NodeType *N) {
    return N->dep_begin();
  }
  static inline ChildIteratorType child_end(NodeType *N) {
    return N->dep_end();
  }
};
template<> struct GraphTraits<Inverse<const HWAtom*> > {
  typedef const HWAtom NodeType;
  typedef HWAtom::const_dep_iterator ChildIteratorType;
  static NodeType *getEntryNode(NodeType* N) { return N; }
  static inline ChildIteratorType child_begin(NodeType *N) {
    return N->dep_begin();
  }
  static inline ChildIteratorType child_end(NodeType *N) {
    return N->dep_end();
  }
};
template<> struct GraphTraits<HWAtom*> {
  typedef HWAtom NodeType;
  typedef HWAtom::use_iterator ChildIteratorType;
  static NodeType *getEntryNode(NodeType* N) { return N; }
  static inline ChildIteratorType child_begin(NodeType *N) {
    return N->use_begin();
  }
  static inline ChildIteratorType child_end(NodeType *N) {
    return N->use_end();
  }
};
template<> struct GraphTraits<const HWAtom*> {
  typedef const HWAtom NodeType;
  typedef HWAtom::const_use_iterator ChildIteratorType;
  static NodeType *getEntryNode(NodeType* N) { return N; }
  static inline ChildIteratorType child_begin(NodeType *N) {
    return N->use_begin();
  }
  static inline ChildIteratorType child_end(NodeType *N) {
    return N->use_end();
  }
};

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

// Virtual Root
// FIXME: Do not derived from HWAtom.
class FSMState {
public:
  typedef std::vector<HWAtom*> AtomVecTy;
private:
  MachineBasicBlock *MBB;
  AtomVecTy Atoms;

  // Modulo for modulo schedule.
  unsigned short II;
  const unsigned short startSlot;
  bool HaveSelfLoop;
  const VTargetMachine &TM;

  /// Scheduling implementation.
  void scheduleLinear(ForceDirectedSchedulingBase *Scheduler);
  void scheduleLoop(ForceDirectedSchedulingBase *Scheduler,
                    unsigned II);

  typedef DenseMap<const MachineInstr*, HWAtom*> AtomMapType;
  AtomMapType InstToHWAtoms;
public:
  FSMState(const VTargetMachine &Target, MachineBasicBlock *MachBB,
           bool HaveSelfLoop, unsigned short StartSlot)
    : TM(Target), MBB(MachBB), II(0), startSlot(StartSlot),
    HaveSelfLoop(HaveSelfLoop) {}

  ~FSMState() {
    std::for_each(Atoms.begin(), Atoms.end(), deleter<HWAtom>);
  }

  MachineBasicBlock *getMachineBasicBlock() const { return MBB; }
  MachineBasicBlock *operator->() const { return getMachineBasicBlock(); }

  /// Mapping machine instruction to schedule unit, this will help us build the
  /// the dependences between schedule unit based on dependences between machine
  /// instructions.
  HWAtom *lookupAtom(const MachineInstr *MI) const {
    AtomMapType::const_iterator At = InstToHWAtoms.find(MI);
    return At != InstToHWAtoms.end() ? At->second : 0;
  }

  /// @name Roots
  //{
  HWAtom *getEntryRoot() const { return Atoms.front(); }
  HWAtom *getExitRoot() const { return Atoms.back(); }
  //}

  /// iterator/begin/end - Iterate over all schedule unit in the graph.
  typedef AtomVecTy::iterator iterator;
  iterator begin()  { return Atoms.begin(); }
  iterator end()    { return Atoms.end(); }

  typedef AtomVecTy::const_iterator const_iterator;
  const_iterator begin() const { return Atoms.begin(); }
  const_iterator end()   const { return Atoms.end(); }

  typedef AtomVecTy::reverse_iterator reverse_iterator;
  reverse_iterator rbegin()  { return Atoms.rbegin(); }
  reverse_iterator rend()    { return Atoms.rend(); }

  typedef AtomVecTy::const_reverse_iterator const_reverse_iterator;
  const_reverse_iterator rbegin() const { return Atoms.rbegin(); }
  const_reverse_iterator rend()   const { return Atoms.rend(); }

  void addAtom(HWAtom *A) {
    Atoms.push_back(A);
    for (HWAtom::instr_iterator I = A->instr_begin(), E = A->instr_end();
        I != E; ++I) {
      AtomMapType::iterator where;
      bool inserted;
      tie(where, inserted) = InstToHWAtoms.insert(std::make_pair(*I, A));
      assert(inserted && "Mapping from I already exist!");
    }
    
  }

  void eraseAtom(HWAtom *A) {
    iterator at = std::find(begin(), end(), A);
    assert(at != end() && "Can not find atom!");
    Atoms.erase(at);

    assert((std::find(usetree_iterator::begin(getEntryRoot()),
                      usetree_iterator::end(getEntryRoot()), A)
            == usetree_iterator::end(getEntryRoot())) && "Who using dead atom?");
  }

  size_t getNumAtoms() const { return Atoms.size(); }

  void resetSchedule() {
    for (iterator I = begin(), E = end(); I != E; ++I)
      (*I)->resetSchedule();
    getEntryRoot()->scheduledTo(startSlot);
  }

  unsigned getStartSlot() const { return getEntryRoot()->getSlot(); }
  unsigned getEndSlot() const { return getExitRoot()->getSlot(); }
  unsigned getTotalSlot() const { return getEndSlot() - getStartSlot() + 1; }

  // II for Modulo schedule
  void setII(unsigned ii) { II = ii; }
  void setNoOverlapII() { II = getTotalSlot() + 1; }
  bool isPipelined() const { return II != 0 && II != getTotalSlot() + 1; }
  unsigned getII() const { return II; }
  unsigned getIISlot() const { return getStartSlot() + II - 1; }
  bool haveSelfLoop() const { return HaveSelfLoop; }

  void print(raw_ostream &OS) const;
  void dump() const;
  void viewGraph();

  /// @name Scheduling
  //{
  void schedule();
  MachineBasicBlock *emitSchedule(BitLevelInfo &BLI);
  //}
};


template <> struct GraphTraits<FSMState*> : public GraphTraits<HWAtom*> {
  typedef FSMState::iterator nodes_iterator;
  static nodes_iterator nodes_begin(FSMState *G) {
    return G->begin();
  }
  static nodes_iterator nodes_end(FSMState *G) {
    return G->end();
  }
};

} // end namespace

#endif
