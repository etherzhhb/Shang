//===------------- VSUnit.h - Translate LLVM IR to VSUnit  -------*- C++ -*-===//
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
// This file define the VSUnit class, which represent the basic atom operation
// in hardware.
//
//===----------------------------------------------------------------------===//
//

#ifndef VBE_HARDWARE_ATOM_H
#define VBE_HARDWARE_ATOM_H

#include "vtm/VInstrInfo.h"
#include "vtm/FUInfo.h"

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
class SchedulingBase;
class FuncUnitId;
class VSUnit;
class VSchedGraph;

class MachineBasicBlock;
class MachineInstr;
class MachineOperand;

/// @brief Inline operation
class VDEdge {
public:
  enum VDEdgeTypes {
    edgeValDep,
    edgeMemDep,
    edgeCtrlDep
  };
private:
  PointerIntPair<VSUnit*, 2, VDEdgeTypes> Src;
  // Iterate distance.
  const unsigned short ItDst;
  // The latancy of this edge.
  const unsigned short Latancy;
  
  VDEdge(const VDEdge &);            // DO NOT IMPLEMENT
  void operator=(const VDEdge &);    // DO NOT IMPLEMENT
protected:
  VDEdge(enum VDEdgeTypes T, VSUnit *src, unsigned latancy, unsigned Dst)
    : Src(src, T), ItDst(Dst), Latancy(latancy) {}
public:
  // The referenced value.
  VSUnit *getSrc() const { return Src.getPointer(); }
  VSUnit* operator->() const { return getSrc(); }
  //VSUnit* operator*() const { return getSrc(); }
  unsigned getEdgeType() const { return Src.getInt(); }
  unsigned getLatency() const { return (Latancy + 1) / 2; }
  unsigned getDetailLatency() const { return Latancy; }
  unsigned getItDst() const { return ItDst; }
  bool isLoopCarried() const { return getItDst() > 0; }

  void print(raw_ostream &OS) const;
};

template<class IteratorType, class NodeType>
class VSUnitDepIterator : public std::iterator<std::forward_iterator_tag,
                                               NodeType*, ptrdiff_t> {
    IteratorType I;   // std::vector<MSchedGraphEdge>::iterator or const_iterator
    typedef VSUnitDepIterator<IteratorType, NodeType> Self;
public:
  VSUnitDepIterator(IteratorType i) : I(i) {}

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
  VSUnitDepIterator operator++(int) { // Postincrement
    VSUnitDepIterator tmp = *this;
    ++*this;
    return tmp; 
  }

  VDEdge *getEdge() { return *I; }
  const VDEdge *getEdge() const { return *I; }
};

/// @brief Value Dependence Edge.
class VDValDep : public VDEdge {
public:
  VDValDep(VSUnit *Src, unsigned latancy)
    : VDEdge(edgeValDep, Src, latancy,  0) {}

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const VDValDep *A) { return true; }
  static inline bool classof(const VDEdge *A) {
    return A->getEdgeType() == edgeValDep;
  }
};

class VDCtrlDep : public VDEdge {
public:
  VDCtrlDep(VSUnit *Src, unsigned latancy)
    : VDEdge(edgeCtrlDep, Src, latancy, 0) {}

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const VDCtrlDep *A) { return true; }
  static inline bool classof(const VDEdge *A) {
    return A->getEdgeType() == edgeCtrlDep;
  }
};

class VDMemDep : public VDEdge {
public:
  VDMemDep(VSUnit *Src, unsigned latancy, unsigned Dist)
    : VDEdge(edgeMemDep, Src, latancy, Dist) {}

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const VDCtrlDep *A) { return true; }
  static inline bool classof(const VDEdge *A) {
    return A->getEdgeType() == edgeMemDep;
  }
};

class OpSlot {
  int SlotNum;
  OpSlot(int S) : SlotNum(S) {}
  enum SlotType {Control, Datapath};
public:
  OpSlot() : SlotNum(0) {}
  OpSlot(int Slot, bool isCtrl) {
    SlotType T = isCtrl ? Control : Datapath;
    SlotNum = (Slot << 0x1) | (0x1 & T);
  }

  SlotType getSlotType() const {
    return (SlotType)(SlotNum & 0x1);
  }

  bool isControl() const {
    return getSlotType() == OpSlot::Control;
  }

  bool isDatapath() const {
    return getSlotType() == OpSlot::Datapath;
  }

  int getSlot() const { return SlotNum / 2; }

  inline bool operator==(OpSlot S) const {
    return SlotNum == S.SlotNum;
  }

  inline bool operator!=(OpSlot S) const {
    return SlotNum != S.SlotNum;
  }

  inline bool operator<(OpSlot S) const {
    return SlotNum < S.SlotNum;
  }

  inline bool operator<=(OpSlot S) const {
    return SlotNum <= S.SlotNum;
  }

  inline bool operator>(OpSlot S) const {
    return SlotNum > S.SlotNum;
  }
  inline bool operator>=(OpSlot S) const {
    return SlotNum >= S.SlotNum;
  }

  inline OpSlot &setSlot(unsigned RHS) {
    unsigned T = SlotNum & 0x1;
    SlotNum = (RHS << 0x1) | T;
    return *this;
  }

  void setType(bool isCtrl) {
    SlotType T = isCtrl ? Control : Datapath;
    SlotNum = (getSlot() << 0x1) | (0x1 & T);
  }

  inline OpSlot operator+(unsigned RHS) const {
    return OpSlot(getSlot() + RHS, isControl());
  }

  inline OpSlot &operator+=(unsigned RHS) {
    SlotNum += RHS * 2;
    return *this;
  }

  inline OpSlot &operator++() { return operator+=(1); }

  inline OpSlot operator++(int) {
    OpSlot Temp = *this;
    SlotNum += 2;
    return Temp;
  }

  OpSlot getNextSlot() const { return OpSlot(SlotNum + 1); }
  OpSlot getPrevSlot() const { return OpSlot(SlotNum - 1); }

  int getDetailStep() const { return SlotNum; }

  static OpSlot detailStepCeil(int S, bool isDatapath);
  static OpSlot detailStepFloor(int S, bool isDatapath);
};

/// @brief Base Class of all hardware atom. 
class VSUnit {
  // TODO: typedef SlotType
  OpSlot SchedSlot;
  unsigned short InstIdx;
  unsigned short FUNum;

  /// First of all, we schedule all atom base on dependence
  SmallVector<VDEdge*, 4> Deps;

  // The atoms that using this atom.
  std::list<VSUnit*> UseList;

  void addToUseList(VSUnit *User) {
    UseList.push_back(User);
  }

  VSUnit(const VSUnit&);          // DO NOT IMPLEMENT
  void operator=(const VSUnit&);  // DO NOT IMPLEMENT

  /// The corresponding Instructions - We may store several instruction inside
  /// the same schedule unit, so we can clamp them in a same slot.
  SmallVector<MachineInstr*, 8> Instrs;
  // Latency from representative instruction.
  SmallVector<int8_t, 8> latencies;

  friend class VSchedGraph;

  // Create the entry node.
  VSUnit(unsigned short Idx) : InstIdx(Idx), FUNum(0) {
    addInstr(0, 0);
    SchedSlot.setType(!hasDatapath());
  }

  VSUnit(bool datapath, unsigned short Idx, unsigned fuid)
    : InstIdx(Idx), FUNum(fuid)
  {
    SchedSlot.setType(!datapath);
  }

  void addInstr(MachineInstr *I, int8_t Latency) {
    Instrs.push_back(I);
    latencies.push_back(Latency);
  }

  VSUnit *updateIdx(unsigned short Idx) {
    InstIdx = Idx;
    return this;
  }

public:
  static const unsigned short MaxSlot = ~0 >> 1;

  ~VSUnit() {
    std::for_each(Deps.begin(), Deps.end(), deleter<VDEdge>);
  }

  unsigned short getIdx() const { return InstIdx; }

  // Add a new depencence edge to the atom.
  void addDep(VDEdge *E) {
    E->getSrc()->addToUseList(this);
    Deps.push_back(E);
  }

  typedef SmallVectorImpl<VDEdge*>::iterator edge_iterator;
  edge_iterator edge_begin() { return Deps.begin(); }
  edge_iterator edge_end() { return Deps.end(); }

  typedef SmallVectorImpl<VDEdge*>::const_iterator const_edge_iterator;
  const_edge_iterator edge_begin() const { return Deps.begin(); }
  const_edge_iterator edge_end() const { return Deps.end(); }

  /// @name Operands
  //{
  VDEdge &getDep(unsigned i) const { return *Deps[i]; }

  typedef VSUnitDepIterator<SmallVectorImpl<VDEdge*>::iterator, VSUnit>
    dep_iterator;
  typedef VSUnitDepIterator<SmallVectorImpl<VDEdge*>::const_iterator, const VSUnit>
    const_dep_iterator;

  dep_iterator dep_begin() { return Deps.begin(); }
  dep_iterator dep_end() { return Deps.end(); }
  const_dep_iterator dep_begin() const { return Deps.begin(); }
  const_dep_iterator dep_end() const { return Deps.end(); }

  size_t getNumDeps() const { return Deps.size(); }
  bool dep_empty() const { return Deps.empty(); }
  // If the current atom depend on A?
  bool isDepOn(const VSUnit *A) const { return getDepIt(A) != dep_end(); }

  // If this Depend on A? return the position if found, return dep_end otherwise.
  const_dep_iterator getDepIt(const VSUnit *A) const {
    for (const_dep_iterator I = dep_begin(), E = dep_end(); I != E; ++I)
      if ((*I) == A)
        return I;

    return dep_end();
  }

  dep_iterator getDepIt(const VSUnit *A) {
    for (dep_iterator I = dep_begin(), E = dep_end(); I != E; ++I)
      if ((*I) == A)
        return I;
    
    return dep_end();
  }

  VDEdge *getEdgeFrom(const VSUnit *A) {
    assert(isDepOn(A) && "Current atom not depend on A!");
    return getDepIt(A).getEdge();
  }
  VDEdge *getEdgeFrom(const VSUnit *A) const {
    assert(isDepOn(A) && "Current atom not depend on A!");
    return getDepIt(A).getEdge();
  }
  //}

  /// @name Use
  //{
  typedef std::list<VSUnit*>::iterator use_iterator;
  typedef std::list<VSUnit*>::const_iterator const_use_iterator;
  use_iterator use_begin() { return UseList.begin(); }
  const_use_iterator use_begin() const { return UseList.begin(); }
  use_iterator use_end() { return UseList.end(); }
  const_use_iterator use_end() const { return UseList.end(); }

  VSUnit *use_back() { return UseList.back(); }
  VSUnit *use_back() const { return UseList.back(); }

  bool use_empty() { return UseList.empty(); }
  size_t getNumUses() const { return UseList.size(); }
  //}

  // Dirty Hack: Only return the first instruction.
  MachineInstr *getRepresentativeInst() const {
    return Instrs.front();
  }

  bool isRepresentativeInst(MachineInstr *MI) const {
    return MI == getRepresentativeInst();
  }

  size_t num_instrs() const { return Instrs.size(); }

  // Get the latency from RepresentativeInst to MI.
  int8_t getLatencyFor(MachineInstr *MI) const;
  int8_t getLatencyAt(unsigned Idx) const {
    return latencies[Idx];
  }

  // Get the total latency from the RepresentativeInst through SrcMI to DstMI.
  unsigned getLatencyTo(MachineInstr *SrcMI, MachineInstr *DstMI) const;
  
  // Get the maximum latency from RepresentativeInst to DstMI.
  unsigned getMaxLatencyTo(MachineInstr *DstMI) const {
    unsigned latency = 0;
    for (const_instr_iterator I = instr_begin(), E = instr_end(); I != E; ++I)
      latency = std::max(getLatencyTo(*I, DstMI), latency);
    
    return latency;
  }

  typedef SmallVectorImpl<MachineInstr*>::iterator instr_iterator;

  instr_iterator instr_begin() { return Instrs.begin(); }
  instr_iterator instr_end()   { return Instrs.end(); }

  typedef SmallVectorImpl<MachineInstr*>::const_iterator const_instr_iterator;
  const_instr_iterator instr_begin() const { return Instrs.begin(); }
  const_instr_iterator instr_end()   const { return Instrs.end(); }

  // If this Schedule Unit is just the place holder for the Entry node.
  bool isEntry() const { return getRepresentativeInst() == 0; }

  unsigned getLatency() const {
    if (isEntry()) return 0;
  
    VIDesc Info = *getRepresentativeInst();
    return Info.getLatency();
  }

  unsigned getSlot() const { return SchedSlot.getSlot(); }
  unsigned getDetailStep() const {return SchedSlot.getDetailStep(); }

  bool isScheduled() const { return SchedSlot.getSlot() != 0; }
  void scheduledTo(unsigned slot);
  void resetSchedule() { SchedSlot.setSlot(0); }

  unsigned getOpcode() const;
  VFUs::FUTypes getFUType() const;
  bool hasDatapath() const;

  unsigned getFUNum() const { return FUNum; }
  FuncUnitId getFUId() const {
    return FuncUnitId(getFUType(), getFUNum());
  }

  /// print - Print out the internal representation of this atom to the
  /// specified stream.  This should really only be used for debugging
  /// purposes.
  void print(raw_ostream &OS) const;

  /// dump - This method is used for debugging.
  ///
  void dump() const;
};

template<> struct GraphTraits<Inverse<VSUnit*> > {
  typedef VSUnit NodeType;
  typedef VSUnit::dep_iterator ChildIteratorType;
  static NodeType *getEntryNode(NodeType* N) { return N; }
  static inline ChildIteratorType child_begin(NodeType *N) {
    return N->dep_begin();
  }
  static inline ChildIteratorType child_end(NodeType *N) {
    return N->dep_end();
  }
};
template<> struct GraphTraits<Inverse<const VSUnit*> > {
  typedef const VSUnit NodeType;
  typedef VSUnit::const_dep_iterator ChildIteratorType;
  static NodeType *getEntryNode(NodeType* N) { return N; }
  static inline ChildIteratorType child_begin(NodeType *N) {
    return N->dep_begin();
  }
  static inline ChildIteratorType child_end(NodeType *N) {
    return N->dep_end();
  }
};
template<> struct GraphTraits<VSUnit*> {
  typedef VSUnit NodeType;
  typedef VSUnit::use_iterator ChildIteratorType;
  static NodeType *getEntryNode(NodeType* N) { return N; }
  static inline ChildIteratorType child_begin(NodeType *N) {
    return N->use_begin();
  }
  static inline ChildIteratorType child_end(NodeType *N) {
    return N->use_end();
  }
};
template<> struct GraphTraits<const VSUnit*> {
  typedef const VSUnit NodeType;
  typedef VSUnit::const_use_iterator ChildIteratorType;
  static NodeType *getEntryNode(NodeType* N) { return N; }
  static inline ChildIteratorType child_begin(NodeType *N) {
    return N->use_begin();
  }
  static inline ChildIteratorType child_end(NodeType *N) {
    return N->use_end();
  }
};

// Use tree iterator.
typedef df_iterator<VSUnit*, SmallPtrSet<VSUnit*, 8>, false,
  GraphTraits<VSUnit*> > usetree_iterator;
typedef df_iterator<const VSUnit*, SmallPtrSet<const VSUnit*, 8>, false,
  GraphTraits<const VSUnit*> > const_usetree_iterator;

// Predecessor tree iterator, travel the tree from exit node.
typedef df_iterator<VSUnit*, SmallPtrSet<VSUnit*, 8>, false,
  GraphTraits<Inverse<VSUnit*> > > deptree_iterator;

typedef df_iterator<const VSUnit*, SmallPtrSet<const VSUnit*, 8>, false,
  GraphTraits<Inverse<const VSUnit*> > > const_deptree_iterator;


class VSchedGraph {
public:
  typedef std::vector<VSUnit*> SUnitVecTy;
private:
  const TargetMachine &TM;
  MachineBasicBlock *MBB;
  SUnitVecTy SUnits;
  // The number of schedule unit.
  unsigned SUCount;

  const unsigned startSlot;
  // The schedule unit that jump back to current fsm state.
  PointerIntPair<MachineInstr*, 1, bool> LoopOp;

  /// Scheduling implementation.
  void scheduleLinear();
  void scheduleLoop();

  typedef DenseMap<const MachineInstr*, VSUnit*> SUnitMapType;
  SUnitMapType InstToSUnits;

  bool trySetLoopOp(MachineInstr *MI);

  unsigned computeRecMII();
  unsigned computeResMII();
  unsigned computeMII();

  VSUnit *createEntry() {
    VSUnit *Entry = new VSUnit(SUCount);
    ++SUCount;
    SUnits.push_back(Entry);
    return Entry;
  }

public:
  VSchedGraph(const TargetMachine &Target, MachineBasicBlock *MachBB,
              bool HaveLoopOp, unsigned short StartSlot)
    : TM(Target), MBB(MachBB), SUCount(0), startSlot(StartSlot),
      LoopOp(0, HaveLoopOp) {
    // Create a dummy entry node.
    (void) createEntry();
  }

  ~VSchedGraph() {
    std::for_each(SUnits.begin(), SUnits.end(), deleter<VSUnit>);
  }

  void mapMI2SU(MachineInstr *MI, VSUnit *SU, int8_t latency) {
    SU->addInstr(MI, latency);
    SUnitMapType::iterator where;
    bool inserted;
    tie(where, inserted) = InstToSUnits.insert(std::make_pair(MI, SU));
    assert(inserted && "Mapping from I already exist!");
  }

  // Merge Src into Dst with a given latency.
  void mergeSU(VSUnit *Src, VSUnit *Dst, int8_t Latency);
  void removeDeadSU();

  VSUnit *createVSUnit(MachineInstr *I, unsigned fuid = 0);

  bool eatTerminator(MachineInstr *MI) {
    if (!MI->getDesc().isTerminator())
      return false;

    // Do not eat the current instruction as terminator if it is jumping back
    // to the current state and we want to pipeline the state.
    if (trySetLoopOp(MI) && enablePipeLine())
      return false;

    return true;
  }

  MachineBasicBlock *getMachineBasicBlock() const { return MBB; }
  MachineBasicBlock *operator->() const { return getMachineBasicBlock(); }

  /// Mapping machine instruction to schedule unit, this will help us build the
  /// the dependences between schedule unit based on dependences between machine
  /// instructions.
  VSUnit *lookupSUnit(const MachineInstr *MI) const {
    SUnitMapType::const_iterator At = InstToSUnits.find(MI);
    return At != InstToSUnits.end() ? At->second : 0;
  }

  /// @name Roots
  //{
  VSUnit *getEntryRoot() const { return SUnits.front(); }
  VSUnit *getExitRoot() const { return SUnits.back(); }
  //}

  /// iterator/begin/end - Iterate over all schedule unit in the graph.
  typedef SUnitVecTy::iterator iterator;
  iterator begin()  { return SUnits.begin(); }
  iterator end()    { return SUnits.end(); }

  typedef SUnitVecTy::const_iterator const_iterator;
  const_iterator begin() const { return SUnits.begin(); }
  const_iterator end()   const { return SUnits.end(); }

  typedef SUnitVecTy::reverse_iterator reverse_iterator;
  reverse_iterator rbegin()  { return SUnits.rbegin(); }
  reverse_iterator rend()    { return SUnits.rend(); }

  typedef SUnitVecTy::const_reverse_iterator const_reverse_iterator;
  const_reverse_iterator rbegin() const { return SUnits.rbegin(); }
  const_reverse_iterator rend()   const { return SUnits.rend(); }

  size_t getNumSUnits() const { return SUnits.size(); }

  void resetSchedule();

  unsigned getStartSlot() const { return startSlot; }
  unsigned getEndSlot() const { return getExitRoot()->getSlot(); }
  unsigned getTotalSlot() const { return getEndSlot() - getStartSlot(); }

  // II for Modulo schedule
  bool isPipelined() const {
    return getII() < getTotalSlot();
  }

  unsigned getLoopOpSlot() const {
    if (VSUnit *SE = getLoopOp())
      return SE->getSlot();
    
    return getEndSlot();
  }
  unsigned getII() const { return getLoopOpSlot() - getStartSlot(); }

  bool enablePipeLine() const {
    return LoopOp.getInt();
  }

  bool hasLoopOp() const { return LoopOp.getPointer() != 0; }
  VSUnit *getLoopOp() const {
    return lookupSUnit(LoopOp.getPointer());
  }

  void print(raw_ostream &OS) const;
  void dump() const;
  void viewGraph();

  /// @name Scheduling
  //{
  // Sort the schedule units base on the order of underlying instruction.
  void preSchedTopSort();
  void schedule();
  void emitSchedule();
  //}
};


template <> struct GraphTraits<VSchedGraph*> : public GraphTraits<VSUnit*> {
  typedef VSchedGraph::iterator nodes_iterator;
  static nodes_iterator nodes_begin(VSchedGraph *G) {
    return G->begin();
  }
  static nodes_iterator nodes_end(VSchedGraph *G) {
    return G->end();
  }
};

} // end namespace

#endif
