//===------------- VSUnit.h - Translate LLVM IR to VSUnit  -------*- C++ -*-===//
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
#include "llvm/ADT/ArrayRef.h"
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
    edgeCtrlDep,
    edgeFixedTiming
  };
private:
  uint8_t  EdgeType : 2;
  uint8_t  IsCrossBB : 1;
  // Iterate distance.
  uint16_t Distance : 13;
  // The latancy of this edge.
  uint16_t Latancy;

  friend class VSUnit;
  void setIsCrossBB(bool v = true) { IsCrossBB = v; }
protected:
  VDEdge(enum VDEdgeTypes T, unsigned latancy, unsigned Dst)
    : EdgeType(T), IsCrossBB(false), Distance(Dst), Latancy(latancy) {}
public:
  VDEdgeTypes getEdgeType() const { return VDEdgeTypes(EdgeType); }
  // Compute the latency considering the distance between iterations in a loop.
  inline int getLatency(unsigned II = 0) const {
    return int(Latancy) - int(II) * int(getDistance());
  }
  void setLatency(unsigned latency) { Latancy = latency; }
  // Get the distance between iterations in a loop.
  unsigned getDistance() const { return Distance; }
  bool isLoopCarried() const { return getDistance() > 0; }
  bool isCrossBB() const { return IsCrossBB; }

  void print(raw_ostream &OS) const;

  template<int DISTANCE>
  static VDEdge CreateMemDep(unsigned Latency) {
    return VDEdge(edgeMemDep, Latency, DISTANCE);
  }

  static VDEdge CreateMemDep(unsigned Latency, unsigned Distance) {
    return VDEdge(edgeMemDep, Latency, Distance);
  }

  static VDEdge CreateValDep(unsigned Latency) {
    return VDEdge(edgeValDep, Latency, 0);
  }

  static VDEdge CreateCtrlDep(unsigned Latency) {
    return VDEdge(edgeCtrlDep, Latency, 0);
  }

  static VDEdge CreateFixTimingConstraint(unsigned Latency) {
    return VDEdge(edgeFixedTiming, Latency, 0);
  }

  template<bool IsCtrl>
  static VDEdge CreateCtrlOrValDep(unsigned Latency) {
    return VDEdge(IsCtrl ? edgeCtrlDep : edgeValDep, Latency, 0);
  }
};

template<class IteratorType, bool IsConst>
class VSUnitDepIterator : public IteratorType {
  typedef VSUnitDepIterator<IteratorType, IsConst> Self;
  typedef typename conditional<IsConst, const VSUnit, VSUnit>::type NodeType;
public:
  VSUnitDepIterator(IteratorType i) : IteratorType(i) {}

  NodeType *operator*() const {
    return IteratorType::operator->()->first;
  }

  NodeType *operator->() const { return operator*(); }

  VDEdge getEdge() const { return IteratorType::operator->()->second; }

  Self& operator++() {                // Preincrement
    IteratorType::operator++();
    return *this;
  }

  Self operator++(int) { // Postincrement
    return IteratorType::operator++(0);
  }


  // Forwarding the function from the Edge.
  VDEdge::VDEdgeTypes getEdgeType() const { return getEdge().getEdgeType(); }
  inline unsigned getLatency(unsigned II = 0) const {
    return getEdge().getLatency(II);
  }
  bool isLoopCarried() const { return getEdge().isLoopCarried(); }
  bool isCrossBB() const { return getEdge().isCrossBB(); }
  unsigned getDistance() const { return getEdge().getDistance(); }
};

/// @brief Base Class of all hardware atom.
class VSUnit {
  // TODO: typedef SlotType
  unsigned SchedSlot : 30;
  bool     IsDangling : 1;
  bool     HasFixedTiming: 1;
  uint16_t InstIdx;
  uint16_t FUNum;

  // Remember the dependencies of the scheduling unit.
  typedef DenseMap<VSUnit*, VDEdge> DepSet;
  DepSet Deps;

  // The atoms that using this atom.
  typedef std::set<VSUnit*> UseListTy;
  UseListTy UseList;

  void addToUseList(VSUnit *User) {
    UseList.insert(User);
  }

  VSUnit(const VSUnit&);          // DO NOT IMPLEMENT
  void operator=(const VSUnit&);  // DO NOT IMPLEMENT

  /// The corresponding Instructions - We may store several instruction inside
  /// the same schedule unit, so we can clamp them in a same slot.
  SmallVector<InstPtrTy, 8> Instrs;
  // Latency from representative instruction, the latency of the SUnit is store
  // in latencies[0].
  SmallVector<int8_t, 8> latencies;

  friend class VSchedGraph;

  // Create the entry node.
  VSUnit(MachineBasicBlock *MBB, uint16_t Idx);

  VSUnit(unsigned short Idx, uint16_t FUNum);

  void addPtr(InstPtrTy Ptr, int8_t Latency) {
    assert((Instrs.empty() || getParentBB() == Ptr.getParent())
           && "Mixing instructions from different BB!");
    Instrs.push_back(Ptr);
    latencies.push_back(Latency);
  }

  VSUnit *updateIdx(unsigned short Idx);

  void cleanDepAndUse() {
    Deps.clear();
    UseList.clear();
  }
public:
  static const unsigned short MaxSlot = ~0 >> 1;

  unsigned short getIdx() const { return InstIdx; }
  bool isDangling() const { return IsDangling; }
  void setIsDangling(bool isDangling = true) { IsDangling = isDangling; }

  typedef DepSet::iterator edge_iterator;
  edge_iterator edge_begin() { return Deps.begin(); }
  edge_iterator edge_end() { return Deps.end(); }

  typedef DepSet::const_iterator const_edge_iterator;
  const_edge_iterator edge_begin() const { return Deps.begin(); }
  const_edge_iterator edge_end() const { return Deps.end(); }

  /// @name Operands
  //{
  // Add a new depencence edge to the atom.
  void addDep(VSUnit *Src, VDEdge NewE);

  typedef VSUnitDepIterator<edge_iterator, false> dep_iterator;
  dep_iterator dep_begin() { return Deps.begin(); }
  dep_iterator dep_end() { return Deps.end(); }

  typedef VSUnitDepIterator<const_edge_iterator, true> const_dep_iterator;
  const_dep_iterator dep_begin() const { return Deps.begin(); }
  const_dep_iterator dep_end() const { return Deps.end(); }

  size_t num_deps() const { return Deps.size(); }
  bool dep_empty() const { return Deps.empty(); }
  // If the current atom depend on A?
  bool isDepOn(const VSUnit *A) const { return getDepIt(A) != dep_end(); }

  // If this Depend on A? return the position if found, return dep_end otherwise.
  const_dep_iterator getDepIt(const VSUnit *A) const {
    return Deps.find(const_cast<VSUnit*>(A));
  }

  VDEdge getEdgeFrom(const VSUnit *A) const {
    assert(isDepOn(A) && "Current atom not depend on A!");
    return getDepIt(A).getEdge();
  }
  //}

  /// @name Use
  //{
  typedef UseListTy::iterator use_iterator;
  typedef UseListTy::const_iterator const_use_iterator;
  use_iterator use_begin() { return UseList.begin(); }
  const_use_iterator use_begin() const { return UseList.begin(); }
  use_iterator use_end() { return UseList.end(); }
  const_use_iterator use_end() const { return UseList.end(); }

  bool use_empty() const { return UseList.empty(); }
  size_t num_uses() const { return UseList.size(); }
  //}

  unsigned countValDeps() const;
  unsigned countValUses() const;

  // Dirty Hack: Only return the first instruction.
  InstPtrTy getRepresentativePtr() const {
    return Instrs.front();
  }

  InstPtrTy getPtrAt(unsigned Idx) const { return Instrs[Idx]; }

  bool isRepresentativeInst(MachineInstr *MI) const {
    return MI == getRepresentativePtr();
  }

  bool isBBEntry() const { return getRepresentativePtr().isMBB(); }
  bool isTerminator() const {
    return !isBBEntry() && getRepresentativePtr()->isTerminator();
  }

  MachineBasicBlock *getParentBB() const {
    return getRepresentativePtr().getParent();
  }

  // Get the latency from RepresentativeInst to MI.
  int8_t getLatencyFor(MachineInstr *MI) const;
  int8_t getLatencyAt(unsigned Idx) const {
    assert(Idx && "Cannot get latency at index 0!");
    return latencies[Idx];
  }

  // Get the total latency from the RepresentativeInst through SrcMI to DstMI.

  template<bool IsValDep>
  int getLatencyTo(MachineInstr *SrcMI, MachineInstr *DstMI, VSchedGraph &G) const;
  int getLatencyFrom(MachineInstr *SrcMI, int SrcLatency) const;

  // Get the maximum latency from RepresentativeInst to DstMI.
  template<bool IsValDep>
  int getMaxLatencyTo(MachineInstr *DstMI, VSchedGraph &G) const;

  typedef SmallVectorImpl<InstPtrTy>::iterator instr_iterator;

  size_t num_instrs() const { return Instrs.size(); }
  instr_iterator instr_begin() { return Instrs.begin(); }
  instr_iterator instr_end()   { return Instrs.end(); }

  typedef SmallVectorImpl<InstPtrTy>::const_iterator const_instr_iterator;
  const_instr_iterator instr_begin() const { return Instrs.begin(); }
  const_instr_iterator instr_end()   const { return Instrs.end(); }

  MachineInstr *instr_back() const { return Instrs.back(); }

  // If this Schedule Unit is just the place holder for the Entry node.
  bool isEntry() const { return getRepresentativePtr().isMBB(); }
  bool isPHI() const {
    if (MachineInstr *MI = getRepresentativePtr())
      return MI->isPHI();

    return false;
  }

  unsigned getLatency() const {
    return latencies.front();
  }

  void setLatency(unsigned L) {
    latencies[0] = L;
    assert(getLatency() == L && "Latency overflow!");
  }

  unsigned getSlot() const { return SchedSlot; }
  unsigned getFinSlot() const { return getSlot() + getLatency(); }

  bool isScheduled() const { return SchedSlot != 0; }
  void scheduledTo(unsigned slot);
  void resetSchedule() { SchedSlot = 0; }
  
  bool hasFixedTiming() const { return HasFixedTiming; }
  void setHasFixedTiming(bool has = true) { HasFixedTiming = has; }

  unsigned getOpcode() const;
  VFUs::FUTypes getFUType() const;
  bool isDatapath() const;
  bool isControl() const { return !isDatapath(); }

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

  // Index functor for VSUnit.
  template<typename T>
  struct IdxFunctor : public std::unary_function<const T*, unsigned> {
    unsigned operator()(const T *U) const {
      return U->getIdx();
    }
  };
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
  typedef SUnitVecTy::iterator iterator;
  enum { NullSUIdx = 0u, FirstSUIdx = 1u };
  DetialLatencyInfo &DLInfo;
private:
  SUnitVecTy AllSUs;
  // The VSUnits to schedule.
  ArrayRef<VSUnit*> SUsToSched;
  VSUnit *Exit;
  // The number of schedule unit.
  unsigned NextSUIdx;

  // The schedule unit that jump back to current fsm state.
  PointerIntPair<MachineInstr*, 1, bool> LoopOp;

  typedef std::map<InstPtrTy, VSUnit*> SUnitMapType;
  SUnitMapType InstToSUnits;
  typedef std::map<MachineBasicBlock*, VSUnit*> TerminatorMapTy;
  TerminatorMapTy Terminators;
  typedef std::map<MachineBasicBlock*, unsigned> IIMapTy;
  IIMapTy IIMap;

  bool trySetLoopOp(MachineInstr *MI);

  unsigned emitSchedule(iterator su_begin, iterator su_end, unsigned StartSlot,
                        MachineBasicBlock *MBB);
  void fixPHISchedules(iterator su_begin, iterator su_end);
public:
  const unsigned EntrySlot;

  VSchedGraph(DetialLatencyInfo &DLInfo, bool EnablePipeline, unsigned EntrySlot)
    : DLInfo(DLInfo), Exit(0), NextSUIdx(FirstSUIdx), LoopOp(0, EnablePipeline),
      EntrySlot(EntrySlot) {}

  ~VSchedGraph() {
    std::for_each(AllSUs.begin(), AllSUs.end(), deleter<VSUnit>);
    delete Exit;
  }

  // Forwarding function from DetialLatencyInfo.
  unsigned getStepsToFinish(const MachineInstr *MI) const {
    return DLInfo.getStepsToFinish(MI);
  }

  void addDummyLatencyEntry(const MachineInstr *MI, float l = 0.0f) {
    DLInfo.addDummyLatencyEntry(MI, l);
  }

  typedef DetialLatencyInfo::DepLatInfoTy DepLatInfoTy;
  const DepLatInfoTy *getDepLatInfo(const MachineInstr *DstMI) const {
    return DLInfo.getDepLatInfo(DstMI);
  }


  float getChainingLatency(const MachineInstr *SrcInstr,
                           const MachineInstr *DstInstr) const {
    return DLInfo.getChainingLatency(SrcInstr, DstInstr);
  }

  void addInstr(const MachineInstr *MI) {
    DLInfo.addInstr(MI);
  }

  void buildExitMIInfo(const MachineInstr *ExitMI, DepLatInfoTy &Info) {
    DLInfo.buildExitMIInfo(ExitMI, Info);
  }

  // Erase the instructions from exit set.
  void eraseFromWaitSet(const MachineInstr *MI) {
    DLInfo.eraseFromWaitSet(MI);
  }

  template<bool IsValDep>
  unsigned getCtrlStepBetween(const MachineInstr *SrcInstr,
                              const MachineInstr *DstInstr) {
    return DLInfo.getCtrlStepBetween<IsValDep>(SrcInstr, DstInstr);
  }

  unsigned getStepsFromEntry(const MachineInstr *DstInstr) const {
    return DLInfo.getStepsFromEntry(DstInstr);
  }

  // Verify the schedule graph, should be call after the graph is built.
  void verify() const;
  void verifySU(const VSUnit *SU) const;

  // Add the the iterator point to the first newly added SU.
  iterator mergeSUsInSubGraph(VSchedGraph &SubGraph);

  // VSUnit Creating/Mapping/Merging
  bool mapMI2SU(InstPtrTy Ptr, VSUnit *SU, int8_t latency) {
    if (SU->num_instrs() && Ptr.isMI()
        && SU->isDatapath() != VInstrInfo::isDatapath(Ptr->getOpcode()))
      return false;

    SU->addPtr(Ptr, latency);
    SUnitMapType::iterator where;
    bool inserted;
    tie(where, inserted) = InstToSUnits.insert(std::make_pair(Ptr, SU));
    assert(inserted && "Mapping from I already exist!");
    return true;
  }

  // Sort the schedule units to place control operations at the beginning of
  // the SU list, so we can only schedule the control operations
  void prepareForCtrlSched();
  // Extend the to schedule SU list to all SU in current schedule graph.
  void prepareForDatapathSched();

  void topologicalSortScheduleUnits();

  VSUnit *createTerminator(MachineBasicBlock *MBB) {
    VSUnit *&SU = Terminators[MBB];
    assert(SU == 0 && "Terminator already exist!");
    SU = new VSUnit(NextSUIdx++, 0);

    AllSUs.push_back(SU);
    return SU;
  }

  VSUnit *lookUpTerminator(MachineBasicBlock *MBB) const {
    TerminatorMapTy::const_iterator at = Terminators.find(MBB);
    return at == Terminators.end() ? 0 : at->second;
  }

  unsigned num_bbs() const { return Terminators.size(); }

  VSUnit *createVSUnit(InstPtrTy Ptr, unsigned fuid = 0);
  VSUnit *createExitRoot(MachineBasicBlock *MBB) {
    assert (Exit == 0 && "Exit already created!");
    Exit = new VSUnit(NextSUIdx++, 0);
    Exit->addPtr(MBB, 0);

    AllSUs.push_back(Exit);

    typedef TerminatorMapTy::iterator it;
    for (it I = Terminators.begin(), E = Terminators.end(); I != E; ++I)
      if (I->second->use_empty())
        Exit->addDep(I->second, VDEdge::CreateCtrlDep(0));

    return Exit;
  }

  bool isLoopOp(MachineInstr *MI) {
    assert(MI->getDesc().isTerminator() && "Expected terminator!");

    // Set the current instruction as loop operation if it is jumping back
    // to the current state and we want to pipeline the state.
    if (trySetLoopOp(MI) && enablePipeLine())
      return true;

    return false;
  }

  bool isLoopPHIMove(MachineInstr *MI);

  /// Mapping machine instruction to schedule unit, this will help us build the
  /// the dependences between schedule unit based on dependences between machine
  /// instructions.
  VSUnit *lookupSUnit(InstPtrTy Ptr) const {
    SUnitMapType::const_iterator at = InstToSUnits.find(Ptr);
    return at == InstToSUnits.end() ? 0 : at->second;
  }

  /// @name Roots
  //{
  VSUnit *getEntryRoot() const { return AllSUs.front(); }
  MachineBasicBlock *getEntryBB() const {
    return getEntryRoot()->getParentBB();
  }
  VSUnit *getExitRoot() const { return Exit; }
  //}

  /// iterator/begin/end - Iterate over all schedule unit in the graph.
  iterator begin() { return AllSUs.begin(); }
  iterator end() { return AllSUs.end(); }
  typedef ArrayRef<VSUnit*>::iterator sched_iterator;
  sched_iterator sched_begin()  const { return SUsToSched.begin(); }
  sched_iterator sched_end()    const { return SUsToSched.end(); }
  size_t num_scheds() const { return SUsToSched.size(); }
  unsigned getNextSUIdx() const { return NextSUIdx; }
  VSUnit *getCtrlAt(unsigned Idx) const { return SUsToSched[Idx]; }
  void resetSchedule(unsigned MII);

  unsigned getStartSlot(MachineBasicBlock *MBB) const {
    VSUnit *EntrySU = lookupSUnit(MBB);
    assert(EntrySU && "Cannot find entry!");
    return EntrySU->getSlot();
  }

  unsigned getEndSlot(MachineBasicBlock *MBB) const {
    VSUnit *Terminator = lookUpTerminator(MBB);
    assert(Terminator && "Cannot find terminator!");
    return Terminator->getSlot();
  }

  unsigned getTotalSlot(MachineBasicBlock *MBB) const {
    return getEndSlot(MBB) - getStartSlot(MBB);
  }

  // II for Modulo schedule
  bool isPipelined(MachineBasicBlock *MBB) const {
    return getII(MBB) < getTotalSlot(MBB);
  }

  unsigned getLoopOpSlot(MachineBasicBlock *MBB) const {
    return getStartSlot(MBB) + getII(MBB);
  }

  unsigned getII(MachineBasicBlock *MBB) const {
    IIMapTy::const_iterator at = IIMap.find(MBB);
    return at == IIMap.end() ? getTotalSlot(MBB) : at->second;
  }

  bool enablePipeLine() const {
    return LoopOp.getInt();
  }

  bool hasLoopOp() const { return LoopOp.getPointer() != 0; }
  VSUnit *getLoopOp() const {
    if (MachineInstr *MI = LoopOp.getPointer())
      return lookupSUnit(MI);

    return 0;
  }

  void print(raw_ostream &OS) const;
  void dump() const;
  void viewGraph();

  /// @name Scheduling
  //{
  void scheduleLoop();
  // Schedule datapath operations as late as possible after control operations
  // scheduled, this can reduce register usage.
  void scheduleDatapath();
  void scheduleDatapathALAP();
  void scheduleDatapathASAP();
  unsigned emitSchedule();
  //}

  // If a datapath operation is chained with a non-trivial control operation,
  // copy its result to register, otherwise the result register of the function
  // unit of the control operation may have a long live interval and hard to
  // be shared.
  void fixChainedDatapathRC(VSUnit *U);
};

template <> struct GraphTraits<VSchedGraph*> : public GraphTraits<VSUnit*> {
  typedef VSchedGraph::sched_iterator nodes_iterator;
  static nodes_iterator nodes_begin(VSchedGraph *G) {
    return G->sched_begin();
  }
  static nodes_iterator nodes_end(VSchedGraph *G) {
    return G->sched_end();
  }
};

template<bool IsValDep>
int VSUnit::getLatencyTo(MachineInstr *SrcMI, MachineInstr *DstMI,
                         VSchedGraph &G) const {
  int Latency = G.getCtrlStepBetween<IsValDep>(SrcMI, DstMI);
  if (SrcMI != getRepresentativePtr()) {
    Latency += getLatencyFor(SrcMI);
  }

  return Latency;
}

template<bool IsValDep>
int VSUnit::getMaxLatencyTo(MachineInstr *DstMI, VSchedGraph &G) const {
  int latency = 0;
  for (const_instr_iterator I = instr_begin(), E = instr_end(); I != E; ++I)
    // Also compute the latency to DstMI even *I (SrcMI) is 0, which means the
    // source is the entry root of the state.
    latency = std::max(getLatencyTo<IsValDep>(*I, DstMI, G), latency);

  return latency;
}

} // end namespace

#endif
