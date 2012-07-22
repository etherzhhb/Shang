//===- ForceDirectedSchedulingBase.h - ForceDirected information analyze --*- C++ -*-===//
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
// This file define the Force Direct information computation pass describe in
// Force-Directed Scheduling for the Behavioral Synthesis of ASIC's
//
//===----------------------------------------------------------------------===//
#ifndef VBE_FORCE_DIRECTED_INFO
#define VBE_FORCE_DIRECTED_INFO

#include "VSUnit.h"

#include "llvm/ADT/PriorityQueue.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/SmallSet.h"
#include <map>
using namespace llvm;

//Dirty Hack
struct _lprec;
typedef _lprec lprec;

namespace llvm {
class SchedulingBase {
  // MII in modulo schedule.
  const unsigned EntrySlot;
  unsigned MII, CriticalPathEnd;
  // Time Frame {asap step, alap step }
public:
  typedef std::pair<unsigned, unsigned> TimeFrame;
private:
  // Time frames for each schedule unit.
  typedef std::map<const VSUnit*, TimeFrame> TFMapTy;
  TFMapTy SUnitToTF;

  typedef SmallVector<MachineInstr*, 4> InstSetTy;
  static InstSetTy::const_iterator findConflictedInst(const InstSetTy &Set,
                                                      MachineInstr *MI);
  static MachineInstr *getConflictedInst(const InstSetTy &S, MachineInstr *MI) {
    InstSetTy::const_iterator at = findConflictedInst(S, MI);
    return at != S.end() ? *at : 0;
  }

  bool hasConflictedInst(const InstSetTy &Set, MachineInstr *MI) {
    // Nothing to conflict if the set is empty.
    if (Set.empty()) return false;

    // Even the two MIs not in the same trace, the different trace may active at
    // the same time in different iterations.
    if (State.enablePipeLine()) return true;

    return findConflictedInst(Set, MI) != Set.end();
  }

  // Remember the Instructions those are scheduled to a particular step.
  // (FU, Step) -> Instructions.
  typedef std::map<std::pair<FuncUnitId, unsigned>, InstSetTy> RTType;
  RTType RT;

  // Do not mixing pipeline stage with variable latency FUs.
  typedef DenseMap<unsigned, InstSetTy> PipelineStatusMap;
  PipelineStatusMap PipeFUs, PipeBreakerFUs;
protected:
  /// @name PriorityQueue
  //{

  VSchedGraph &State;
  unsigned computeStepKey(unsigned step) const;
  SchedulingBase(VSchedGraph &S)
    : EntrySlot(S.EntrySlot), MII(0), CriticalPathEnd(0),
      State(S) {}

public:
  virtual ~SchedulingBase() {}

  VSchedGraph &getState() const { return State; }
  VSchedGraph *operator->() const { return &State; }

  virtual bool scheduleState() = 0;
  // Return true when resource constraints preserved after citical path
  // scheduled
  bool scheduleCriticalPath(bool refreshFDepHD);
  void schedulePassiveSUnits();
  bool allNodesSchedued() const;

  /// @name TimeFrame
  //{
  typedef VSUnit::dep_iterator dep_it;
  typedef VSUnit::const_dep_iterator const_dep_it;
  typedef VSUnit::use_iterator use_it;
  typedef VSUnit::const_use_iterator const_use_it;
  unsigned calculateASAP(const VSUnit *A);
  void buildASAPStep();
  unsigned calculateALAP(const VSUnit *A);
  void buildALAPStep();
  void buildTimeFrame();

  unsigned getASAPStep(const VSUnit *A) const {
    std::map<const VSUnit*, TimeFrame>::const_iterator at = SUnitToTF.find(A);
    assert(at != SUnitToTF.end() && "TimeFrame for SU not exist!");
    return at->second.first;
  }
  unsigned getALAPStep(const VSUnit *A) const {
    std::map<const VSUnit*, TimeFrame>::const_iterator at = SUnitToTF.find(A);
    assert(at != SUnitToTF.end() && "TimeFrame for SU not exist!");
    return at->second.second;
  }

  unsigned getTimeFrame(const VSUnit *A) const {
    return getALAPStep(A) - getASAPStep(A) + 1;
  }

  void printTimeFrame(raw_ostream &OS) const;
  void dumpTimeFrame() const;
  //}

  void resetRT() {
    for (RTType::iterator I = RT.begin(), E = RT.end(); I != E; ++I)
      I->second.clear();

    PipeBreakerFUs.clear();
    PipeFUs.clear();
  }

  InstSetTy &getRTFor(unsigned Step, FuncUnitId FU) {
    return RT[std::make_pair(FU, Step)];
  }

  void revertFUUsage(MachineInstr *MI, unsigned step, unsigned Latency,
                     FuncUnitId FU);
  void revertFUUsage(VSUnit *U, unsigned step);
  void takeFU(VSUnit *U, unsigned step);
  void takeFU(MachineInstr *MI, unsigned step, unsigned Latency, FuncUnitId FU);
  MachineInstr *getConflictedInst(MachineInstr *MI, unsigned step,
                                  unsigned Latency, FuncUnitId FU);
  MachineInstr *getConflictedInst(VSUnit *U, unsigned step);
  bool tryTakeResAtStep(VSUnit *U, unsigned step);
  void scheduleSU(VSUnit *U, unsigned step);
  void unscheduleSU(VSUnit *U);

  void verifyFUUsage();

  unsigned buildTimeFrameAndResetSchedule(bool resetSTF);

  unsigned computeRecMII();
  unsigned computeResMII();
  bool computeMII();

  void setMII(unsigned II) { MII = II; }
  unsigned getMII() const { return MII; }
  void increaseMII() { ++MII; }
  void decreaseMII() { --MII; }
  void lengthenCriticalPath() { ++CriticalPathEnd; }
  void shortenCriticalPath() { --CriticalPathEnd; }
  unsigned getCriticalPathLength() {
    return CriticalPathEnd - State.EntrySlot;
  }
  void setCriticalPathLength(unsigned L) {
    CriticalPathEnd = State.EntrySlot + L;
  }

  void viewGraph();
};

template <> struct GraphTraits<SchedulingBase*> 
    : public GraphTraits<VSchedGraph*> {
  typedef VSchedGraph::sched_iterator nodes_iterator;
  static nodes_iterator nodes_begin(SchedulingBase *G) {
    return G->getState().sched_begin();
  }
  static nodes_iterator nodes_end(SchedulingBase *G) {
    return G->getState().sched_end();
  }
};

class IterativeModuloScheduling : public SchedulingBase {
  std::map<const VSUnit*, std::set<unsigned> > ExcludeSlots;

  void excludeStep(VSUnit *A, unsigned step);
  bool isStepExcluded(VSUnit *A, unsigned step);
  bool isAllSUnitScheduled();
  VSUnit *findBlockingSUnit(VSUnit *U, unsigned step);
public:
  IterativeModuloScheduling(VSchedGraph &S)
    : SchedulingBase(S) {}

  bool scheduleState();
};

struct ASAPScheduler : public SchedulingBase {
  ASAPScheduler(VSchedGraph &S) : SchedulingBase(S) {}

  bool scheduleState();
};

// A pseudo scheduler which generate linear order for SDC scheduler.
class BasicLinearOrderGenerator {
protected:
  typedef std::vector<VSUnit*> SUVecTy;
  typedef std::map<FuncUnitId, SUVecTy> ConflictListTy;
  typedef std::map<MachineBasicBlock*, SmallSet<FuncUnitId, 2> >
          FirstSlotConflictMapTy;
  SchedulingBase &S;
  FirstSlotConflictMapTy FirstSlotConflicts;
  void buildSuccConflictMap(const VSUnit *Terminator);
  bool isFUConflictedAtFirstSlot(MachineBasicBlock *MBB, FuncUnitId Id) const {
    FirstSlotConflictMapTy::const_iterator at = FirstSlotConflicts.find(MBB);
    return at == FirstSlotConflicts.end() ? false : at->second.count(Id);
  }

  void addLinOrdEdge(ConflictListTy &ConflictList, SUVecTy &PipeBreakers);
  // Add the linear ordering edges to the SUs in the vector and return the first
  // SU.
  VSUnit *addLinOrdEdge(SUVecTy &SUs);
  VSUnit *addLinOrdEdgeForPipeOp(FuncUnitId Id, SUVecTy &SUs);

  explicit BasicLinearOrderGenerator(SchedulingBase &S) : S(S) {}

  virtual void addLinOrdEdge();
public:

  static void addLinOrdEdge(SchedulingBase &S) {
    BasicLinearOrderGenerator(S).addLinOrdEdge();
  }
};

class ILPScheduler : public SchedulingBase {
  // The index of first step variable of the schedule unit.
  // For a given schedule unit with index i, who's time frame is N, then there
  // will be N step variables:
  // sv_i_0, sv_i_1, sv_i_2, ... , sv_i_(N-1) 
  // sv_i_j set to 1 means the schedule unit is scheduled to asap + j step
  std::map<const VSUnit*, unsigned> SUnitToSV;
  // Total step variables count.
  unsigned NumStepVars;
  // Total Step Variable;
  unsigned StepIdx;
  // Total variables in the model.
  unsigned TotalVariables;

  unsigned TotalRows;

  // Build the first step variable index mapping of schedule units based on
  // their ASAP and ALAP and return the total step variable count.
  //
  unsigned buildSVIdx();

  unsigned getFstSVIdxOf(const VSUnit *U) const {
    assert(U && "Unexpected NULL pointer!");
    std::map<const VSUnit*, unsigned>::const_iterator at = SUnitToSV.find(U);
    assert(at != SUnitToSV.end() && "SVIdx for VSUnit not exist!");
    return at->second;
  }

  typedef std::pair<const VSUnit*, unsigned> SUToIdx;
  typedef SmallVector<SUToIdx, 64> ActiveSUVec;
  typedef std::map<unsigned, ActiveSUVec> ActiveSUMap;
  ActiveSUMap ActiveSUs;

  // Only 1 step variable allowed set to 1.
  void buildOneActiveStepConstraints(lprec *lp);

  // The schedule should satisfy the dependences.
  void buildPrecedenceConstraints(lprec *lp);

  unsigned getIdxOf(VFUs::FUTypes FU) {
    return NumStepVars + FU - VFUs::FirstNonTrivialFUType;
  }

  // Avoid the resources conflict for the function units. 
  void buildFUConstraints(lprec *lp);

  // Set the variables' name in the model.
  void setUpVariables(lprec *lp);

  void buildObject(lprec *lp);

  // Build the schedule form the result of ILP.
  void buildSchedule(lprec *lp);
public:
  ILPScheduler(VSchedGraph &S);

  bool scheduleState();
};

class SDCScheduler : public SchedulingBase {
  public:
    SDCScheduler(VSchedGraph &S);
    bool scheduleState();
  private:
    lprec *lp;
    // The number of step variables.
    unsigned NumVars;
    // The number of VSUnits to schedule.
    unsigned NumInst;
    // Total rows in LP.
    unsigned TotalRows;
    // The table of the index of the VSUnits and the column number in LP.
    typedef std::map<const VSUnit*, unsigned> SUIdx2LPColMap;
    typedef SUIdx2LPColMap::iterator SUIdxIt;
    SUIdx2LPColMap SUIdx;

    // Set the variables' name in the model.
    void createStepVariables(lprec *lp);

    // The schedule should satisfy the dependences.
    void addDependencyConstraints(lprec *lp);

    // Build the schedule object function.
    void buildASAPObject();
    void buildOptimizingSlackDistributionObject();

    // Build the schedule form the result of ILP.
    void buildSchedule(lprec *lp);
};


} // End namespace.
#endif
