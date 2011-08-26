//===- ForceDirectedSchedulingBase.h - ForceDirected information analyze --*- C++ -*-===//
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
// This file define the Force Direct information computation pass describe in
// Force-Directed Scheduling for the Behavioral Synthesis of ASIC's
//
//===----------------------------------------------------------------------===//
#ifndef VBE_FORCE_DIRECTED_INFO
#define VBE_FORCE_DIRECTED_INFO

#include "VSUnit.h"

#include "llvm/ADT/PriorityQueue.h"
#include <map>
using namespace llvm;

//Dirty Hack
struct _lprec;
typedef _lprec lprec;

namespace llvm {
class SchedulingBase {
  // MII in modulo schedule.
  unsigned MII, CriticalPathEnd;
  double ExtraResReq;
  // Time Frame {asap step, alap step }
public:
  typedef std::pair<OpSlot, OpSlot> TimeFrame;
private:
  // Time frames for each schedule unit.
  SmallVector<TimeFrame, 256> SUnitToTF;

  // Step -> resource require number.
  typedef std::map<unsigned, unsigned> UsageMapType;
  // Resource -> resource usage at each step.
  typedef std::map<FuncUnitId, UsageMapType> RTType;
  RTType RT;

protected:
  /// @name PriorityQueue
  //{
  struct fds_sort {
    SchedulingBase &Info;
    fds_sort(SchedulingBase &s) : Info(s) {}
    bool operator() (const VSUnit *LHS, const VSUnit *RHS) const;
  };

  VSchedGraph &State;
  unsigned computeStepKey(unsigned step) const;
  SchedulingBase(VSchedGraph &S)
    : MII(0), CriticalPathEnd(0), ExtraResReq(0.0),
    SUnitToTF(S.getNumSUnits()), State(S) {}

public:
  virtual ~SchedulingBase() {}

  VSchedGraph &getState() const { return State; }

  virtual bool scheduleState() = 0;
  // Return true when resource constraints preserved after citical path
  // scheduled
  bool scheduleCriticalPath(bool refreshFDepHD);
  void schedulePassiveSUnits();

  /// @name TimeFrame
  //{
  void buildASAPStep();
  void buildALAPStep();
  void buildTimeFrame();

  unsigned getASAPStep(const VSUnit *A) const {
    return SUnitToTF[A->getIdx()].first.getSlot();
  }
  unsigned getALAPStep(const VSUnit *A) const {
    return SUnitToTF[A->getIdx()].second.getSlot();
  }

  unsigned getASAPDetailStep(const VSUnit *A) const {
    return SUnitToTF[A->getIdx()].first.getDetailSlot();
  }
  unsigned getALAPDetailStep(const VSUnit *A) const {
    return SUnitToTF[A->getIdx()].second.getDetailSlot();
  }

  unsigned getTimeFrame(const VSUnit *A) const {
    return getALAPStep(A) - getASAPStep(A) + 1;
  }

  void printTimeFrame(raw_ostream &OS) const;
  void dumpTimeFrame() const;
  //}

  /// Check the distribution graphs to see if we could schedule the nodes
  /// without breaking the resource constrain.
  void resetRT() {
    // FIXME: Do not clear the RT but set the function unit count in the
    // table to 0.
    RT.clear();
  }

  bool tryTakeResAtStep(VSUnit *U, unsigned step);
  bool isResourceConstraintPreserved();
  double getExtraResReq() const { return ExtraResReq; }

  unsigned buildFDepHD(bool resetSTF);

  void setMII(unsigned II) { MII = II; }
  unsigned getMII() const { return MII; }
  void increaseMII() { ++MII; }
  void decreaseMII() { --MII; }
  void lengthenCriticalPath() { ++CriticalPathEnd; }
  void shortenCriticalPath() { --CriticalPathEnd; }
  unsigned getCriticalPathLength() {
    return CriticalPathEnd - State.getStartSlot();
  }
  void setCriticalPathLength(unsigned L) {
    CriticalPathEnd = State.getStartSlot() + L;
  }

  void viewGraph();
};

template <> struct GraphTraits<SchedulingBase*> 
    : public GraphTraits<VSchedGraph*> {
  typedef VSchedGraph::iterator nodes_iterator;
  static nodes_iterator nodes_begin(SchedulingBase *G) {
    return G->getState().begin();
  }
  static nodes_iterator nodes_end(SchedulingBase *G) {
    return G->getState().end();
  }
};

class IteractiveModuloScheduling : public SchedulingBase {
  SmallVector<std::set<unsigned>, 256> ExcludeSlots;

  void excludeStep(VSUnit *A, unsigned step);
  bool isStepExcluded(VSUnit *A, unsigned step);
  bool isAllSUnitScheduled();
  VSUnit *findBlockingSUnit(FuncUnitId FU, unsigned step); 
public:
  IteractiveModuloScheduling(VSchedGraph &S)
    : SchedulingBase(S), ExcludeSlots(S.getNumSUnits()){}

  bool scheduleState();
};

class ILPScheduler : public SchedulingBase {
  // The index of first step variable of the schedule unit.
  // For a given schedule unit with index i, who's time frame is N, then there
  // will be N step variables:
  // sv_i_0, sv_i_1, sv_i_2, ... , sv_i_(N-1) 
  // sv_i_j set to 1 means the schedule unit is scheduled to asap + j step
  SmallVector<unsigned, 256> SUnitToSV;
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
    return SUnitToSV[U->getIdx()];
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

} // End namespace.
#endif
