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

namespace llvm {
class SchedulingBase {
  // MII in modulo schedule.
  unsigned MII, CriticalPathEnd;
  double ExtraResReq;
  // Time Frame {asap step, alap step }
public:
  typedef std::pair<unsigned, unsigned> TimeFrame;
private:
  // Time frames for each schedule unit.
  SmallVector<TimeFrame, 256> SUnitToTF;
  // Schedule time frames for each schedule unit.
  // The scheduling is accomplish by sink the width of time frames to 1.
  SmallVector<TimeFrame, 256> SUnitToSTF;
  // Resource requirement distribution of a specific resource
  // at each step.
  // step -> distribution
  typedef std::map<unsigned, double> DGStepMapType;
  // DG for every kind of function unit.
  typedef std::map<FuncUnitId, DGStepMapType> DGType;
  DGType DGraph;

  unsigned computeStepKey(unsigned step) const;

  SmallVector<double, 256> AvgDG;

  void resetSTF();

protected:
  VSchedGraph &State;

  SchedulingBase(VSchedGraph &S)
    : MII(0), CriticalPathEnd(0), ExtraResReq(0.0),
    SUnitToTF(S.getNumSUnits()), SUnitToSTF(S.getNumSUnits()),
    AvgDG(S.getNumSUnits()), State(S) {}

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
  void sinkSTF(const VSUnit *A, unsigned ASAP, unsigned ALAP);
  void updateSTF();


  void buildASAPStep(const VSUnit *ClampedSUnit = 0, unsigned ClampedASAP = 0);
  void buildALAPStep(const VSUnit *ClampedSUnit = 0, unsigned ClampedALAP = 0);
  void buildTimeFrame(const VSUnit *ClampedSUnit = 0,
                      unsigned ClampedASAP = 0,
                      unsigned ClampedALAP = 0);

  unsigned getASAPStep(const VSUnit *A) const {
    return SUnitToTF[A->getIdx()].first;
  }
  unsigned getALAPStep(const VSUnit *A) const {
    return SUnitToTF[A->getIdx()].second;
  }

  unsigned getSTFASAP(const VSUnit *A) const {
    return SUnitToSTF[A->getIdx()].first;
  }
  unsigned getSTFALAP(const VSUnit *A) const {
    return SUnitToSTF[A->getIdx()].second;
  }

  unsigned getTimeFrame(const VSUnit *A) const {
    return getALAPStep(A) - getASAPStep(A) + 1;
  }

  unsigned getScheduleTimeFrame(const VSUnit *A) const {
    return getSTFALAP(A) - getSTFASAP(A) + 1;
  }

  bool isSTFScheduled(const VSUnit *A) const {
    return getScheduleTimeFrame(A) < VSUnit::MaxSlot;
  }

  void printTimeFrame(raw_ostream &OS) const;
  void dumpTimeFrame() const;
  //}

  /// @name Distribution Graphs
  //{
  void buildDGraph();
  double getDGraphAt(unsigned step, FuncUnitId FUClass) const;
  void accDGraphAt(unsigned step, FuncUnitId FUClass, double d);
  void printDG(raw_ostream &OS) const;
  void dumpDG() const;
  /// Check the distribution graphs to see if we could schedule the nodes
  /// without breaking the resource constrain.
  bool isResourceConstraintPreserved();
  double getExtraResReq() const { return ExtraResReq; }
  //}

  /// @name Force computation
  //{
  void buildAvgDG();
  double getAvgDG(const VSUnit *A) {  return AvgDG[A->getIdx()]; }
  double getRangeDG(FuncUnitId FUClass, unsigned start, unsigned end/*included*/);

  double computeRangeForce(const VSUnit *A,
                           unsigned start, unsigned end/*include*/);
  double computeSelfForce(const VSUnit *A,
                          unsigned start, unsigned end/*include*/);

  /// If there are recurrences in the graph,
  /// Define "pred tree" and "succ tree" may intersect.
  /// All we can do is compute "other force", and the time frame of atoms
  /// not in "pred tree" or "succ tree" will not change, and not contribute
  /// to the force.
  double computeOtherForce(const VSUnit *A);

  double computeForce(const VSUnit *A, unsigned ASAP, unsigned ALAP);
  //}

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

class FDListScheduler : public SchedulingBase {
protected:
  /// @name PriorityQueue
  //{
  struct fds_sort {
    SchedulingBase &Info;
    fds_sort(SchedulingBase &s) : Info(s) {}
    bool operator() (const VSUnit *LHS, const VSUnit *RHS) const;
  };

  typedef PriorityQueue<VSUnit*, std::vector<VSUnit*>, fds_sort> SUnitQueueType;

  // Fill the priorityQueue, ignore FirstNode.
  template<class It>
  void fillQueue(SUnitQueueType &Queue, It begin, It end, VSUnit *FirstNode = 0);

  unsigned findBestStep(VSUnit *A);

  bool scheduleSUnit(VSUnit *A);
  bool scheduleQueue(SUnitQueueType &Queue);
  //}

public:
  FDListScheduler(VSchedGraph &S)
    : SchedulingBase(S) {}

  bool scheduleState();

};

class IteractiveModuloScheduling : public FDListScheduler {
  // Step -> resource require number.
  typedef std::map<unsigned, unsigned> UsageMapType;
  // Resource -> resource usage at each step.
  typedef std::map<FuncUnitId, UsageMapType> MRTType;
  MRTType MRT;
  SmallVector<std::set<unsigned>, 256> ExcludeSlots;

  bool isResAvailable(FuncUnitId FU, unsigned step);
  void excludeStep(VSUnit *A, unsigned step);
  bool isStepExcluded(VSUnit *A, unsigned step);
  bool isAllSUnitScheduled();
  VSUnit *findBlockingSUnit(FuncUnitId FU, unsigned step); 
public:
  IteractiveModuloScheduling(VSchedGraph &S)
    : FDListScheduler(S), ExcludeSlots(S.getNumSUnits()){}

  bool scheduleState();
};

class FDScheduler : public SchedulingBase {
public:
  FDScheduler(VSchedGraph &S)
    : SchedulingBase(S) {}

  bool scheduleState();
  bool findBestSink();
  double trySinkSUnit(VSUnit *A, TimeFrame &NewTimeFrame);
};

class IPLScheduler : public SchedulingBase {
public:
  IPLScheduler(VSchedGraph &S)
    : SchedulingBase(S) {}

  bool scheduleState();
};

} // End namespace.
#endif
