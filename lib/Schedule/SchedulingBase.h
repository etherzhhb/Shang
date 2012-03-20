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
  typedef std::pair<unsigned, unsigned> TimeFrame;
private:
  // Time frames for each schedule unit.
  typedef std::map<const VSUnit*, TimeFrame> TFMapTy;
  TFMapTy SUnitToTF;

  // Step -> resource require number.
  typedef std::map<unsigned, unsigned> UsageMapType;
  // Resource -> resource usage at each step.
  typedef std::map<FuncUnitId, UsageMapType> RTType;
  RTType RT;

protected:
  /// @name PriorityQueue
  //{

  VSchedGraph &State;
  unsigned computeStepKey(unsigned step) const;
  SchedulingBase(VSchedGraph &S)
    : MII(0), CriticalPathEnd(0), ExtraResReq(0.0), State(S) {}

public:
  virtual ~SchedulingBase() {}

  VSchedGraph &getState() const { return State; }

  virtual bool scheduleState() = 0;
  // Return true when resource constraints preserved after citical path
  // scheduled
  bool scheduleCriticalPath(bool refreshFDepHD);
  void schedulePassiveSUnits();
  bool allNodesSchedued() const;

  /// @name TimeFrame
  //{
  void buildASAPStep();
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

  /// Check the distribution graphs to see if we could schedule the nodes
  /// without breaking the resource constrain.
  void resetRT() {
    // FIXME: Do not clear the RT but set the function unit count in the
    // table to 0.
    RT.clear();
  }

  bool tryTakeResAtStep(VSUnit *U, unsigned step);
  void scheduleSU(VSUnit *U, unsigned step);
  void unscheduleSU(VSUnit *U);

  bool isResourceConstraintPreserved();
  double getExtraResReq() const { return ExtraResReq; }

  unsigned buildFDepHD(bool resetSTF);

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
    return CriticalPathEnd - State.getStartSlot();
  }
  void setCriticalPathLength(unsigned L) {
    CriticalPathEnd = State.getStartSlot() + L;
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
    // Total step variables count.
    int NumVars;
    // Total instructions count.
    int NumInst;
    // Total rows in LP.
    unsigned TotalRows;
    // The table of the index of the VSUnits and the column number in LP.
    typedef std::map<const VSUnit*, unsigned> SUIdx2LPColMap;
    typedef SUIdx2LPColMap::iterator SUIdxIt;
    SUIdx2LPColMap SUIdx;
    // The table of the slots and the VSUnits.
    typedef std::vector<const VSUnit*> BoundSUVec;
    typedef std::map<unsigned, BoundSUVec> Step2SUMap;

    // Get the MaxLatency of the VSUnit.
    unsigned getMaxLatency(const VSUnit* U){
      typedef std::list<VSUnit*>::const_iterator const_use_iterator;
      unsigned FinLatency = U->getLatency();
      for(const_use_iterator EI = U->use_begin(),EE = U->use_end();
        EI != EE; ++EI){
          const VSUnit* Use = *EI;
          const VDEdge* Edge = Use->getEdgeFrom(U);
          FinLatency = std::max<unsigned>(FinLatency, ((*Edge).getLatency()));
      }
      return FinLatency;
    }

    unsigned getComInNum(const VSUnit* Src, const VSUnit* Dst);

    //Sort the vector in ASAP decreasing order.
    void sortVector(BoundSUVec &Vec, Step2SUMap &Map, unsigned Idx);

    unsigned countValDeps(const VSUnit* U);
    unsigned countValUses(const VSUnit* U);

    bool isOverlap(const VSUnit* Dst, const VSUnit* Src){
      return getALAPStep(Src) >= getASAPStep(Dst);
    }
    // Set the variables' name in the model.
    void createStepVariables(lprec *lp);

    // Build the intrinsic constraints for LP variables.
    void addStepConstraints(lprec *lp);

    // The schedule should satisfy the dependences.
    void addDependencyConstraints(lprec *lp);

    template<typename FuncTy>
    static unsigned getMaxOrMinSlot(Step2SUMap &Map, unsigned InitVal, FuncTy F);

    static unsigned getMaxSlot(Step2SUMap &Map) {
      return getMaxOrMinSlot(Map, 0, std::max<unsigned>);
    }

    static unsigned getMinSlot(Step2SUMap &Map) {
      return getMaxOrMinSlot(Map, VSUnit::MaxSlot, std::min<unsigned>);
    }

    // Avoid the resources conflict for the function units.
    void PreBind();
    // PreBind the Memory.
    void allcoMem(unsigned FUType, Step2SUMap& Map);
    // PreBind the FUs.
    void PreBindFU(unsigned FUType, Step2SUMap &Map);

    // Build the schedule object function.
    void buildASAPObject();
    void buildOptimizingSlackDistributionObject();

    // Build the schedule form the result of ILP.
    void buildSchedule(lprec *lp);

};


} // End namespace.
#endif
