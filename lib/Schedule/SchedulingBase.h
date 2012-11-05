//===- ForceDirectedSchedulingBase.h - ForceDirected information analyze --*- C++ -*-===//
//
//                      The Shang HLS frameowrk                               //
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
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
protected:
  // MII in modulo schedule.
  const unsigned EntrySlot;
  unsigned MII, CriticalPathEnd;
  // Time Frame {asap step, alap step }
public:
  typedef std::pair<unsigned, unsigned> TimeFrame;
private:
  typedef SmallVector<const MachineInstr*, 4> InstSetTy;
  static InstSetTy::const_iterator findConflictedInst(const InstSetTy &Set,
                                                      const MachineInstr *MI);
  static const MachineInstr *getConflictedInst(const InstSetTy &S,
                                               const MachineInstr *MI) {
    InstSetTy::const_iterator at = findConflictedInst(S, MI);
    return at != S.end() ? *at : 0;
  }

  bool hasConflictedInst(const InstSetTy &Set, const MachineInstr *MI) {
    // Nothing to conflict if the set is empty.
    if (Set.empty()) return false;

    // Even the two MIs not in the same trace, the different trace may active at
    // the same time in different iterations.
    if (G.enablePipeLine()) return true;

    return findConflictedInst(Set, MI) != Set.end();
  }

  // Remember the Instructions those are scheduled to a particular step.
  // (FU, Step) -> Instructions.
  typedef std::map<std::pair<FuncUnitId, unsigned>, InstSetTy> RTType;
  RTType RT;

protected:
  // Time frames for each schedule unit.
  typedef std::map<const VSUnit*, TimeFrame> TFMapTy;
  TFMapTy SUnitToTF;

  /// @name PriorityQueue
  //{
  VSchedGraph &G;

  unsigned computeStepKey(unsigned step) const;
  SchedulingBase(VSchedGraph &S)
    : EntrySlot(S.EntrySlot), MII(0), CriticalPathEnd(0),
      G(S) {}

public:
  virtual ~SchedulingBase() {}

  VSchedGraph *operator*() const { return &G; }
  VSchedGraph *operator->() const { return &G; }

  virtual bool scheduleState() { return false; };

  // Return true when resource constraints preserved after citical path
  // scheduled
  typedef VSchedGraph::iterator iterator;
  bool scheduleCriticalPath(iterator I, iterator E);
  typedef VSchedGraph::const_iterator const_iterator;
  bool allNodesSchedued(const_iterator I, const_iterator E) const;

  /// @name TimeFrame
  //{

  unsigned getASAPStep(const VSUnit *A) const {
    TFMapTy::const_iterator at = SUnitToTF.find(A);
    assert(at != SUnitToTF.end() && "TimeFrame for SU not exist!");
    return at->second.first;
  }
  unsigned getALAPStep(const VSUnit *A) const {
    TFMapTy::const_iterator at = SUnitToTF.find(A);
    assert(at != SUnitToTF.end() && "TimeFrame for SU not exist!");
    return at->second.second;
  }

  unsigned getTimeFrame(const VSUnit *A) const {
    return getALAPStep(A) - getASAPStep(A) + 1;
  }
  //}

  void resetRT() {
    for (RTType::iterator I = RT.begin(), E = RT.end(); I != E; ++I)
      I->second.clear();
  }

  InstSetTy &getRTFor(unsigned Step, FuncUnitId FU) {
    return RT[std::make_pair(FU, Step)];
  }

  void revertFUUsage(const MachineInstr *MI, unsigned step, unsigned Latency,
                     FuncUnitId FU);
  void revertFUUsage(const VSUnit *U, unsigned step);
  void takeFU(const VSUnit *U, unsigned step);
  void takeFU(const MachineInstr *MI, unsigned step, unsigned Latency,
              FuncUnitId FU);
  const MachineInstr *getConflictedInst(const MachineInstr *MI, unsigned step,
                                        unsigned Latency, FuncUnitId FU);
  const MachineInstr *getConflictedInst(const VSUnit *U, unsigned step);
  bool tryTakeResAtStep(const VSUnit *U, unsigned step);
  void scheduleSU(VSUnit *U, unsigned step);
  void unscheduleSU(VSUnit *U);
  void verifyFUUsage(iterator I, iterator E);

  unsigned computeRecMII();
  unsigned computeResMII();

  void setMII(unsigned II) { MII = II; }
  unsigned getMII() const { return MII; }
  void increaseMII() { ++MII; }
  void decreaseMII() { --MII; }
  void lengthenCriticalPath() { ++CriticalPathEnd; }
  void shortenCriticalPath() { --CriticalPathEnd; }
  unsigned getCriticalPathLength() {
    assert(CriticalPathEnd > G.EntrySlot && "CriticalPathLength not available!");
    return CriticalPathEnd - G.EntrySlot;
  }
  void setCriticalPathLength(unsigned L) {
    CriticalPathEnd = G.EntrySlot + L;
  }

};

template<bool IsCtrlPath>
class Scheduler : public SchedulingBase {
protected:
  friend class SchedulingBase;
  explicit Scheduler(VSchedGraph &S) : SchedulingBase(S) {}

  // Helper function for SU traversing, SU dependencies traversing and SU users
  // traversing.
  typedef VSUnit::const_dep_iterator const_dep_it;
  static const_dep_it dep_begin(const VSUnit *U) {
    return U->dep_begin<IsCtrlPath>();
  }

  static const_dep_it dep_end(const VSUnit *U) {
    return U->dep_end<IsCtrlPath>();
  }

  typedef VSUnit::const_use_iterator const_use_it;
  static const_use_it use_begin(const VSUnit *U) {
    return U->use_begin<IsCtrlPath>();
  }

  static const_use_it use_end(const VSUnit *U) {
    return U->use_end<IsCtrlPath>();
  }

  unsigned calculateASAP(const VSUnit *A);
  // Apply the Bellman-Ford like algorithm at most |V|-1 times, return true if
  // negative cycle found.
  bool buildASAPStep();
  unsigned calculateALAP(const VSUnit *A);
  void buildALAPStep();

  using SchedulingBase::scheduleCriticalPath;
public:

  iterator begin() const {
    return G.begin<IsCtrlPath>();
  }

  iterator end() const {
    return G.end<IsCtrlPath>();
  }

  unsigned buildTimeFrameAndResetSchedule(bool reset);
  void resetTimeFrame();
  void buildTimeFrame();
  void printSUTimeFrame(raw_ostream &OS, const VSUnit *A) const;

  void printTimeFrame(raw_ostream &OS) const {
    OS << "Time frame:\n";
    for (iterator I = begin(), E = end(); I != E; ++I)
      printSUTimeFrame(OS, *I);
  }

  void dumpTimeFrame() const;

  bool scheduleCriticalPath() {
    buildTimeFrameAndResetSchedule(true);
    return scheduleCriticalPath(G.begin<IsCtrlPath>(), G.end<IsCtrlPath>());
  }

  // Find the minimal II which can eliminate the negative cycles. Where we
  // detect negative cycles by applying Bellman-Ford like algorithm at most
  // |V|-1 times to see if the ASAP steps convergence.
  unsigned computeRecMII(unsigned MinRecMII);

  void viewGraph() {
    ViewGraph(this, G.getEntryBB()->getName());
  }
};

template <bool IsCtrlPath> struct GraphTraits<Scheduler<IsCtrlPath>*> 
    : public GraphTraits<VSchedGraphWrapper<IsCtrlPath> > {
  typedef VSchedGraph::iterator nodes_iterator;
  static nodes_iterator nodes_begin(Scheduler<IsCtrlPath> *G) {
    return G->begin();
  }
  static nodes_iterator nodes_end(Scheduler<IsCtrlPath> *G) {
    return G->end();
  }
};

EXTERN_TEMPLATE_INSTANTIATION(class Scheduler<false>);
EXTERN_TEMPLATE_INSTANTIATION(class Scheduler<true>);

class IterativeModuloScheduling : public Scheduler<true> {
  std::map<const VSUnit*, std::set<unsigned> > ExcludeSlots;

  void excludeStep(VSUnit *A, unsigned step);
  bool isStepExcluded(VSUnit *A, unsigned step);
  VSUnit *findBlockingSUnit(VSUnit *U, unsigned step);
public:
  IterativeModuloScheduling(VSchedGraph &S) : Scheduler<true>(S) {}

  enum ScheduleResult {
    Success,
    MIITooSmall,
    Unknown
  };

  ScheduleResult scheduleLoop();
};

struct ASAPScheduler : public Scheduler<true> {
  ASAPScheduler(VSchedGraph &S) : Scheduler<true>(S) {}

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

  void addLinOrdEdge(ConflictListTy &ConflictList);
  // Add the linear ordering edges to the SUs in the vector and return the first
  // SU.
  VSUnit *addLinOrdEdge(SUVecTy &SUs);

  explicit BasicLinearOrderGenerator(SchedulingBase &S) : S(S) {}

  virtual void addLinOrdEdge();
public:

  static void addLinOrdEdge(SchedulingBase &S) {
    BasicLinearOrderGenerator(S).addLinOrdEdge();
  }
};


class SDCSchedulingBase {
  struct SoftConstraint {
    double Penalty;
    const VSUnit *Src, *Dst;
    unsigned SlackIdx, Slack;
  };
public:

  typedef VSchedGraph::iterator iterator;
  // Set the variables' name in the model.
  unsigned createLPAndVariables(iterator I, iterator E);
  unsigned addSoftConstraint(const VSUnit *Src, const VSUnit *Dst,
                             unsigned Slack, double Penalty);

  // Build the schedule object function.
  void buildASAPObject(iterator I, iterator E, double weight);
  void buildOptSlackObject(iterator I, iterator E, double weight);
  void addSoftConstraintsPenalties(double weight);

  void addObjectCoeff(const VSUnit *U, double Value) {
    // Ignore the constants.
    if (U->isScheduled()) return;
    
    ObjFn[getSUIdx(U)] += Value;
  }

  unsigned getSUIdx(const VSUnit* U) const {
    SUIdxIt at = SUIdx.find(U);
    assert(at != SUIdx.end() && "Idx not existed!");
    return at->second;
  }

protected:
  const unsigned ScheduleLB;
  lprec *lp;

  // Helper class to build the object function for lp.
  struct LPObjFn : public std::map<unsigned, double> {
    LPObjFn &operator*=(double val) {
      for (iterator I = begin(), E = end(); I != E; ++I)
        I->second *= val;

      return *this;
    }

    LPObjFn &operator+=(const LPObjFn &Other) {
      for (const_iterator I = Other.begin(), E = Other.end(); I != E; ++I)
        (*this)[I->first] += I->second;

      return *this;
    }

    void setLPObj(lprec *lp) const;
  };

  LPObjFn ObjFn;

  // The table of the index of the VSUnits and the column number in LP.
  typedef std::map<const VSUnit*, unsigned> SUI2IdxMapTy;
  typedef SUI2IdxMapTy::const_iterator SUIdxIt;
  SUI2IdxMapTy SUIdx;

  SDCSchedulingBase(unsigned ScheduleLB) : ScheduleLB(ScheduleLB), lp(0) {}

  typedef std::vector<SoftConstraint> SoftCstrVecTy;
  SoftCstrVecTy SoftCstrs;

  // Create step variables, which represent the c-step that the VSUnits are
  // scheduled to.
  unsigned createStepVariable(const VSUnit *U, unsigned Col);

  void addSoftConstraints(lprec *lp);

  bool solveLP(lprec *lp);

  // Build the schedule form the result of ILP.
  void buildSchedule(lprec *lp, unsigned TotalRows, iterator I, iterator E);
};

template<bool IsCtrlPath>
class SDCScheduler : public SDCSchedulingBase, public Scheduler<IsCtrlPath> {
  // The schedule should satisfy the dependences.
  void addDependencyConstraints(lprec *lp);

  using Scheduler<IsCtrlPath>::G;
  typedef typename Scheduler<IsCtrlPath>::const_dep_it const_dep_it;
  using Scheduler<IsCtrlPath>::dep_begin;
  using Scheduler<IsCtrlPath>::dep_end;
  typedef typename Scheduler<IsCtrlPath>::const_use_it const_use_it;
  using Scheduler<IsCtrlPath>::use_begin;
  using Scheduler<IsCtrlPath>::use_end;

  using Scheduler<IsCtrlPath>::begin;
  using Scheduler<IsCtrlPath>::end;
  using SDCSchedulingBase::createLPAndVariables;
  using SDCSchedulingBase::buildASAPObject;
  typedef SDCSchedulingBase::iterator iterator;

public:
  explicit SDCScheduler(VSchedGraph &S)
    : SDCSchedulingBase(S.EntrySlot), Scheduler<IsCtrlPath>(S) {}

  unsigned createLPAndVariables() {
    return createLPAndVariables(begin(), end());
  }

  void buildASAPObject(double weight) {
    buildASAPObject(begin(), end(), weight);
  }

  bool schedule();
};

EXTERN_TEMPLATE_INSTANTIATION(class SDCScheduler<false>);
EXTERN_TEMPLATE_INSTANTIATION(class SDCScheduler<true>);
} // End namespace.
#endif
