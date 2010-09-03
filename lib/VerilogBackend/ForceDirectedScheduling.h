//===- ForceDirectedInfo.h - ForceDirected information analyze --*- C++ -*-===//
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

#include "HWAtomInfo.h"
#include "ModuloScheduleInfo.h"

#include "llvm/ADT/PriorityQueue.h"

using namespace llvm;

namespace esyn {;

class ForceDirectedInfo {
  HWAtomInfo *HI;
  FSMState *State;

  // Time Frame { {asap step, alap step }, isMIIContraint }
  typedef std::pair<unsigned, unsigned> TimeFrame;
  // Mapping hardware atoms to time frames.
  typedef std::map<const HWAtom*, TimeFrame> TimeFrameMapType;

  TimeFrameMapType AtomToTF;
  TimeFrameMapType AtomToSTF;

  typedef std::map<unsigned, double> DGStepMapType;
  typedef std::map<HWFUnit*, std::map<unsigned, double> > DGType;
  DGType DGraph;

  unsigned computeStepKey(unsigned step) const;

  std::map<const HWAOpFU*, double> AvgDG;

  void buildASAPStep(const HWAtom *Root, unsigned step);
  void buildALAPStep(const HWAtom *Root, unsigned step);

  // MII in modulo schedule.
  unsigned MII, CriticalPathEnd;

  void resetSTF();
  void sinkSTF(const HWAtom *A, unsigned ASAP, unsigned ALAP);
  void updateSTF();
public:

  /// @name TimeFrame
  //{
  void buildTimeFrame();

  unsigned getASAPStep(const HWAtom *A) const {
    return const_cast<ForceDirectedInfo*>(this)->AtomToTF[A].first;
  }
  unsigned getALAPStep(const HWAtom *A) const {
    return const_cast<ForceDirectedInfo*>(this)->AtomToTF[A].second;
  }

  unsigned getSTFASAP(const HWAtom *A) const {
    return const_cast<ForceDirectedInfo*>(this)->AtomToSTF[A].first;
  }
  unsigned getSTFALAP(const HWAtom *A) const {
    return const_cast<ForceDirectedInfo*>(this)->AtomToSTF[A].second;
  }

  unsigned getTimeFrame(const HWAtom *A) const {
    return getALAPStep(A) - getASAPStep(A) + 1;
  }

  // If the TimeFrame Constraints by II.
  bool constraintByMII(const HWAOpFU *A) const {
    return getTimeFrame(A) == MII - A->getLatency();
  }

  void printTimeFrame(raw_ostream &OS) const;
  void dumpTimeFrame() const;
  //}

  /// @name Distribution Graphs
  //{
  void buildDGraph();
  double getDGraphAt(unsigned step, HWFUnit *FU) const;
  void accDGraphAt(unsigned step, HWFUnit  *FUID, double d);
  void printDG(raw_ostream &OS) const;
  void dumpDG() const;
  /// Check the distribution graphs to see if we could schedule the nodes
  /// without breaking the resource constrain.
  bool isResourceConstraintPreserved();
  //}

  /// @name Force computation
  //{
  void buildAvgDG();
  double getAvgDG(const HWAOpFU *A) {  return AvgDG[A]; }
  double getRangeDG(HWFUnit *FU, unsigned start, unsigned end/*included*/);

  double computeRangeForce(const HWAtom *A,
                           unsigned start, unsigned end/*include*/);
  double computeSelfForceAt(const HWAtom *A, unsigned step);
  /// This function will invalid the asap step of all node in
  /// successor tree
  double computeSuccForceAt(const HWAtom *A, unsigned step);
  /// This function will invalid the alap step of all node in
  /// predecessor tree
  double computePredForceAt(const HWAtom *A, unsigned step);
  //}

  unsigned buildFDInfo(bool resetSTF);

  void setMII(unsigned II) { MII = II; }
  unsigned getMII() const { return MII; }
  void lengthenMII() { ++MII; }
  void lengthenCriticalPath() { ++CriticalPathEnd; }
  unsigned getCriticalPathEnd() { return CriticalPathEnd; }

  ForceDirectedInfo(HWAtomInfo *HAInfo, FSMState *S) : HI(HAInfo), State(S),
    MII(0), CriticalPathEnd(0) {}
};

class ForceDirectedListSchedulingBase {
protected:
  HWAtomInfo *HI;
  ForceDirectedInfo FDInfo;
  FSMState *CurState;

  unsigned II;

  /// @name PriorityQueue
  //{
  struct fds_sort {
    ForceDirectedInfo &Info;
    fds_sort(ForceDirectedInfo &s) : Info(s) {}
    bool operator() (const HWAOpFU *LHS, const HWAOpFU *RHS) const;
  };

  typedef PriorityQueue<HWAOpFU*, std::vector<HWAOpFU*>, fds_sort> AtomQueueType;

  // Fill the priorityQueue, ignore FirstNode.
  template<class It>
  void fillQueue(AtomQueueType &Queue, It begin, It end, HWAtom *FirstNode = 0);

  // Return true when resource constraints preserved after citical path
  // scheduled
  bool scheduleCriticalPath();

  unsigned findBestStep(HWAtom *A);

  bool scheduleAtom(HWAtom *A);
  void schedulePassiveAtoms();
  bool scheduleQueue(AtomQueueType &Queue);
  //}

public:
  ForceDirectedListSchedulingBase(HWAtomInfo *HAInfo, FSMState *S, unsigned MII)
    : HI(HAInfo), FDInfo(HAInfo, S), CurState(S), II(MII) {}

  virtual void scheduleState() = 0;
};

class ForceDirectedListScheduler : public ForceDirectedListSchedulingBase {

public:
  ForceDirectedListScheduler(HWAtomInfo *HAInfo, FSMState *S, unsigned MII)
    : ForceDirectedListSchedulingBase(HAInfo, S, MII) {}

  void scheduleState();
};

class ForceDirectedModuloScheduler : public ForceDirectedListSchedulingBase {
  typedef ModuloScheduleInfo::rec_iterator rec_iterator;
  typedef ModuloScheduleInfo::rec_vector rec_vector;
public:
  ForceDirectedModuloScheduler(HWAtomInfo *HAInfo, FSMState *S, unsigned MII)
    : ForceDirectedListSchedulingBase(HAInfo, S, MII) {}

  void scheduleState();
  bool scheduleAtII();
};

} // End namespace.
#endif
