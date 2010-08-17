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

using namespace llvm;

namespace esyn {
class ModuloScheduleInfo;

class ForceDirectedInfo : public BasicBlockPass {
  HWAtomInfo *HI;
  ResourceConfig *RC;
  FSMState *State;

  // Time Frame { {asap step, alap step }, isMIIContraint }
  typedef std::pair<unsigned, unsigned> TimeFrame;
  // Mapping hardware atoms to time frames.
  typedef std::map<const HWAtom*, TimeFrame> TimeFrameMapType;

  TimeFrameMapType AtomToTF;

  void buildASAPStep(const FSMState *EntryRoot, unsigned step);
  void buildALAPStep(const HWAOpInst *ExitRoot, unsigned step);

  // The Key of DG, { step, resource type }
  typedef std::map<unsigned, double> DGType;
  DGType DGraph;

  // Resource usage.
  typedef std::map<unsigned, unsigned> ResUsageTabType;
  ResUsageTabType ResUsage;

  unsigned computeStepKey(unsigned step, HWFUnitID FUID) const;

  static void decompseStepKey(unsigned key, unsigned &step, unsigned &FUID);

  std::map<const HWAOpInst*, double> AvgDG;

  // MII in modulo schedule.
  unsigned MII, CriticalPathEnd;
public:

  /// @name TimeFrame
  //{
  void buildASAPStep() {
    buildASAPStep(State, State->getSlot());
  }
  unsigned getASAPStep(const HWAtom *A) const {
    assert((isa<HWAOpInst>(A) || isa<FSMState>(A) || isa<HWADelay>(A))
          && "Bad atom type!");
    return const_cast<ForceDirectedInfo*>(this)->AtomToTF[A].first;
  }

  void initALAPStep();

  void buildALAPStep() {
    initALAPStep();
    const HWAOpInst *Root = State->getExitRoot();
    return buildALAPStep(Root, getALAPStep(Root));
  }

  unsigned getALAPStep(const HWAtom *A) const {
    assert((isa<HWAOpInst>(A) || isa<FSMState>(A) || isa<HWADelay>(A))
          && "Bad atom type!");
    return const_cast<ForceDirectedInfo*>(this)->AtomToTF[A].second;
  }

  unsigned getTimeFrame(const HWAtom *A) const {
    return getALAPStep(A) - getASAPStep(A) + 1;
  }

  // If the TimeFrame Constrains by II.
  bool constrainByMII(const HWAOpInst *A) const {
    return getTimeFrame(A) == MII - A->getLatency();
  }

  void printTimeFrame(raw_ostream &OS) const;
  void dumpTimeFrame() const;
  //}

  /// @name Distribution Graphs
  //{
  void buildDGraph();
  double getDGraphAt(unsigned step, HWFUnitID FUID) const;
  void accDGraphAt(unsigned step, HWFUnitID FUID, double d);
  void printDG(raw_ostream &OS) const ;
  void dumpDG() const ;
  //}

  /// @name Resource usage table
  //{
  /// @brief If the usage of given kind of FU not exceed the maximum available number.
  bool isFUAvailalbe(unsigned step, HWFUnit FU) const;
  void presevesFUForAtom(HWAtom *A);
  //}

  /// @name Force computation
  //{
  void buildAvgDG();
  double getAvgDG(const HWAOpInst *A) {  return AvgDG[A]; }
  double getRangeDG(HWFUnitID FUID, unsigned start, unsigned end/*included*/);

  double computeSelfForceAt(const HWAOpInst *OpInst, unsigned step);
  /// This function will invalid the asap step of all node in
  /// successor tree
  double computeSuccForceAt(const HWAOpInst *OpInst, unsigned step);
  /// This function will invalid the alap step of all node in
  /// predecessor tree
  double computePredForceAt(const HWAOpInst *OpInst, unsigned step);

  unsigned findBestStep(HWAOpInst *A);
  //}

  unsigned buildFDInfo();

  void setMII(unsigned II) { MII = II; }
  unsigned getMII() const { return MII; }
  void lengthenMII() { ++MII; }
  void lengthenCriticalPath() { ++CriticalPathEnd; }

  void reset();
  /// @name Common pass interface
  //{
  static char ID;
  ForceDirectedInfo() : BasicBlockPass(&ID), HI(0), RC(0), MII(0),
    CriticalPathEnd(0) {}
  bool runOnBasicBlock(BasicBlock &BB);
  void releaseMemory();
  void getAnalysisUsage(AnalysisUsage &AU) const;
  virtual void print(raw_ostream &O, const Module *M) const {}
  //}
};
} // End namespace.
#endif
