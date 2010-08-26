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

namespace esyn {;

class ForceDirectedInfo : public BasicBlockPass {
  HWAtomInfo *HI;
  FSMState *State;

  // Time Frame { {asap step, alap step }, isMIIContraint }
  typedef std::pair<unsigned, unsigned> TimeFrame;
  // Mapping hardware atoms to time frames.
  typedef std::map<const HWAtom*, TimeFrame> TimeFrameMapType;

  TimeFrameMapType AtomToTF;

  typedef std::map<unsigned, double> DGStepMapType;
  typedef std::map<HWFUnit*, std::map<unsigned, double> > DGType;
  DGType DGraph;

  unsigned computeStepKey(unsigned step) const;

  std::map<const HWAOpFU*, double> AvgDG;

  // MII in modulo schedule.
  unsigned MII, CriticalPathEnd;
public:

  /// @name TimeFrame
  //{
  unsigned getASAPStep(const HWAtom *A) const {
    return const_cast<ForceDirectedInfo*>(this)->AtomToTF[A].first;
  }

  void buildASAPStep(const HWAtom *Root, unsigned step);
  void buildALAPStep(const HWAtom *Root, unsigned step);

  unsigned getALAPStep(const HWAtom *A) const {
    return const_cast<ForceDirectedInfo*>(this)->AtomToTF[A].second;
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

  unsigned findBestStep(HWAOpFU *A);
  //}

  unsigned buildFDInfo();

  void setMII(unsigned II) { MII = II; }
  unsigned getMII() const { return MII; }
  void lengthenMII() { ++MII; }
  void lengthenCriticalPath() { ++CriticalPathEnd; }
  unsigned getCriticalPathEnd() { return CriticalPathEnd; }

  void reset();
  /// @name Common pass interface
  //{
  static char ID;
  ForceDirectedInfo() : BasicBlockPass(&ID), HI(0), MII(0),
    CriticalPathEnd(0) {}
  bool runOnBasicBlock(BasicBlock &BB);
  void releaseMemory();
  void getAnalysisUsage(AnalysisUsage &AU) const;
  virtual void print(raw_ostream &O, const Module *M) const {}
  //}
};
} // End namespace.
#endif
