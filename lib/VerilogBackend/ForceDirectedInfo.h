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
class ForceDirectedInfo : public FunctionPass {
  HWAtomInfo *HI;
  ResourceConfig *RC;

  // Time Frame {asap step, alap step }
  typedef std::pair<unsigned, unsigned> TimeFrame;
  // Mapping hardware atoms to time frames.
  typedef std::map<const HWAtom*, TimeFrame> TimeFrameMapType;

  TimeFrameMapType AtomToTF;

  // The Key of DG, { step, resource type }
  typedef std::map<unsigned, double> DGType;

  DGType DGraph;

  std::map<const HWAPostBind*, double> AvgDG;

  // MII in modulo schedule.
  unsigned Modulo;
public:

  /// @name TimeFrame
  //{
  void buildASAPStep(const HWAtom *Root, unsigned step);
  unsigned getASAPStep(const HWAOpInst *A) const {
    return const_cast<ForceDirectedInfo*>(this)->AtomToTF[A].first;
  }


  void buildALAPStep(const HWAtom *Root, unsigned step);
  unsigned getALAPStep(const HWAOpInst *A) const { 
    return const_cast<ForceDirectedInfo*>(this)->AtomToTF[A].second;
  }

  unsigned getTimeFrame(const HWAOpInst *A) const {
    return getALAPStep(A) - getASAPStep(A) + 1;
  }

  // If the TimeFrame Constrains by II.
  bool isModuloConstrains(const HWAOpInst *A) const {
    return getTimeFrame(A) == Modulo;
  }

  void printTimeFrame(FSMState *State, raw_ostream &OS) const;
  void dumpTimeFrame(FSMState *State) const;
  //}

  /// @name Distribution Graphs
  //{
  void buildDGraph(FSMState *State);
  double getDGraphAt(unsigned step, enum HWResource::ResTypes ResType) const;
  void accDGraphAt(unsigned step, enum HWResource::ResTypes ResType, double d);
  void printDG(FSMState *State, raw_ostream &OS) const ;
  void dumpDG(FSMState *State) const ;
  //}

  /// @name Force computation
  //{
  void buildAvgDG(FSMState *State);
  double getAvgDG(const HWAPostBind *A) {  return AvgDG[A]; }
  double getRangeDG(const HWAPostBind *A, unsigned start, unsigned end/*included*/);

  double computeSelfForceAt(const HWAOpInst *OpInst, unsigned step);
  /// This function will invalid the asap step of all node in
  /// successor tree
  double computeSuccForceAt(const HWAOpInst *OpInst, unsigned step);
  /// This function will invalid the alap step of all node in
  /// predecessor tree
  double computePredForceAt(const HWAOpInst *OpInst, unsigned step);

  unsigned findBestStep(HWAOpInst *A);
  //}

  unsigned buildFDInfo(FSMState *State, unsigned StartStep,
                       unsigned EndStep = 0);

  void enableModuleFD(unsigned II) { Modulo = II; }

  void clear();
  /// @name Common pass interface
  //{
  static char ID;
  ForceDirectedInfo() : FunctionPass(&ID), HI(0), RC(0) {}
  bool runOnFunction(Function &F);
  void releaseMemory();
  void getAnalysisUsage(AnalysisUsage &AU) const;
  virtual void print(raw_ostream &O, const Module *M) const {}
  //}
};
} // End namespace.
#endif
