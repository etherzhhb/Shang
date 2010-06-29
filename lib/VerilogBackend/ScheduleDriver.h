//===-------- ScheduleDriver.h - The Scheduler driver pass  ------*- C++ -*-===//
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
// This file define the ScheduleDriver Pass, which run difference schedulers on
// a llvm function to schedule the Hardware atoms.
//
//===----------------------------------------------------------------------===//
//
#ifndef VBE_SCHEDULE_DRIVER_H
#define VBE_SCHEDULE_DRIVER_H

#include "vbe/ResourceConfig.h"
#include "vbe/HWAtom.h"

#include "llvm/ADT/ScopedHashTable.h"

#include "HWAtomInfo.h"

using namespace llvm;

namespace esyn {

class Scheduler : public BasicBlockPass {
  // {instance, next available cycle}
  typedef std::map<HWResource::ResIdType, unsigned> ResCycMapType;

  ResCycMapType ResCycMap;

protected:
  HWAtomInfo *HI;
  ResourceConfig *RC;

  void clear();

  // Get the ready cycle of the given resource.
  unsigned getReadyCycle(HWResource::ResIdType ResId);

  // Remember the ready cycle of the given resource.
  void rememberReadyCycle(HWResource::ResIdType ResId, unsigned ReadyCycle);

  // Get Any ready atom.
  HWAtom *getReadyAtoms(SmallVectorImpl<HWAtom*> &ToSchedAtoms,
                        unsigned Cycle) const;

public:
  explicit Scheduler(const void *pid) : BasicBlockPass(pid), HI(0) {}
  explicit Scheduler(intptr_t pid) : BasicBlockPass(pid), HI(0) {}

  virtual ~Scheduler();

  // Request common analysis usage for scheduler
  void getAnalysisUsage(AnalysisUsage &AU) const;
  // Do common Initialization for scheduler
  bool runOnBasicBlock(BasicBlock &BB);
  void releaseMemory();

  // Interface for schedulers
  virtual void scheduleBasicBlock(ExecStage &State) = 0;
  virtual void releaseContext() {}
};

/// @brief Hardware atome schedule pass.
class ScheduleDriver :public FunctionPass {
  // The loop info 
  LoopInfo *LI;

  // The hardware atoms
  //HWAtomInfo *HI;

  void clear();
public:
  /// @name FunctionPass interface
  //{
  static char ID;
  explicit ScheduleDriver() : FunctionPass(&ID), LI(0)/*, HI(0)*/{}
  bool runOnFunction(Function &F);
  void releaseMemory();
  void getAnalysisUsage(AnalysisUsage &AU) const;
  virtual void print(raw_ostream &O, const Module *M) const;
  //}
};
} // end namespace

#endif
