//===- ModuloScheduleInfo.h - ModuleSchedule information analyze -*- C++ -*-===//
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
// This file define the ModuleSchedule information computation pass describe in
// Josep, L. (1996). Swing Modulo Scheduling: A Lifetime-Sensitive Approach.
//
//===----------------------------------------------------------------------===//
#ifndef VBE_MODULE_SCHEDULE_INFO
#define VBE_MODULE_SCHEDULE_INFO

#include "HWAtomInfo.h"

namespace llvm {
  class LoopInfo;
}
using namespace llvm;

namespace esyn {
class ModuloScheduleInfo : public FunctionPass {
    HWAtomInfo *HI;
    LoopInfo *LI;
    ResourceConfig *RC;

    std::map<unsigned, std::vector<HWAtom*> > RecList;
    
public:
  void clear();

  /// Could us preform modulo schedule on the given state?
  bool isModuloSchedulable(FSMState &State) const;

  unsigned computeResMII(FSMState &State) const;

  ///
  unsigned computeRecMII(FSMState &State);

  /// @name Common pass interface
  //{
  static char ID;
  ModuloScheduleInfo() : FunctionPass(&ID), HI(0), LI(0), RC(0) {}
  ~ModuloScheduleInfo();
  bool runOnFunction(Function &F);
  void releaseMemory() { clear(); }
  void getAnalysisUsage(AnalysisUsage &AU) const;
  virtual void print(raw_ostream &O, const Module *M) const {}
  //}
};
}
#endif
