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
class ModuloScheduleInfo {
  HWAtomInfo *HI;
  LoopInfo *LI;
  FSMState *State;

  typedef std::multimap<unsigned, const std::vector<HWAtom*> > RecMapType;
  RecMapType RecList;
public:
  void clear();

  typedef RecMapType::iterator rec_iterator;
  typedef RecMapType::const_iterator const_rec_iterator;
  rec_iterator rec_begin(unsigned II) { return RecList.lower_bound(II); }
  const_rec_iterator rec_begin(unsigned II) const { return RecList.lower_bound(II); }
  rec_iterator rec_end(unsigned II) { return RecList.upper_bound(II); }
  const_rec_iterator rec_end(unsigned II) const { return RecList.upper_bound(II); }

  typedef const std::vector<HWAtom*> rec_vector;

  void addRecurrence(unsigned II, rec_vector Rec);

  /// Could us preform modulo schedule on the given state?
  bool isModuloSchedulable() const;

  unsigned computeResMII() const;

  unsigned computeRecMII();

  /// @name Common pass interface
  //{
  ModuloScheduleInfo(HWAtomInfo *HAInfo, LoopInfo *LInfo, FSMState *S)
    : HI(HAInfo), LI(LInfo), State(S) {}
  ~ModuloScheduleInfo();
  //}
};
}
#endif
