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

class SchedAtom {
  // Corresponding atom
  HWAtom *Atom;

public:
  /*implicit*/ SchedAtom(HWAtom &atom) : Atom(&atom) {}

  /// @name Smart Pointer
  //{
  HWAtom *operator*() const { return Atom; }
  HWAtom *operator->() const { return Atom; }
  //}

  typedef HWAtom::const_dep_iterator const_dep_iterator;
  typedef HWAtom::dep_iterator dep_iterator;

  HWAtom *getAtom() const { return Atom; }

  /// operator== - Convenience function for get().operator==
  bool operator==(const SchedAtom &V) const {
    return Atom == V.getAtom();
  }

  /// operator!= - Convenience function for get().operator!=
  bool operator!=(const SchedAtom &V) const {
    return !operator ==(V);
  }

  /// operator< - Convenience function for get().operator<
  bool operator<(const SchedAtom &V) const {
    return Atom < V.getAtom();
  }
  
  // TOOD: move these to schedule dag.
  bool isOperationFinish(unsigned CurSlot) const;

  bool isAllDepsOpFin(unsigned CurSlot) const;
};

class Scheduler : public BasicBlockPass {
  // {instance, next available cycle}
  typedef std::map<HWResource::ResIdType, unsigned> ResCycMapType;

  ResCycMapType ResCycMap;

protected:
  typedef std::vector<SchedAtom*> SchedAtomVec;
  SchedAtomVec ScheduleAtoms;

  HWAtomInfo *HI;
  ResourceConfig *RC;

  void clear();

  // Get the ready cycle of the given resource.
  unsigned getReadyCycle(HWResource::ResIdType ResId);

  // Remember the ready cycle of the given resource.
  void rememberReadyCycle(HWResource::ResIdType ResId, unsigned ReadyCycle);

  // Get Any ready atom.
  SchedAtom *getReadyAtoms(unsigned Cycle);

  void removeFromList(SchedAtom *Atom);
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

} // end namespace

namespace llvm {
  /// simplify_type specializations - Allow casting operators to work directly on
  /// SDValues as if they were SDNode*'s.
  template<> struct simplify_type<esyn::SchedAtom> {
    typedef esyn::HWAtom* SimpleType;
    static SimpleType getSimplifiedValue(const esyn::SchedAtom &Val) {
      return Val.getAtom();
    }
  };

  template<> struct simplify_type<const esyn::SchedAtom> {
    typedef esyn::HWAtom* SimpleType;
    static SimpleType getSimplifiedValue(const esyn::SchedAtom &Val) {
      return Val.getAtom();
    }
  };
}

#endif
