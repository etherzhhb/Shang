//===-- ChainBreakingAnalysis.cpp - Chain Breaking Caculation ---*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the ChainBreakingAnalysis class.
// The ChainBreakingAnalysis class calculates the best step to break the
// multi-cycles chain during/after scheduling, to expose more FU sharing
// opportunity.
//
//===----------------------------------------------------------------------===//

#include "ChainBreakingAnalysis.h"
#include "VSUnit.h"

using namespace llvm;

ScheduleLiveInterval::ScheduleLiveInterval(unsigned StartSlot, unsigned EndSlot)
  : StartSlot(StartSlot), EndSlot(EndSlot) {
  assert(StartSlot < EndSlot && "Bad interval!");
}

unsigned ChainBreakingAnalysis::getBreakingSlot(const VSUnit *U) const {
  assert(U->isControl() && "Bad schedule unit type!");
  return U->getFinSlot();
}
