//===-- ChainBreakingAnalysis.h - Chain Breaking Caculation -----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines the ChainBreakingAnalysis class.
// The ChainBreakingAnalysis class calculates the best step to break the
// multi-cycles chain during/after scheduling, to expose more FU sharing
// opportunity.
//
//===----------------------------------------------------------------------===//

#ifndef CHAIN_BREAKING_ANALYSIS_H
#define CHAIN_BREAKING_ANALYSIS_H

namespace llvm {
class VSUnit;

/// @brief Represents the live interval in the scheduling graph.
class ScheduleLiveInterval {
  // The bounds of the interval. [StartSlot, EndSlot), please note that if we
  // ignore the interval of PHINodes, we should always have StartSlot > EndSlot,
  // which means the user must be scheduled the definition.

  /// @brief The start slot of the interval, inclusive.
  unsigned StartSlot;

  /// @brief The end slot of the interval, exclusive.
  unsigned EndSlot;

public:
  explicit ScheduleLiveInterval(unsigned StartSlot = 0, unsigned EndSlot = 1)
                              : StartSlot(StartSlot), EndSlot(EndSlot) {
    assert(StartSlot < EndSlot && "Bad interval!");
  }

  bool isOverlapped(const ScheduleLiveInterval &Other) const {
    return Other.EndSlot < StartSlot || EndSlot < Other.StartSlot;
  }
};

class ChainBreakingAnalysis {
public:

  /// @brief Get the slot to break the chain start from U.
  unsigned getBreakingSlot(const VSUnit *U) const;
};
}

#endif
