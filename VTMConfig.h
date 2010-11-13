//===-------- VSubtarget.h - Define Subtarget for the VTM --------*- C++ -*-====//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares the VTM specific subclass of TargetSubtarget.
//
//===----------------------------------------------------------------------===//

#ifndef VSUBTARGET_H
#define VSUBTARGET_H

#include "VFunctionUnit.h"

#include "llvm/Target/TargetSubtarget.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/ADT/FoldingSet.h"
#include "llvm/Support/Allocator.h"

#include <string>
#include <set>
#include <map>

namespace llvm {
class Module;
class VTMConfig;

class VTMConfig : public TargetSubtarget {
  /// Selected instruction itineraries (one entry per itinerary class.)
  InstrItineraryData InstrItins;
  // Just some dummy subtarget features.
  bool vtmattr;

  /// mapping allocated instences to atom
  VFUDesc *ResSet[VFUs::NumFUs];

public:
  VTMConfig(const std::string &TT, const std::string &FS);
  ~VTMConfig();

  std::string ParseSubtargetFeatures(const std::string &FS,
                                     const std::string &CPU);

  /// getInstrItins - Return the instruction itineraies based on subtarget
  /// selection.
  const InstrItineraryData &getInstrItineraryData() const { return InstrItins; }

  void initializeTarget();

  void setupMemBus(unsigned latency, unsigned startInt, unsigned totalRes,
                   unsigned addrWidth, unsigned dataWidth);

  template<class BinOpResType>
  void setupBinOpRes(unsigned latency, unsigned startInt, unsigned totalRes,
    unsigned maxBitWidth);

  void print(raw_ostream &OS, const Module *) const;

  template<class ResType>
  ResType *getFUDesc() const {
    return cast<ResType>(getFUDesc(ResType::getType()));
  }

  VFUDesc *getFUDesc(enum VFUs::FUTypes T) const {
    unsigned idx = (unsigned)T - (unsigned)VFUs::FirstFUType;
    assert(ResSet[idx] && "Bad resource!");
    return ResSet[idx];
  }

  typedef VFUDesc *const * iterator;
  typedef const VFUDesc *const * const_iterator;

  iterator begin() { return &ResSet[0]; }
  const_iterator begin() const { return &ResSet[0]; }

  iterator end() { 
    return begin() + (size_t)VFUs::LastFUType -
      (size_t)VFUs::FirstFUType;
  }

  const_iterator end() const { 
    return begin() + (size_t)VFUs::LastFUType -
      (size_t)VFUs::FirstFUType;
  }
};

} // end namespace llvm

#endif
