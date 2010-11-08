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

#include "VInstrInfo.h"

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
/// @brief Represent hardware resource
class HWResType {
  // The HWResource baseclass this node corresponds to
  VInstrInfo::FUTypes ResourceType;
  // How many cycles to finish?
  const unsigned Latency;
  // Start interval
  const unsigned StartInt;
  // How many resources available?
  const unsigned TotalRes;

  //// Use a map mapping instance to count?
  //typedef std::vector<unsigned> UsingCountVec;
  //UsingCountVec UsingCount;

  HWResType(const HWResType &);            // DO NOT IMPLEMENT
  void operator=(const HWResType &);  // DO NOT IMPLEMENT
protected:
  HWResType(VInstrInfo::FUTypes type, unsigned latency, unsigned startInt, unsigned totalRes)
    : ResourceType(type), Latency(latency), StartInt(startInt),
    TotalRes(totalRes) {}
public:
  VInstrInfo::FUTypes getType() const { return ResourceType; }
  
  unsigned getLatency() const { return Latency; }
  unsigned getTotalRes() const { return TotalRes; }
  unsigned getStartInt() const { return StartInt; }

  virtual void print(raw_ostream &OS) const;
}; 

class HWMemBus : public HWResType {
  unsigned AddrWidth;
  unsigned DataWidth;

  HWMemBus(unsigned latency, unsigned startInt, unsigned totalRes,
    unsigned addrWidth, unsigned dataWidth)
    : HWResType(VInstrInfo::MemoryBus, latency, startInt, totalRes),
    AddrWidth(addrWidth), DataWidth(dataWidth) {}

  friend class VTMConfig;
public:
  unsigned getAddrWidth() const { return AddrWidth; }
  unsigned getDataWidth() const { return DataWidth; }

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWMemBus *A) { return true; }
  static inline bool classof(const HWResType *A) {
    return A->getType() == VInstrInfo::MemoryBus;
  }

  static std::string getTypeName() { return "MemoryBus"; }
  static VInstrInfo::FUTypes getType() { return VInstrInfo::MemoryBus; }
};

class HWBinOpResType : public HWResType {
  unsigned MaxBitWidth;

protected:
  HWBinOpResType(VInstrInfo::FUTypes T, unsigned latency, unsigned startInt,
    unsigned totalRes, unsigned maxBitWidth)
    : HWResType(T, latency, startInt, totalRes), MaxBitWidth(maxBitWidth) {}

public:
  unsigned getMaxBitWidth() const { return MaxBitWidth; }

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWBinOpResType *A) { return true; }
  static inline bool classof(const HWResType *A) {
    return A->getType() == VInstrInfo::AddSub
      || A->getType() == VInstrInfo::SHL
      || A->getType() == VInstrInfo::ASR
      || A->getType() == VInstrInfo::LSR
      || A->getType() == VInstrInfo::Mult;
  }
};

#define BINOPRESTYPECLASS(Name) \
class HW##Name : public HWBinOpResType { \
  unsigned MaxBitWidth; \
  HW##Name(unsigned latency, unsigned startInt, unsigned totalRes, \
  unsigned maxBitWidth) \
  : HWBinOpResType(VInstrInfo::##Name, latency, startInt, totalRes, \
  maxBitWidth) \
{} \
  friend class VTMConfig; \
public: \
  static inline bool classof(const HW##Name *A) { return true; } \
  static inline bool classof(const HWResType *A) { \
  return A->getType() == VInstrInfo::##Name; \
} \
  static std::string getTypeName() { return #Name; } \
  static VInstrInfo::FUTypes getType() { return VInstrInfo::##Name; } \
}

BINOPRESTYPECLASS(Mult);
BINOPRESTYPECLASS(AddSub);
BINOPRESTYPECLASS(SHL);
BINOPRESTYPECLASS(ASR);
BINOPRESTYPECLASS(LSR);

class VTMConfig : public TargetSubtarget {
  /// Selected instruction itineraries (one entry per itinerary class.)
  InstrItineraryData InstrItins;
  // Just some dummy subtarget features.
  bool vtmattr;

  /// mapping allocated instences to atom
  HWResType *ResSet[(size_t)VInstrInfo::LastResourceType -
                    (size_t)VInstrInfo::FirstResourceType + 1];

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
  ResType *getResType() const {
    return cast<ResType>(getResType(ResType::getType()));
  }

  HWResType *getResType(enum VInstrInfo::FUTypes T) const {
    unsigned idx = (unsigned)T - (unsigned)VInstrInfo::FirstResourceType;
    assert(ResSet[idx] && "Bad resource!");
    return ResSet[idx];
  }

  typedef HWResType *const * iterator;
  typedef const HWResType *const * const_iterator;

  iterator begin() { return &ResSet[0]; }
  const_iterator begin() const { return &ResSet[0]; }

  iterator end() { 
    return begin() + (size_t)VInstrInfo::LastResourceType -
      (size_t)VInstrInfo::FirstResourceType;
  }

  const_iterator end() const { 
    return begin() + (size_t)VInstrInfo::LastResourceType -
      (size_t)VInstrInfo::FirstResourceType;
  }
};

} // end namespace llvm

#endif
