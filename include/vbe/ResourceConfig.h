/*
* Copyright: 2008 by Nadav Rotem. all rights reserved.
* IMPORTANT: This software is supplied to you by Nadav Rotem in consideration
* of your agreement to the following terms, and your use, installation, 
* modification or redistribution of this software constitutes acceptance
* of these terms.  If you do not agree with these terms, please do not use, 
* install, modify or redistribute this software. You may not redistribute, 
* install copy or modify this software without written permission from 
* Nadav Rotem. 
*/

#ifndef VBE_RESOURCE_CONFIG_H
#define VBE_RESOURCE_CONFIG_H

#include "llvm/Pass.h"
#include "llvm/Function.h"
#include "llvm/Instructions.h"
#include "llvm/Constants.h"
#include "llvm/DerivedTypes.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Module.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/FoldingSet.h"
#include "llvm/Support/Allocator.h"
#include "llvm/Support/CFG.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/System/DataTypes.h"
#include "llvm/Support/raw_ostream.h"

#include <set>
#include <map>
#include <sstream>

namespace rapidxml {
template<class> class xml_node;
}

using namespace llvm;

namespace esyn {
class HWFUnit;
class ResourceConfig;
/// @brief Represent hardware resource
class HWResType {
public:
  enum Types {
    MemoryBus = 1,
    SHL,
    ASR,
    LSR,
    AddSub,
    Mult,
    Trivial,

    FirstResourceType = MemoryBus,
    LastResourceType = Trivial
  };
private:
  // The HWResource baseclass this node corresponds to
  Types ResourceType;
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
  HWResType(Types type, unsigned latency, unsigned startInt, unsigned totalRes)
    : ResourceType(type), Latency(latency), StartInt(startInt),
    TotalRes(totalRes) {}
public:
  Types getType() const { return ResourceType; }
  
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
    : HWResType(HWResType::MemoryBus, latency, startInt, totalRes),
    AddrWidth(addrWidth), DataWidth(dataWidth) {}
public:
  unsigned getAddrWidth() const { return AddrWidth; }
  unsigned getDataWidth() const { return DataWidth; }

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWMemBus *A) { return true; }
  static inline bool classof(const HWResType *A) {
    return A->getType() == HWResType::MemoryBus;
  }

  static HWMemBus *createFromXml(rapidxml::xml_node<char> *Node);
  static std::string getTypeName() { return "MemoryBus"; }
  static Types getType() { return HWResType::MemoryBus; }
};

class HWBinOpResType : public HWResType {
  unsigned MaxBitWidth;

protected:
  HWBinOpResType(HWResType::Types T, unsigned latency, unsigned startInt,
    unsigned totalRes, unsigned maxBitWidth)
    : HWResType(T, latency, startInt, totalRes), MaxBitWidth(maxBitWidth) {}

public:
  unsigned getMaxBitWidth() const { return MaxBitWidth; }

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWBinOpResType *A) { return true; }
  static inline bool classof(const HWResType *A) {
    return A->getType() == HWResType::AddSub
      || A->getType() == HWResType::SHL
      || A->getType() == HWResType::ASR
      || A->getType() == HWResType::LSR
      || A->getType() == HWResType::Mult;
  }

  template<class BinOpResType>
  static BinOpResType *createFromXml(rapidxml::xml_node<char> *Node);
};

#define BINOPRESTYPECLASS(Name) \
class HW##Name : public HWBinOpResType { \
  unsigned MaxBitWidth; \
  HW##Name(unsigned latency, unsigned startInt, unsigned totalRes, \
  unsigned maxBitWidth) \
  : HWBinOpResType(HWResType::##Name, latency, startInt, totalRes, \
  maxBitWidth) \
{} \
  friend class HWBinOpResType; \
public: \
  static inline bool classof(const HW##Name *A) { return true; } \
  static inline bool classof(const HWResType *A) { \
  return A->getType() == HWResType::##Name; \
} \
  static std::string getTypeName() { return #Name; } \
  static Types getType() { return HWResType::##Name; } \
};

BINOPRESTYPECLASS(Mult);
BINOPRESTYPECLASS(AddSub);
BINOPRESTYPECLASS(SHL);
BINOPRESTYPECLASS(ASR);
BINOPRESTYPECLASS(LSR);

class HWFUnit : public FoldingSetNode {
  /// FastID - A reference to an Interned FoldingSetNodeID for this node.
  /// The ScalarEvolution's BumpPtrAllocator holds the data.
  FoldingSetNodeIDRef FastID;

  enum HWResType::Types T : 4;
  unsigned Latency        : 4;
  unsigned TotalFUs       : 24;

  // FIXME: char is enough for bit width.
  uint8_t *const InputBitWidth;
  size_t NumInputs;
  uint8_t *const OutputBitWidth;
  size_t NumOutputs;

  inline HWFUnit(const FoldingSetNodeIDRef ID, HWResType::Types type,
                 unsigned totalFUs, unsigned latency,
                 uint8_t *I, size_t NI, uint8_t *O, size_t NO)
    : FastID(ID), T(type), TotalFUs(totalFUs), Latency(latency),
    InputBitWidth(I), NumInputs(NI), OutputBitWidth(O), NumOutputs(NO) {
    assert(totalFUs && "Unavailable Function Unit?");
    assert((totalFUs == TotalFUs || T == HWResType::Trivial)
           && latency == Latency && "Data overflow!");
  }
  friend class HWResType;
  friend class ResourceConfig;
public:
  /// Profile - FoldingSet support.
  void Profile(FoldingSetNodeID& ID) { ID = FastID; }

  inline enum HWResType::Types getResType() const { return T; }
  
  inline unsigned getTotalFUs() const { return TotalFUs; }
  inline unsigned getLatency() const { return Latency; }

  inline uint8_t getInputBitwidth(unsigned idx) const {
    assert(idx < NumInputs && "Index out of range!");
    return InputBitWidth[idx];
  }

  inline unsigned getNumInputs() const { return NumInputs; }

  inline uint8_t getOutputBitwidth(unsigned idx) const {
    assert(idx < NumOutputs && "Index out of range!");
    return OutputBitWidth[idx];
  }

  inline unsigned getNumOutputs() const { return NumOutputs; }
};


/// Print a RegionNode.
inline raw_ostream &operator<<(raw_ostream &OS, const HWFUnit &U) {
  OS << U.getResType();
  return OS;
}

class ResourceConfig : public ImmutablePass {
  
  /// mapping allocated instences to atom
  HWResType *ResSet[(size_t)HWResType::LastResourceType -
                     (size_t)HWResType::FirstResourceType + 1];

  void ParseConfigFile(const std::string &Filename);

  HWResType *getResType(enum HWResType::Types T) const {
    unsigned idx = (unsigned)T - (unsigned)HWResType::FirstResourceType;
    assert(ResSet[idx] && "Bad resource!");
    return ResSet[idx];
  }

  // Allocator
  BumpPtrAllocator HWFUAllocator;
  FoldingSet<HWFUnit> UniqiueHWFUs;

public:
  static char ID;
  ResourceConfig() : ImmutablePass(&ID) {
    for (size_t i = 0, e = (size_t)HWResType::LastResourceType; i != e; ++i)
      ResSet[i] = 0;
  }

  ~ResourceConfig();

  virtual void initializePass();

  void print(raw_ostream &OS) const;

  template<class ResType>
  ResType *getResType() const {
    return cast<ResType>(getResType(ResType::getType()));
  }

  HWFUnit *allocaBinOpFU(HWResType::Types T, unsigned BitWitdh,
                         unsigned UnitID = 0);
  HWFUnit *allocaMemBusFU(unsigned UnitID);
  HWFUnit *allocaTrivialFU(unsigned latency);

  typedef HWResType *const * iterator;
  typedef const HWResType *const * const_iterator;

  iterator begin() { return &ResSet[0]; }
  const_iterator begin() const { return &ResSet[0]; }

  iterator end() { 
    return begin() + (size_t)HWResType::LastResourceType -
      (size_t)HWResType::FirstResourceType;
  }
  const_iterator end() const { 
    return begin() + (size_t)HWResType::LastResourceType -
      (size_t)HWResType::FirstResourceType;
  }
}; //class
} // namespace

#endif // h guard

