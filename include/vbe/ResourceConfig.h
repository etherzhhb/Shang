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
class HWAPreBind;

/// @brief Represent hardware resource
class HWResource {
public:
  enum ResTypes {
    MemoryBus = 1,
    SHL,
    ASR,
    LSR,
    AddSub,
    Mul,
    Trivial,

    FirstResourceType = MemoryBus,
    LastResourceType = Trivial
  };
private:
  // The HWResource baseclass this node corresponds to
  ResTypes ResourceType;
  // The name of resource
  const std::string Name;
  // How many cycles to finish?
  const unsigned Latency;
  // Start interval
  const unsigned StartInt;
  // How many resources available?
  const unsigned TotalRes;

  // Use a map mapping instance to count?
  typedef std::vector<unsigned> UsingCountVec;
  UsingCountVec UsingCount;

  HWResource(const HWResource &);            // DO NOT IMPLEMENT
  void operator=(const HWResource &);  // DO NOT IMPLEMENT
protected:
  explicit HWResource(enum ResTypes type,
    std::string name, unsigned latency, unsigned startInt, unsigned totalRes)
    : ResourceType(type), Name(name), Latency(latency), StartInt(startInt),
      TotalRes(totalRes), UsingCount(totalRes) {
    //
    clear();
  }
public:
  ResTypes getResourceType() const { return ResourceType; }
  
  unsigned getLatency() const { return Latency; }
  unsigned getTotalRes() const { return TotalRes; }
  unsigned getStartInt() const { return StartInt; }
  const std::string &getName() const { return Name; }

  virtual void print(raw_ostream &OS) const;

  size_t getTotalUsed() const {
    size_t ret = 0;
    for (UsingCountVec::const_iterator I = UsingCount.begin(),
        E = UsingCount.end(); I != E; ++I)
      ret += *I;
    
    return ret;
  }

  size_t getUsingCount(unsigned instance) const {
      return UsingCount[instance];
  }

  void assignToInstance(unsigned instance) {
    ++UsingCount[instance];
  }

  unsigned getLeastBusyInstance() const;

  void clear();
}; 


struct HWFUnitID {
  union {
    struct {
      HWResource::ResTypes T : 4;
      unsigned UnitID        : 28;
    } S;
    unsigned Data;
  } U;

  /*implicit*/ inline HWFUnitID(enum HWResource::ResTypes type,
                                unsigned UID = 0) {
    U.S.T = type;
    U.S.UnitID = UID;
    assert(U.S.T == type && U.S.UnitID == UID && "Data overflow!");
  }
  /*implicit*/ inline HWFUnitID(unsigned Data) { U.Data = Data; }

  inline HWFUnitID(const HWFUnitID &O) { U.Data = O.U.Data; }
  inline const HWFUnitID &operator=(const HWFUnitID &O) { U.Data = O.U.Data; }

  inline enum HWResource::ResTypes getResType() const { return U.S.T; }
  inline unsigned getUnitID() const { return U.S.UnitID; }

  inline unsigned getRawData() const { return U.Data; }
};


/// Print a RegionNode.
inline raw_ostream &operator<<(raw_ostream &OS, const HWFUnitID UID) {
  OS << UID.getRawData();
  return OS;
}

/// @name HWFUnitID Comparison Operators
/// @{

inline bool operator==(HWFUnitID LHS, HWFUnitID RHS) {
  return LHS.U.Data == RHS.U.Data;
}

inline bool operator!=(HWFUnitID LHS, HWFUnitID RHS) {
  return !(LHS == RHS);
}

inline bool operator<(HWFUnitID LHS, HWFUnitID RHS) {
  return LHS.U.Data < RHS.U.Data;
}

inline bool operator<=(HWFUnitID LHS, HWFUnitID RHS) {
  return LHS.U.Data <= RHS.U.Data;
}

inline bool operator>(HWFUnitID LHS, HWFUnitID RHS) {
  return LHS.U.Data > RHS.U.Data;
}

inline bool operator>=(HWFUnitID LHS, HWFUnitID RHS) {
  return LHS.U.Data >= RHS.U.Data;
}

/// @}

class HWMemBus : public HWResource {
  unsigned AddrWidth;
  unsigned DataWidth;
  // Read latency and write latency

  HWMemBus(std::string name, unsigned latency,
    unsigned startInt, unsigned totalRes,
    unsigned addrWidth, unsigned dataWidth)
    : HWResource(HWResource::MemoryBus, name, latency, startInt, totalRes),
    AddrWidth(addrWidth), DataWidth(dataWidth) {}
public:
  unsigned getAddrWidth() const { return AddrWidth; }
  unsigned getDataWidth() const { return DataWidth; }

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWMemBus *A) { return true; }
  static inline bool classof(const HWResource *A) {
    return A->getResourceType() == HWResource::MemoryBus;
  }

  static HWMemBus *createFromXml(rapidxml::xml_node<char> *Node);
};

class HWAddSub : public HWResource {
  unsigned MaxBitWidth;
  // Read latency and write latency

  HWAddSub(std::string name, unsigned latency,
    unsigned startInt, unsigned totalRes, unsigned maxBitWidth)
    : HWResource(HWResource::AddSub, name, latency, startInt, totalRes),
    MaxBitWidth(maxBitWidth) {}
public:
  unsigned getMaxBitWidth() const { return MaxBitWidth; }

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWAddSub *A) { return true; }
  static inline bool classof(const HWResource *A) {
    return A->getResourceType() == HWResource::AddSub;
  }

  static HWAddSub *createFromXml(rapidxml::xml_node<char> *Node);
};

class ResourceConfig : public ImmutablePass {
  
  /// mapping allocated instences to atom
  HWResource *ResSet[(size_t)HWResource::LastResourceType];

  void ParseConfigFile(const std::string &Filename);

public:
  static char ID;
  ResourceConfig() : ImmutablePass(&ID) {
    for (size_t i = 0, e = (size_t)HWResource::LastResourceType; i != e; ++i)
      ResSet[i] = 0;
  }

  ~ResourceConfig();

  virtual void initializePass();

  void print(raw_ostream &OS) const;

  HWResource *getResource(enum HWResource::ResTypes T) const {
    unsigned idx = (unsigned)T - 1;
    return ResSet[idx];
  }


  HWResource *operator[] (enum HWResource::ResTypes T) const {
    return getResource(T);
  }

  typedef HWResource *const * iterator;
  typedef const HWResource *const * const_iterator;

  iterator begin() { return &ResSet[0]; }
  const_iterator begin() const { return &ResSet[0]; }

  iterator end() { 
    return begin() + (size_t)HWResource::LastResourceType -
      (size_t)HWResource::FirstResourceType;
  }
  const_iterator end() const { 
    return begin() + (size_t)HWResource::LastResourceType -
      (size_t)HWResource::FirstResourceType;
  }
}; //class

} // namespace

#endif // h guard

