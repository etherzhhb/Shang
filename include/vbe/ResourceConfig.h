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
class HWAOpRes;

enum HWResourceTypes {
  MemoryBus = 1,
  Shifter = 2,
  Comparator = 3,
  LogicUnit = 4,
  ArithUnit = 5,

  FirstResourceType = MemoryBus,
  LastResourceType = ArithUnit
};

/// @brief Represent hardware resource
class HWResource {
  // The HWResource baseclass this node corresponds to
  const unsigned short ResourceType;
  // The name of resource
  const std::string Name;
  // How many cycles to finish?
  const unsigned Latency;
  // Start interval
  const unsigned StartInt;
  // How many resources available?
  const unsigned TotalRes;

  typedef std::set<HWAOpRes*> HWAtomSetType;
  HWAtomSetType UsingAtoms;

  typedef std::vector<unsigned> UsingCountVec;
  UsingCountVec UsingCount;

  HWResource(const HWResource &);            // DO NOT IMPLEMENT
  void operator=(const HWResource &);  // DO NOT IMPLEMENT
public:
  static const unsigned Infinite = UINT32_MAX;

  explicit HWResource(enum HWResourceTypes type,
    std::string name, unsigned latency, unsigned startInt, unsigned totalRes)
    : ResourceType(type), Name(name), Latency(latency), StartInt(startInt),
      TotalRes(totalRes), UsingCount(totalRes != UINT32_MAX ? totalRes : 0) {
    //
    clear();
  }

  unsigned getResourceType() const { return ResourceType; }
  
  unsigned getLatency() const { return Latency; }
  unsigned getTotalRes() const { return TotalRes; }
  bool isInfinite() const { return TotalRes == Infinite; }
  unsigned getStartInt() const { return StartInt; }
  const std::string &getName() const { return Name; }

  virtual void print(raw_ostream &OS) const;

  void addUsingAtom(HWAOpRes *Atom) {
    UsingAtoms.insert(Atom);
  } 

  size_t getUsingCount(unsigned idx = 0) const {
    assert(idx <= UsingCount.size() + 1 && "idx out of range!");
    if (idx == 0) // Return the total usage
      return UsingAtoms.size(); 
    else // 
      return UsingCount[idx - 1];
  }

  void assignToInstance(unsigned instance) {
    ++UsingCount[instance - 1];
  }

  unsigned getLeastBusyInstance() const;

  void clear();
}; 

class HWMemBus : public HWResource {
  unsigned AddrWidth;
  unsigned DataWidth;
  // Read latency and write latency

  explicit HWMemBus(std::string name, unsigned latency,
    unsigned startInt, unsigned totalRes,
    unsigned addrWidth, unsigned dataWidth)
    : HWResource(MemoryBus, name, latency, startInt, totalRes),
    AddrWidth(addrWidth), DataWidth(dataWidth) {}
public:
  unsigned getAddrWidth() const { return AddrWidth; }
  unsigned getDataWidth() const { return DataWidth; }

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const HWMemBus *A) { return true; }
  static inline bool classof(const HWResource *A) {
    return A->getResourceType() == MemoryBus;
  }

  static HWMemBus *createFromXml(rapidxml::xml_node<char> *Node);
};

class ResourceConfig : public ImmutablePass {
  
  typedef std::map<std::string, HWResource*> ResTabTy;

  ResTabTy ResTab;

  void ParseConfigFile(const std::string &Filename);

public:
  static char ID;
  explicit ResourceConfig() : ImmutablePass(&ID) {};

  ~ResourceConfig();

  virtual void initializePass();

  void print(raw_ostream &OS) const;

  HWResource *getResource(std::string Name) const {
    ResTabTy::const_iterator at = ResTab.find(Name);
    return at == ResTab.end() ? 0 : at->second;
  }
}; //class

} // namespace

#endif // h guard

