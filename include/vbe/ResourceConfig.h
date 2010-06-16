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

using namespace llvm;

namespace esyn {
class HWAOpRes;

/// @brief Represent hardware resource
class HWResource {
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

  explicit HWResource(std::string name,
    unsigned latency, unsigned startInt, unsigned totalRes)
    : Name(name), Latency(latency), StartInt(startInt),
      TotalRes(totalRes), UsingCount(totalRes != UINT32_MAX ? totalRes : 0) {
    //
    clear();
  }
  
  unsigned getLatency() const { return Latency; }
  unsigned getTotalRes() const { return TotalRes; }
  bool isInfinite() const { return TotalRes == Infinite; }
  unsigned getStartInt() const { return StartInt; }
  const std::string &getName() const { return Name; }

  void print(raw_ostream &OS) const;

  void addUsingAtom(HWAOpRes *Atom) {
    UsingAtoms.insert(Atom);
  } 

  size_t getUsingCount(unsigned idx = 0) const {
    assert(idx <= UsingCount.size() + 1 && "idx out of range!");
    if (idx == 0)
      return UsingAtoms.size(); 
    else
      return UsingCount[idx - 1];
  }

  void assignToInstance(unsigned instance) {
    ++UsingCount[instance - 1];
  }

  unsigned getLeastBusyInstance() const;

  void clear();
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

