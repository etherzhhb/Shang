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

#ifndef LLVM_PARAMS_H
#define LLVM_PARAMS_H

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

#include <map>
#include <sstream>

using namespace llvm;

namespace esyn {

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

  HWResource(const HWResource &);            // DO NOT IMPLEMENT
  void operator=(const HWResource &);  // DO NOT IMPLEMENT
public:
  static const unsigned Infinite = UINT32_MAX;

  explicit HWResource(std::string name,
    unsigned latency, unsigned startInt, unsigned totalRes)
    : Name(name), Latency(latency), StartInt(startInt), TotalRes(totalRes) {}
  
  unsigned getLatency() const { return Latency; }
  unsigned getTotalRes() const { return TotalRes; }
  bool isInfinite() const { return TotalRes == Infinite; }
  unsigned getStartInt() const { return StartInt; }
  const std::string &getName() const { return Name; }

  void print(raw_ostream &OS) const;

  // TODO: how to instance this resource.
}; 

/// @brief Resource Table
class HWResTable {
  /// The resource and the instances left

  /// Get the least busy resource of a given kind
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

