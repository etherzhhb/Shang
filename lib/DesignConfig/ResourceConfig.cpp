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

#include "vbe/ResourceConfig.h"

#include "llvm/ADT/StringRef.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MemoryBuffer.h"

#define DEBUG_TYPE "vbe-res-config-file"
#include "llvm/Support/Debug.h"

// Include the lua headers (the extern "C" is a requirement because we're
// using C++ and lua has been compiled as C code)
extern "C" {
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
}

// This is the only header we need to include for LuaBind to work
#include "luabind/luabind.hpp"

using namespace llvm;
using namespace esyn;

//===----------------------------------------------------------------------===//
/// Xml stuff
static cl::opt<std::string>
ConfigScriptName("vbe-res-config-script",
                  cl::desc("vbe - The resource config script."));

//===----------------------------------------------------------------------===//
/// Hardware resource.
void HWResType::print(raw_ostream &OS) const {
  // OS << "Resource: " << Name << '\n';
  OS.indent(2) << "TotalNum: " << TotalRes << '\n';
  OS.indent(2) << "Latency: " << Latency << '\n';
  OS.indent(2) << "StartInterval: " << StartInt << '\n';
}

//===----------------------------------------------------------------------===//
/// Resource config implement

template<class BinOpResType>
void ResourceConfig::setupBinOpRes(unsigned latency, unsigned startInt,
                                   unsigned totalRes, unsigned maxBitWidth) {
  ResSet[(unsigned)BinOpResType::getType()
          - (unsigned)HWResType::FirstResourceType]
    = new BinOpResType(latency, startInt, totalRes, maxBitWidth);
}


void ResourceConfig::setupMemBus(unsigned latency, unsigned startInt,
                                 unsigned totalRes, unsigned addrWidth,
                                 unsigned dataWidth) {
  ResSet[(unsigned)HWResType::MemoryBus
          - (unsigned)HWResType::FirstResourceType]
    = new HWMemBus(latency, startInt, totalRes, addrWidth, dataWidth);
}

void ResourceConfig::initializePass() {
  std::string ErrMsg;

  lua_State *ScriptState = lua_open();

  luabind::open(ScriptState);

  luabind::module(ScriptState)[
    luabind::class_<ResourceConfig>("ResourceConfig")
      .def("setupMemBus", &ResourceConfig::setupMemBus)
      .def("setupSHL",    &ResourceConfig::setupBinOpRes<HWSHL>)
      .def("setupASR",    &ResourceConfig::setupBinOpRes<HWASR>)
      .def("setupLSR",    &ResourceConfig::setupBinOpRes<HWLSR>)
      .def("setupAddSub", &ResourceConfig::setupBinOpRes<HWAddSub>)
      .def("setupMult",   &ResourceConfig::setupBinOpRes<HWMult>)
  ];

  luabind::globals(ScriptState)["DesignConfig"] = this;

  luaL_dofile(ScriptState, ConfigScriptName.c_str());

  lua_close(ScriptState);
  DEBUG(print(dbgs(), 0));
}

HWFUnit *ResourceConfig::allocaBinOpFU(HWResType::Types T, unsigned BitWitdh,
                                       unsigned UnitID) {
  FoldingSetNodeID FUID;
  FUID.AddInteger(T);
  FUID.AddInteger(BitWitdh);
  FUID.AddInteger(UnitID);

  void *IP = 0;
  HWFUnit *FU = UniqiueHWFUs.FindNodeOrInsertPos(FUID, IP);
  if (FU) return FU;
  // TODO: Assert bit width smaller than max bit width.
  uint8_t Inputs[] = { BitWitdh, BitWitdh };
  uint8_t *I = HWFUAllocator.Allocate<uint8_t>(2);
  std::uninitialized_copy(Inputs, Inputs + 2, I);
  HWBinOpResType *HWTy = cast<HWBinOpResType>(getResType(T));
  FU = new (HWFUAllocator) HWFUnit(FUID.Intern(HWFUAllocator), T,
                                   HWTy->getTotalRes(), HWTy->getLatency(),
                                   UnitID, I, 2, BitWitdh);
  UniqiueHWFUs.InsertNode(FU, IP);
  return FU;
}


HWFUnit *ResourceConfig::allocaMemBusFU(unsigned UnitID) {
  FoldingSetNodeID FUID;
  FUID.AddInteger(HWResType::MemoryBus);
  FUID.AddInteger(UnitID);

  void *IP = 0;

  HWFUnit *FU = UniqiueHWFUs.FindNodeOrInsertPos(FUID, IP);
  if (FU) return FU;

  HWMemBus *HWTy = getResType<HWMemBus>();
  uint8_t Inputs[] = { HWTy->getDataWidth(), HWTy->getAddrWidth() };
  uint8_t *I = HWFUAllocator.Allocate<uint8_t>(2);
  std::uninitialized_copy(Inputs, Inputs + 2, I);
  FU = new (HWFUAllocator) HWFUnit(FUID.Intern(HWFUAllocator), HWResType::MemoryBus,
                                   1, HWTy->getLatency(), UnitID, I, 2,
                                   HWTy->getDataWidth());
  UniqiueHWFUs.InsertNode(FU, IP);
  return FU;
}


HWFUnit *ResourceConfig::allocaTrivialFU(unsigned Latency, unsigned BitWitdh) {
  FoldingSetNodeID FUID;
  FUID.AddInteger(HWResType::Trivial);
  FUID.AddInteger(Latency);
  FUID.AddInteger(BitWitdh);

  void *IP = 0;

  HWFUnit *FU = UniqiueHWFUs.FindNodeOrInsertPos(FUID, IP);
  if (FU) return FU;

  FU = new (HWFUAllocator) HWFUnit(FUID.Intern(HWFUAllocator), HWResType::Trivial,
                                   ~0, Latency, 0, 0, 0, BitWitdh);
  UniqiueHWFUs.InsertNode(FU, IP);
  return FU;
}

HWFUnit *ResourceConfig::assignIDToFU(HWFUnit *U, unsigned FUID) {
  switch (U->getResType()) {
  case HWResType::Trivial:
  case HWResType::MemoryBus:
    assert(0 && "Bad unit type!");
    return 0;
  default: // FIXME: Use the right bit width.
    return allocaBinOpFU(U->getResType(), U->getInputBitwidth(0), FUID);
  }
}

void ResourceConfig::print(raw_ostream &OS, const Module *) const {
  OS << "-=========================Resource Config=========================-\n";
  for (const_iterator I = begin(), E = end(); I != E; ++I) {
    if (*I != 0) {
      (*I)->print(OS);
      OS << '\n';
    }
  }
}

ResourceConfig::~ResourceConfig() {
  for (iterator I = begin(), E = end(); I != E; ++I)
    delete *I;

  HWFUAllocator.Reset();
  UniqiueHWFUs.clear();
}

char ResourceConfig::ID = 0;

static RegisterPass<ResourceConfig>
X("vbe-resource-config", "vbe - resource config", false, true);
