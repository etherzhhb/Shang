//===-------------- VSubtarget.cpp - VTM Subtarget Information -------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the vtm specific subclass of TargetSubtarget.
//
//===----------------------------------------------------------------------===//

#include "esly/VSubtarget.h"

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

//===----------------------------------------------------------------------===//
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
void VSubtarget::setupBinOpRes(unsigned latency, unsigned startInt,
                                   unsigned totalRes, unsigned maxBitWidth) {
  ResSet[(unsigned)BinOpResType::getType()
          - (unsigned)HWResType::FirstResourceType]
    = new BinOpResType(latency, startInt, totalRes, maxBitWidth);
}


void VSubtarget::setupMemBus(unsigned latency, unsigned startInt,
                                 unsigned totalRes, unsigned addrWidth,
                                 unsigned dataWidth) {
  ResSet[(unsigned)HWResType::MemoryBus
          - (unsigned)HWResType::FirstResourceType]
    = new HWMemBus(latency, startInt, totalRes, addrWidth, dataWidth);
}

void VSubtarget::initializeTarget() {
  std::string ErrMsg;

  lua_State *ScriptState = lua_open();

  luabind::open(ScriptState);

  luabind::module(ScriptState)[
    luabind::class_<VSubtarget>("VSubtarget")
      .def("setupMemBus", &VSubtarget::setupMemBus)
      .def("setupSHL",    &VSubtarget::setupBinOpRes<HWSHL>)
      .def("setupASR",    &VSubtarget::setupBinOpRes<HWASR>)
      .def("setupLSR",    &VSubtarget::setupBinOpRes<HWLSR>)
      .def("setupAddSub", &VSubtarget::setupBinOpRes<HWAddSub>)
      .def("setupMult",   &VSubtarget::setupBinOpRes<HWMult>)
  ];

  luabind::globals(ScriptState)["DesignConfig"] = this;

  luaL_dofile(ScriptState, ConfigScriptName.c_str());

  lua_close(ScriptState);
  DEBUG(print(dbgs(), 0));
}

HWFUnit *VSubtarget::allocaBinOpFU(HWResType::Types T, unsigned BitWitdh,
                                   unsigned UnitID) const {
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


HWFUnit *VSubtarget::allocaMemBusFU(unsigned UnitID) const {
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


HWFUnit *VSubtarget::allocaTrivialFU(unsigned Latency, unsigned BitWitdh) const {
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

HWFUnit *VSubtarget::assignIDToFU(HWFUnit *U, unsigned FUID) const {
  switch (U->getResType()) {
  case HWResType::Trivial:
  case HWResType::MemoryBus:
    assert(0 && "Bad unit type!");
    return 0;
  default: // FIXME: Use the right bit width.
    return allocaBinOpFU(U->getResType(), U->getInputBitwidth(0), FUID);
  }
}

void VSubtarget::print(raw_ostream &OS, const Module *) const {
  OS << "-=========================Resource Config=========================-\n";
  for (const_iterator I = begin(), E = end(); I != E; ++I) {
    if (*I != 0) {
      (*I)->print(OS);
      OS << '\n';
    }
  }
}

VSubtarget::VSubtarget(const std::string &TT,
                       const std::string &FS) {
  std::string CPU = "generic";

  for (size_t i = 0, e = (size_t)HWResType::LastResourceType; i != e; ++i)
    ResSet[i] = 0;
  // TODO: Parse lua script here.
  initializeTarget();
}

VSubtarget::~VSubtarget() {
  for (iterator I = begin(), E = end(); I != E; ++I)
    delete *I;

  HWFUAllocator.Reset();
  UniqiueHWFUs.clear();
}