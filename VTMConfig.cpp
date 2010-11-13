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

#define DEBUG_TYPE "vtmconfig"
#include "VTMConfig.h"

#include "VFunctionUnit.h"

#define VTMSubtarget VTMConfig
#include "VGenSubtarget.inc"

#include "VTM.h"

#include "llvm/ADT/StringRef.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"

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
/// Resource config implement

template<class BinOpResType>
void VTMConfig::setupBinOpRes(unsigned latency, unsigned startInt,
                                   unsigned totalRes, unsigned maxBitWidth) {
  ResSet[BinOpResType::getType() - VFUs::FirstFUType]
    = new BinOpResType(latency, startInt, totalRes, maxBitWidth);
}


void VTMConfig::setupMemBus(unsigned latency, unsigned startInt,
                                 unsigned totalRes, unsigned addrWidth,
                                 unsigned dataWidth) {
  ResSet[VFUs::MemoryBus - VFUs::FirstFUType]
    = new VFUMemBus(latency, startInt, totalRes, addrWidth, dataWidth);
}

void VTMConfig::initializeTarget() {
  std::string ErrMsg;

  lua_State *ScriptState = lua_open();

  luabind::open(ScriptState);

  luabind::module(ScriptState)[
    luabind::class_<VTMConfig>("VSubtarget")
      .def("setupMemBus", &VTMConfig::setupMemBus)
      .def("setupSHL",    &VTMConfig::setupBinOpRes<VFUSHL>)
      .def("setupASR",    &VTMConfig::setupBinOpRes<VFUASR>)
      .def("setupLSR",    &VTMConfig::setupBinOpRes<VFULSR>)
      .def("setupAddSub", &VTMConfig::setupBinOpRes<VFUAddSub>)
      .def("setupMult",   &VTMConfig::setupBinOpRes<VFUMult>)
  ];

  luabind::globals(ScriptState)["DesignConfig"] = this;

  luaL_dofile(ScriptState, ConfigScriptName.c_str());

  lua_close(ScriptState);
  DEBUG(print(dbgs(), 0));
}

void VTMConfig::print(raw_ostream &OS, const Module *) const {
  OS << "-=========================Resource Config=========================-\n";
  for (const_iterator I = begin(), E = end(); I != E; ++I) {
    if (*I != 0) {
      (*I)->print(OS);
      OS << '\n';
    }
  }
}

VTMConfig::VTMConfig(const std::string &TT,
                           const std::string &FS) {
  std::string CPU = "generic";
  // Parse features string.
  ParseSubtargetFeatures(FS, CPU);

  for (size_t i = 0, e = array_lengthof(ResSet); i != e; ++i)
    ResSet[i] = 0;

  // TODO: Parse lua script here.
  initializeTarget();
}

VTMConfig::~VTMConfig() {
  for (iterator I = begin(), E = end(); I != E; ++I)
    delete *I;
}
