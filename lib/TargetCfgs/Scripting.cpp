//===----- Scripting.cpp - Scripting engine for verilog backend --*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// 
//
//===----------------------------------------------------------------------===//
#include "vtm/FileInfo.h"
#include "vtm/FUInfo.h"
#include "vtm/PartitionInfo.h"

#include "llvm/Support/CommandLine.h"

// This is the only header we need to include for LuaBind to work
#include "luabind/luabind.hpp"


// Include the lua headers (the extern "C" is a requirement because we're
// using C++ and lua has been compiled as C code)
extern "C" {
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
}

using namespace llvm;

namespace llvm {
struct LuaConstraints {
  // DO NOT IMPLEMENT
  LuaConstraints(const LuaConstraints&);
  // DO NOT IMPLEMENT
  const LuaConstraints &operator=(const LuaConstraints&);

  lua_State *State;
  
  FUInfo FUI;
  FileInfo FileI;
  PartitionInfo PartitionI;
  
  LuaConstraints() : State(lua_open()) {
    luabind::open(State);
  }

  ~LuaConstraints() {
    lua_close(State);
  }
};

struct ConstraintsParser : public cl::basic_parser<LuaConstraints> {
  // parse - Return true on error.
  bool parse(cl::Option &O, StringRef ArgName, const std::string &ArgValue,
             LuaConstraints &Val);
};
}

bool ConstraintsParser::parse(cl::Option &O, StringRef ArgName,
                              const std::string &ArgValue, LuaConstraints &Val) {
  lua_State *ScriptState = Val.State;

  luabind::module(ScriptState)[
    luabind::class_<FUInfo>("FUInfo")
      .def("setupMemBus", &FUInfo::setupMemBus)
      .def("setupSHL",    &FUInfo::setupBinOpRes<VFUSHL>)
      .def("setupASR",    &FUInfo::setupBinOpRes<VFUASR>)
      .def("setupLSR",    &FUInfo::setupBinOpRes<VFULSR>)
      .def("setupAddSub", &FUInfo::setupBinOpRes<VFUAddSub>)
      .def("setupMult",   &FUInfo::setupBinOpRes<VFUMult>),

      luabind::class_<FileInfo>("FileInfo")
        .property("OutFilesDir", &FileInfo::getOutFilesDir,
                  &FileInfo::setOutFilesDir)
        .def_readwrite("SystemName", &FileInfo::SystemName)
        .def_readwrite("WriteAllToStdOut", &FileInfo::WriteAllToStdOut),

      luabind::class_<PartitionInfo>("PartitionInfo")
        .def("setHardware", &PartitionInfo::setHardware)
  ];

  luabind::globals(ScriptState)["FUs"] = &Val.FUI;
  luabind::globals(ScriptState)["Paths"] = &Val.FileI;
  luabind::globals(ScriptState)["Partition"] = &Val.PartitionI;

  luaL_dofile(ScriptState, ArgValue.c_str());

  return false;
}


static cl::opt<LuaConstraints, false, ConstraintsParser>
DesignConstraints("vtm-constraint",
                 cl::desc("The constraints script for the synthesis target."));


FUInfo &llvm::vtmfus() {
  return DesignConstraints.getValue().FUI;
}

FileInfo &llvm::vtmfiles() {
  return DesignConstraints.getValue().FileI;
}

PartitionInfo &llvm::partition() {
  return DesignConstraints.getValue().PartitionI;
}
