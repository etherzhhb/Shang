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
#include "llvm/Support/ManagedStatic.h"

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

static cl::opt<std::string>
ConstraintsFile("vtm-constraint",
                cl::desc("The constraints script for the synthesis target."));

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

    luabind::module(State)[
      luabind::class_<FUInfo>("FUInfo")
        .def("setupMemBus", &FUInfo::setupMemBus)
        .def("setupSHL",    &FUInfo::setupBinOpRes<VFUs::SHL>)
        .def("setupASR",    &FUInfo::setupBinOpRes<VFUs::ASR>)
        .def("setupLSR",    &FUInfo::setupBinOpRes<VFUs::LSR>)
        .def("setupAddSub", &FUInfo::setupBinOpRes<VFUs::AddSub>)
        .def("setupMult",   &FUInfo::setupBinOpRes<VFUs::Mult>),

      luabind::class_<FileInfo>("FileInfo")
        .property("OutFilesDir", &FileInfo::getOutFilesDir,
        &FileInfo::setOutFilesDir)
        .def_readwrite("SystemName", &FileInfo::SystemName)
        .def_readwrite("WriteAllToStdOut", &FileInfo::WriteAllToStdOut),

      luabind::class_<PartitionInfo>("PartitionInfo")
        .def("setHardware", &PartitionInfo::setHardware)
    ];

    luabind::globals(State)["FUs"] = &FUI;
    luabind::globals(State)["Paths"] = &FileI;
    luabind::globals(State)["Partition"] = &PartitionI;

    luaL_dofile(State, ConstraintsFile.c_str());

  }

  ~LuaConstraints() {
    lua_close(State);
  }
};
}

static ManagedStatic<LuaConstraints> Constraints;


FUInfo &llvm::vtmfus() {
  return Constraints->FUI;
}

FileInfo &llvm::vtmfiles() {
  return Constraints->FileI;
}

PartitionInfo &llvm::partition() {
  return Constraints->PartitionI;
}
