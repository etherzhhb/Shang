//===----- Scripting.cpp - Scripting engine for verilog backend --*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the LuaScript class, which allow users pass some
// information into the program with lua script. 
//
//===----------------------------------------------------------------------===//
#include "BindingTraits.h"

#include "vtm/LuaScript.h"

#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/ManagedStatic.h"
#include "llvm/Support/ToolOutputFile.h"
#include "llvm/Support/SourceMgr.h"
#include "llvm/ADT/STLExtras.h"

// Include the lua headers (the extern "C" is a requirement because we're
// using C++ and lua has been compiled as C code)
extern "C" {
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
}

using namespace llvm;

// The text template processing lua module
static void openLuapp(lua_State *L) {
#include "luapp.inc"
}

LuaScript::LuaScript() : State(lua_open()) {
  FUSet.grow(VFUs::LastCommonFUType);
}

LuaScript::~LuaScript() {
  // FIXME: Release the function unit descriptors and function settings.
  //for (size_t i = 0, e = array_lengthof(FUSet); i != e; ++i)
  //  if(VFUDesc *Desc = FUSet[i]) delete Desc;

  DeleteContainerSeconds(Files);
  lua_close(State);
}

void LuaScript::keepAllFiles() {
  for (FileMapTy::iterator I = Files.begin(), E = Files.end(); I != E; ++I)
    I->second->keep();
}

void LuaScript::init() {
  // Open lua libraries.
  luaL_openlibs(State);

  openLuapp(State);

  // Bind our class.
  luabind::open(State);

  // Bind the C++ classes.
  luabind::module(State)[
    luabind::class_<SynSettings>("SynSettings")
      .enum_("PipeLine")[
        luabind::value("IMS", SynSettings::IMS),
          luabind::value("FDMS", SynSettings::FDMS),
          luabind::value("ILPMS", SynSettings::ILPMS),
          luabind::value("DontPipeline", SynSettings::DontPipeline)
      ]
      .enum_("Schedule")[
        luabind::value("FDS", SynSettings::FDS),
        luabind::value("FDLS", SynSettings::FDLS),
        luabind::value("ILP", SynSettings::ILP),
        luabind::value("ASAP", SynSettings::ASAP)
      ],

    BindingTraits<VASTPort>::register_("VASTPort"),

    BindingTraits<VASTModule>::register_("VASTModule")
  ];

  // Bind the object.
  luabind::globals(State)["FUs"] = luabind::newtable(State);
  luabind::globals(State)["Functions"] = luabind::newtable(State);
  luabind::globals(State)["Modules"] = luabind::newtable(State);
  // The scripting pass table.
  luabind::globals(State)["Passes"] = luabind::newtable(State);
}

bool LuaScript::runScriptStr(const std::string &ScriptStr, SMDiagnostic &Err) {
  // Run the script.
  if (luaL_dostring(State, ScriptStr.c_str())) {
    Err = SMDiagnostic(ScriptStr, lua_tostring(State, -1));
    return false;
  }

  return true;
}

bool LuaScript::runScriptFile(const std::string &ScriptPath, SMDiagnostic &Err) {
  // Run the script.
  if (luaL_dofile(State, ScriptPath.c_str())) {
    Err = SMDiagnostic(ScriptPath, lua_tostring(State, -1));
    return false;
  }

  return true;
}

raw_ostream &LuaScript::getOutputStream(const std::string &Name) {
  std::string Path = getValueStr(Name);

  // Try to return the existing file.
  FileMapTy::const_iterator at = Files.find(Path);

  if (at != Files.end()) return at->second->os();

  if (Path.empty()) return outs();

  std::string error;

  tool_output_file *NewFile = new tool_output_file(Path.c_str(), error);
  // TODO: Support binary file.
  Files.insert(std::make_pair(Path, NewFile));

  return NewFile->os();
}

void LuaScript::initSimpleFU(enum VFUs::FUTypes T, luabind::object FUs) {
  FUSet[T] = new VFUDesc(T, FUs[VFUDesc::getTypeName(T)]);
}

void LuaScript::updateStatus() {
  luabind::object FUs = luabind::globals(State)["FUs"];
  FUSet[VFUs::MemoryBus]
    = new VFUMemBus(FUs[VFUDesc::getTypeName(VFUs::MemoryBus)]);
  FUSet[VFUs::BRam] = new VFUBRam(FUs[VFUDesc::getTypeName(VFUs::BRam)]);

  initSimpleFU(VFUs::AddSub, FUs);
  initSimpleFU(VFUs::Shift, FUs);
  initSimpleFU(VFUs::Mult, FUs);

  typedef luabind::iterator tab_it;
  for (tab_it I = tab_it(luabind::globals(State)["Functions"]), E = tab_it();
       I != E; ++I) {
    FunctionSettings.GetOrCreateValue(
      luabind::object_cast<std::string>(I.key()).c_str(),
      new SynSettings(*I));
  }

  // Build the data layout.
  raw_string_ostream s(DataLayout);

  // FIXME: Set the correct endian.
  s << 'e';

  s << '-';

  // Setup the address width (pointer width).
  unsigned PtrSize = getFUDesc<VFUMemBus>()->getAddrWidth();
  s << "p:" << PtrSize << ':' << PtrSize << ':' << PtrSize << '-';

  // FIXME: Setup the correct integer layout.
  s << "i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:64:64-";
  s << "n8:16:32:64";

  s.flush();
}

ManagedStatic<LuaScript> Script;

VFUDesc *llvm::getFUDesc(enum VFUs::FUTypes T) {
  return Script->FUSet[T];
}

SynSettings *llvm::getSynSetting(StringRef Name, SynSettings *ParentSetting) {
  StringMap<SynSettings*> *SynSettingMap = &Script->FunctionSettings;
  if (SynSettings *S = SynSettingMap->lookup(Name))
    return S;

  if (!ParentSetting) return 0;

  // Create a new synthesis setting by copying ParentSetting.
  SynSettings *S = new SynSettings(Name, *ParentSetting);
  StringMapEntry<SynSettings*> *Entry =
    StringMapEntry<SynSettings*>::Create(Name.begin(), Name.end(),
                                         SynSettingMap->getAllocator(), S);
  SynSettingMap->insert(Entry);
  return S;
}

LuaScript &llvm::scriptEngin() {
  return *Script;
}

// Dirty Hack: Allow we invoke some scripting function in the libraries
// compiled with no-rtti
void bindToScriptEngine(const char *name, VASTModule *M) {
  Script->bindToGlobals(name, M);
}

bool runScriptFile(const std::string &ScriptPath, SMDiagnostic &Err) {
  return Script->runScriptFile(ScriptPath, Err);
}

bool runScriptStr(const std::string &ScriptStr, SMDiagnostic &Err) {
  return Script->runScriptStr(ScriptStr, Err);
}
