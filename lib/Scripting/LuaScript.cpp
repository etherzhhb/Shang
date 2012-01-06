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

#include "vtm/Passes.h"
#include "vtm/LuaScript.h"
#include "vtm/VInstrInfo.h"
#include "vtm/Utilities.h"

#include "llvm/PassManager.h"
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
  // Synthesis attribute
  luabind::globals(State)["SynAttr"] = luabind::newtable(State);
  // Table for Miscellaneous information
  luabind::globals(State)["Misc"] = luabind::newtable(State);
}

bool LuaScript::runScriptStr(const std::string &ScriptStr, SMDiagnostic &Err) {
  // Run the script.
  if (luaL_dostring(State, ScriptStr.c_str())) {
    Err = SMDiagnostic(ScriptStr, SourceMgr::DK_Warning, lua_tostring(State, -1));
    return false;
  }

  return true;
}

bool LuaScript::runScriptFile(const std::string &ScriptPath, SMDiagnostic &Err) {
  // Run the script.
  if (luaL_dofile(State, ScriptPath.c_str())) {
    Err = SMDiagnostic(ScriptPath, SourceMgr::DK_Warning, lua_tostring(State, -1));
    return false;
  }

  return true;
}

raw_ostream &LuaScript::getOutputStream(const char *Name) {
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

raw_ostream &LuaScript::getOutputFileStream(std::string &Name) {

  std::string Path = Name;

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

void LuaScript::initSimpleFU(enum VFUs::FUTypes T, luabind::object FUs,
                             double *Latencies) {
  FUSet[T] = new VFUDesc(T, FUs[VFUDesc::getTypeName(T)], Latencies);
}

void LuaScript::updateFUs() {
  luabind::object FUs = luabind::globals(State)["FUs"];
  FUSet[VFUs::MemoryBus]
    = new VFUMemBus(FUs[VFUDesc::getTypeName(VFUs::MemoryBus)]);
  FUSet[VFUs::BRam] = new VFUBRam(FUs[VFUDesc::getTypeName(VFUs::BRam)]);

  initSimpleFU(VFUs::AddSub, FUs, VFUs::AdderLatencies);
  // Override the cost parameter if user provided.
  if (unsigned AdderCost = FUSet[VFUs::AddSub]->getCost())
    VFUs::AddCost = AdderCost;

  initSimpleFU(VFUs::Shift, FUs, VFUs::ShiftLatencies);
  if (unsigned ShiftCost = FUSet[VFUs::Shift]->getCost())
    VFUs::ShiftCost = ShiftCost;

  initSimpleFU(VFUs::Mult, FUs, VFUs::MultLatencies);
  if (unsigned MulCost = FUSet[VFUs::Mult]->getCost())
    VFUs::MulCost = MulCost;

  initSimpleFU(VFUs::ICmp, FUs, VFUs::CmpLatencies);
  if (unsigned ICmpCost = FUSet[VFUs::ICmp]->getCost())
    VFUs::ICmpCost = ICmpCost;

  // Read other parameters.
#define READPARAMETER(PARAMETER, T) \
  if (boost::optional<T> PARAMETER \
      = luabind::object_cast_nothrow<T>(FUs[#PARAMETER])) \
    VFUs::PARAMETER = PARAMETER.get();

  READPARAMETER(LUTCost, unsigned);
  READPARAMETER(RegCost, unsigned);
  READPARAMETER(MUXCost, unsigned);
  READPARAMETER(MuxSizeCost, unsigned);

  READPARAMETER(LutLatency, double);
  READPARAMETER(MaxLutSize, unsigned);
  READPARAMETER(MaxMuxPerLut, unsigned);
}

void LuaScript::updateStatus() {
  updateFUs();

  // Read the synthesis attributes.
  const char *Path[] = { "SynAttr", "DirectClkEnAttr" };
  VASTModule::DirectClkEnAttr = getValue<std::string>(Path);
  Path[1] = "ParallelCaseAttr";
  VASTModule::ParallelCaseAttr = getValue<std::string>(Path);
  Path[1] = "FullCaseAttr";
  VASTModule::FullCaseAttr = getValue<std::string>(Path);

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
  StringMapEntry<SynSettings*> &Entry = SynSettingMap->GetOrCreateValue(Name);
  SynSettings *&S = Entry.second;
  if (S) return S;

  if (!ParentSetting) return 0;

  // Create a new synthesis setting by copying ParentSetting.
  S = SynSettingMap->getAllocator().Allocate<SynSettings>();
  new (S) SynSettings(Name, *ParentSetting);
  return S;
}

LuaScript &llvm::scriptEngin() {
  return *Script;
}

// Dirty Hack: Allow we invoke some scripting function in the libraries
// compiled with no-rtti
void llvm::bindToScriptEngine(const char *name, VASTModule *M) {
  Script->bindToGlobals(name, M);
}

unsigned llvm::getIntValueFromEngine(ArrayRef<const char*> Path) {
  return Script->getValue<unsigned>(Path);
}

std::string llvm::getStrValueFromEngine(ArrayRef<const char*> Path) {
  return Script->getValue<std::string>(Path);
}

bool llvm::runScriptFile(const std::string &ScriptPath, SMDiagnostic &Err) {
  return Script->runScriptFile(ScriptPath, Err);
}

bool llvm::runScriptStr(const std::string &ScriptStr, SMDiagnostic &Err) {
  return Script->runScriptStr(ScriptStr, Err);
}
