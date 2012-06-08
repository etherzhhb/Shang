//===--------- vtm/LuaScript.h - Lua Scripting Support ------------===//
//
// Copyright: 2011 by SYSU EDA Group. all rights reserved.
// IMPORTANT: This software is supplied to you by Hongbin Zheng in consideration
// of your agreement to the following terms, and your use, installation,
// modification or redistribution of this software constitutes acceptance
// of these terms.  If you do not agree with these terms, please do not use,
// install, modify or redistribute this software. You may not redistribute,
// install copy or modify this software without written permission from
// Hongbin Zheng.
//
//===----------------------------------------------------------------------===//
//
// This file define the LuaScript class, which provide basic lua scripting
// support.
//
//===----------------------------------------------------------------------===//

#ifndef VTM_LUA_SCRIPT_H
#define VTM_LUA_SCRIPT_H

#include "vtm/FUInfo.h"
#include "vtm/SynSettings.h"

#include "llvm/Function.h"
#include "llvm/ADT/StringMap.h"
#include "llvm/ADT/IndexedMap.h"
#include "llvm/ADT/ArrayRef.h"

// This is the only header we need to include for LuaBind to work
#include "luabind/luabind.hpp"

#include <map>

// Forward declaration.
struct lua_State;

namespace llvm {
// Forward declaration.
class tool_output_file;
class raw_ostream;
class SMDiagnostic;
class PassManager;

// Lua scripting support.
class LuaScript {
  // DO NOT IMPLEMENT
  LuaScript(const LuaScript&);
  // DO NOT IMPLEMENT
  const LuaScript &operator=(const LuaScript&);

  lua_State *State;

  typedef std::map<std::string, tool_output_file*> FileMapTy;
  FileMapTy Files;

  IndexedMap<VFUDesc*, CommonFUIdentityFunctor> FUSet;
  std::string DataLayout;

  StringMap<SynSettings*> FunctionSettings;

  friend SynSettings *getSynSetting(StringRef Name, SynSettings *ParentSetting);
  friend VFUDesc *getFUDesc(enum VFUs::FUTypes T);

  void initSimpleFU(enum VFUs::FUTypes T, luabind::object FUs,
                    unsigned *Costs, float *Latencies);

public:

  LuaScript();
  ~LuaScript();

  template<class T>
  void bindToGlobals(const char *Name, T *O) {
    luabind::globals(State)[Name] = O;
  }

  template<class T>
  T getValue(ArrayRef<const char*> Path) const {
    luabind::object o = luabind::globals(State);
    for (unsigned i = 0; i < Path.size(); ++i)
      o = o[Path[i]];

    boost::optional<T> Res = luabind::object_cast_nothrow<T>(o);

    // If the value not found, just construct then with default constructor.
    if (!Res) return T();

    return Res.get();
  }

  template<class T>
  T getValue(const char *Name) {
    const char *Path[] = { Name };
    return getValue<T>(Path);
  }

  std::string getValueStr(const char *Name) const {
    const char *Path[] = { Name };
    return getValue<std::string>(Path);
  }

  std::string getValueStr(const std::string &Name) const {
    return getValueStr(Name.c_str());
  }

  luabind::object getModTemplate(const std::string &Name) const {
    return luabind::globals(State)["Modules"][Name];
  }

  // Iterator to iterate over all user scripting pass from the constraint script.
  typedef luabind::iterator scriptpass_it;

  scriptpass_it passes_begin() const {
    return scriptpass_it(luabind::globals(State)["Passes"]);
  }

  scriptpass_it passes_end() const {
    return scriptpass_it();
  }

  raw_ostream &getOutputStream(const char *Name);
  raw_ostream &getOutputFileStream(std::string &Name);

  void keepAllFiles();

  void init();
  // Read the Function units information from script engine
  void updateFUs();
  // Update the status of script engine after script run.
  void updateStatus();

  bool runScriptFile(const std::string &ScriptPath, SMDiagnostic &Err);
  bool runScriptStr(const std::string &ScriptStr, SMDiagnostic &Err);

  const std::string &getDataLayout() const { return DataLayout; }
};

LuaScript &scriptEngin();
}

#endif
