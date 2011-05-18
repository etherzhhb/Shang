//===--------- vtm/LuaScript.h - Lua Scripting Support ------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
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
#include "vtm/SystemInfo.h"

#include "llvm/Function.h"
#include "llvm/ADT/StringMap.h"
#include "llvm/ADT/IndexedMap.h"

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

  SystemInfo SystemI;

  friend const SystemInfo &sysinfo(void);
  friend VFUDesc *getFUDesc(enum VFUs::FUTypes T);

  void initSimpleFU(enum VFUs::FUTypes T, luabind::object FUs);

public:

  LuaScript();
  ~LuaScript();

  template<class T>
  void bindToGlobals(const char *Name, T *O) {
    luabind::globals(State)[Name] = O;
  }

  template<class T>
  T getValue(const std::string &Name) const {
    boost::optional<T> Res =
      luabind::object_cast_nothrow<T>(luabind::globals(State)[Name]);

    // If the value not found, just construct then with default constructor.
    if (!Res) return T();

    return Res.get();
  }

  std::string getValueStr(const std::string &Name) const {
    return getValue<std::string>(Name);
  }

  // Iterator to iterate over all user scripting pass from the constraint script.
  typedef luabind::iterator scriptpass_it;

  scriptpass_it passes_begin() const {
    return scriptpass_it(luabind::globals(State)["Passes"]);
  }

  scriptpass_it passes_end() const {
    return scriptpass_it();
  }

  raw_ostream &getOutputStream(const std::string &Name);

  void keepAllFiles();

  void init();
  // Update the status of script engine after script run.
  void updateStatus();

  bool runScriptFile(const std::string &ScriptPath, SMDiagnostic &Err);
  bool runScriptStr(const std::string &ScriptStr, SMDiagnostic &Err);

  const std::string &getDataLayout() const { return DataLayout; }

  // Out of line virtual function to provide home for the class.
  virtual void anchor();
};

LuaScript &scriptEngin();

}

#endif
