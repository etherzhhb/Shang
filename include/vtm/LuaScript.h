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

// Lua scripting support.
class LuaScript {
  // DO NOT IMPLEMENT
  LuaScript(const LuaScript&);
  // DO NOT IMPLEMENT
  const LuaScript &operator=(const LuaScript&);

  lua_State *State;

  typedef std::map<std::string, tool_output_file*> FileMapTy;
  FileMapTy Files;

  FUInfo FUI;
  SystemInfo SystemI;

  friend const SystemInfo &sysinfo(void);
  friend const FUInfo &vtmfus(void);
public:

  LuaScript();
  ~LuaScript();

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

  // Get the LLVM TargetData string from the constraints.
  std::string getTargetDataStr() const;

  raw_ostream &getOutputStream(const std::string &Name);

  void keepAllFiles();

  void init();

  bool runScript(const std::string &ScriptPath, SMDiagnostic &Err);

};

LuaScript &getScript();

}

#endif
