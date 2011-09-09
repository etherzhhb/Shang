//===--------- vtm/SynSettings.h - SW/HW parition infomation ------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file define the SynSettings class, which provide the information about
// software/hardware sysinfo.
//
//===----------------------------------------------------------------------===//

#ifndef VTM_SYSTEM_INFO_H
#define VTM_SYSTEM_INFO_H

#include "llvm/Function.h"
#include "llvm/ADT/StringMap.h"

namespace luabind {
  namespace adl {
    class object;
  }
  using adl::object;
}

namespace llvm {
// Per-function constraints information.
class SynSettings {
public:
  enum PipeLineAlgorithm {
    IMS, FDMS, ILPMS, DontPipeline
  };

  enum ScheduleAlgorithm {
    FDS, FDLS, ILP, ASAP
  };

private:
  PipeLineAlgorithm PipeAlg;
  ScheduleAlgorithm SchedAlg;
  // Rtl module name.
  std::string ModName;
  // Hierarchy prefix
  std::string HierPrefix;
  bool IsTopLevelModule;

  friend class LuaScript;
public:
  SynSettings(luabind::object SettingTable);
  SynSettings(StringRef Name, SynSettings &From);
  PipeLineAlgorithm getPipeLineAlgorithm() const { return PipeAlg; }

  bool isTopLevelModule() const { return IsTopLevelModule; }
  void setTopLevelModule(bool isTop) { IsTopLevelModule = isTop; }
  bool enablePipeLine() const { return getPipeLineAlgorithm() != DontPipeline; }

  ScheduleAlgorithm getScheduleAlgorithm() const { return SchedAlg; }

  const std::string &getModName() const { return ModName; }
  const std::string &getHierPrefix() const { return HierPrefix; }
};

SynSettings *getSynSetting(StringRef Name, SynSettings *ParentSetting = 0);
}

#endif
