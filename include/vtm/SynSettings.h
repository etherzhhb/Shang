//===--------- vtm/SynSettings.h - SW/HW parition infomation ------------===//
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
    IMS, DontPipeline
  };

  enum ScheduleAlgorithm {
    ASAP, SDC
  };

private:
  PipeLineAlgorithm PipeAlg;
  ScheduleAlgorithm SchedAlg;
  // Rtl module name.
  std::string ModName;
  // Hierarchy prefix
  std::string InstName;
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
  const std::string &getInstName() const { return InstName; }

  static inline const std::string getIfPostfix() { return "_if"; }
};

SynSettings *getSynSetting(StringRef Name, SynSettings *ParentSetting = 0);
}

#endif
