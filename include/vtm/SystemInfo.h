//===--------- vtm/SystemInfo.h - SW/HW parition infomation ------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file define the SystemInfo class, which provide the information about
// software/hardware sysinfo.
//
//===----------------------------------------------------------------------===//

#ifndef VTM_SYSTEM_INFO_H
#define VTM_SYSTEM_INFO_H

#include "llvm/Function.h"
#include "llvm/ADT/StringMap.h"


namespace llvm {
// Per-function constraints information.
class ConstraintsInfo {
public:
  enum PipeLineAlgorithm {
    IMS, FDMS, DontPipeline
  };

  enum ScheduleAlgorithm {
    FDS, FDLS, ILP
  };

private:
  PipeLineAlgorithm PipeAlg;
  ScheduleAlgorithm SchedAlg;
  
  friend class LuaScript;
  friend class SystemInfo;

public:
  ConstraintsInfo() : PipeAlg(DontPipeline), SchedAlg(ILP) {}

  PipeLineAlgorithm getPipeLineAlgorithm() const { return PipeAlg; }

  bool enablePipeLine() const { return getPipeLineAlgorithm() != DontPipeline; }

  ScheduleAlgorithm getScheduleAlgorithm() const { return SchedAlg; }

  // Out of line virtual function to provide home for the class.
  virtual void anchor();
};

class SystemInfo {
  // DO NOT IMPLEMENT
  SystemInfo(const SystemInfo&);
  // DO NOT IMPLEMENT
  const SystemInfo &operator=(const SystemInfo&);

  mutable StringMap<ConstraintsInfo> FunctionsInfos;

  void setHardware(const std::string &S) {
    FunctionsInfos.GetOrCreateValue(S);
  }

  // Interfaces.
  std::string hwModName;

  friend class LuaScript;
public:
  SystemInfo() {}

  ConstraintsInfo &getInfo(const std::string &S) const {
    return FunctionsInfos.GetOrCreateValue(S).getValue();
  }

  const std::string &getHwModName() const {
    return hwModName;
  }

  bool isHardware(const Function &F) const {
    // If no explicit sysinfo available, all function are synthesis to
    //  hardware by default.
    if (FunctionsInfos.empty())
      return true;
    
    return FunctionsInfos.count(F.getName());
  }

  bool empty() const { return FunctionsInfos.empty(); }

  // Out of line virtual function to provide home for the class.
  virtual void anchor();
};

const SystemInfo &sysinfo();

}

#endif
