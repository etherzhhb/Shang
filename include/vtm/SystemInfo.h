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
  
  friend struct LuaConstraints;
  friend class SystemInfo;

public:
  ConstraintsInfo() : PipeAlg(DontPipeline), SchedAlg(ILP) {}

  PipeLineAlgorithm getPipeLineAlgorithm() const { return PipeAlg; }

  bool enablePipeLine() const { return getPipeLineAlgorithm() != DontPipeline; }

  ScheduleAlgorithm getScheduleAlgorithm() const { return SchedAlg; }
};

class SystemInfo {
  // DO NOT IMPLEMENT
  SystemInfo(const SystemInfo&);
  // DO NOT IMPLEMENT
  const SystemInfo &operator=(const SystemInfo&);

  StringMap<ConstraintsInfo> FunctionsInfos;

  void setHardware(const std::string &S) {
    FunctionsInfos.GetOrCreateValue(S);
  }

  // Interfaces.
  bool enableVLT, enablePLB;

  friend struct LuaConstraints;
public:
  SystemInfo() : enableVLT(false), enablePLB(false) {}

  ConstraintsInfo &getConstraints(const std::string &S) {
    return FunctionsInfos.GetOrCreateValue(S).getValue();
  }

  bool isHardware(const Function &F) const {
    // If no explicit sysinfo available, all function are synthesis to
    //  hardware by default.
    if (FunctionsInfos.empty())
      return true;
    
    return FunctionsInfos.count(F.getName());
  }

  bool empty() const { return FunctionsInfos.empty(); }

  bool isVLTEnable() const { return enableVLT; }
  bool isPLBEnable() const { return enablePLB; }
};

SystemInfo &sysinfo();

}

#endif
