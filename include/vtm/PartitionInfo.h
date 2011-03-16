//===--------- vtm/PartitionInfo.h - SW/HW parition infomation ------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file define the PartitionInfo class, which provide the information about
// software/hardware partition.
//
//===----------------------------------------------------------------------===//

#ifndef VTM_PARTITION_INFO_H
#define VTM_PARTITION_INFO_H

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
  friend class PartitionInfo;

public:
  ConstraintsInfo() : PipeAlg(DontPipeline), SchedAlg(FDLS) {}

  PipeLineAlgorithm getPipeLineAlgorithm() const { return PipeAlg; }

  bool enablePipeLine() const { return getPipeLineAlgorithm() != DontPipeline; }

  ScheduleAlgorithm getScheduleAlgorithm() const { return SchedAlg; }
};

class PartitionInfo {

  // DO NOT IMPLEMENT
  PartitionInfo(const PartitionInfo&);
  // DO NOT IMPLEMENT
  const PartitionInfo &operator=(const PartitionInfo&);

  StringMap<ConstraintsInfo> FunctionsInfos;

  void setHardware(const std::string &S) {
    FunctionsInfos.GetOrCreateValue(S);
  }

  friend struct LuaConstraints;
public:
  PartitionInfo() {}

  ConstraintsInfo &getConstraints(const std::string &S) {
    return FunctionsInfos.GetOrCreateValue(S).getValue();
  }

  bool isHardware(const Function &F) const {
    // If no explicit partition available, all function are synthesis to
    //  hardware by default.
    if (FunctionsInfos.empty())
      return true;
    
    return FunctionsInfos.count(F.getName());
  }

  bool empty() const { return FunctionsInfos.empty(); }
};

PartitionInfo &partition();

}

#endif
