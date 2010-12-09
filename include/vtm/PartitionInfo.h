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
#include "llvm/ADT/StringSet.h"


namespace llvm {

class PartitionInfo {

  // DO NOT IMPLEMENT
  PartitionInfo(const PartitionInfo&);
  // DO NOT IMPLEMENT
  const PartitionInfo &operator=(const PartitionInfo&);

  StringSet<> HWFunctions;

  void setHardware(const std::string &S) {
    HWFunctions.insert(S);
  }

  friend struct LuaConstraints;
public:
  PartitionInfo() {}

  bool isHardware(const Function &F) const {
    // If no explicit partition available, all function are synthesis to
    //  hardware by default.
    if (HWFunctions.empty())
      return true;
    
    return HWFunctions.count(F.getName());
  }

  bool empty() const { return HWFunctions.empty(); }
};

PartitionInfo &partition();

}

#endif
