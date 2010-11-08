//====- VTMFuctionInfo.h - Verilog target machine function info -*- C++ -*-===//
// 
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
// 
//===----------------------------------------------------------------------===//
//
// This file declares Verilog target machine-specific per-machine-function
// information.
//
//===----------------------------------------------------------------------===//

#ifndef VTM_FUNCTION_INFO_H
#define VTM_FUNCTION_INFO_H

#include "llvm/CodeGen/MachineFunction.h"

#include <map>

namespace llvm {
class MachineBasicBlock;

class VTMFunctionInfo : public MachineFunctionInfo {
  struct StateSlots{
    unsigned startSlot : 32;
    unsigned totalSlot : 16;
    unsigned II        : 16;
  };
  std::map<const MachineBasicBlock*, StateSlots> StateSlotMap;

public:
  explicit VTMFunctionInfo(MachineFunction &MF) {}

  inline unsigned getTotalSlotFor(const MachineBasicBlock* MBB) const {
    std::map<const MachineBasicBlock*, StateSlots>::const_iterator
      at = StateSlotMap.find(MBB);

    assert(at != StateSlotMap.end() && "State not found!");
    return at->second.totalSlot;
  }

  inline unsigned getStartSlotFor(const MachineBasicBlock* MBB) const {
    std::map<const MachineBasicBlock*, StateSlots>::const_iterator
      at = StateSlotMap.find(MBB);

    assert(at != StateSlotMap.end() && "State not found!");
    return at->second.startSlot;
  }

  inline unsigned getIIFor(const MachineBasicBlock* MBB) const {
    std::map<const MachineBasicBlock*, StateSlots>::const_iterator
      at = StateSlotMap.find(MBB);

    assert(at != StateSlotMap.end() && "State not found!");
    return at->second.II;
  }


  void remeberTotalSlot(const MachineBasicBlock* MBB,
                        unsigned startSlot, unsigned totalSlot, unsigned II) {
    StateSlots SS;
    SS.startSlot = startSlot;
    SS.totalSlot = totalSlot;
    SS.II = II;
    StateSlotMap.insert(std::make_pair(MBB, SS));
  }
};

}

#endif
