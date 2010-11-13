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

#include "VFunctionUnit.h"

#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/ADT/STLExtras.h"

#include <map>

namespace llvm {
class MachineBasicBlock;

class VTMFunctionInfo : public MachineFunctionInfo {
  // Information about slots.
  struct StateSlots{
    unsigned startSlot : 32;
    unsigned totalSlot : 16;
    unsigned II        : 16;
  };
  std::map<const MachineBasicBlock*, StateSlots> StateSlotMap;

  // Information about allocated function unit.
  // Mapping FUTypes to allocate function unit identity number.
  typedef std::multimap<VFUs::FUTypes, unsigned> FUIdMapTy;
  FUIdMapTy AllocatedFUs;

  // Helper struct for Id iterator.
  struct IdMapper {
    typedef FUIdMapTy::mapped_type result_type;
    result_type operator()(FUIdMapTy::value_type v) const {
      return v.second;
    } 
  };

public:
  explicit VTMFunctionInfo(MachineFunction &MF) {}

  /// Slots information for machine basicblock.

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

  /// Information for allocated function units.

  void rememberAllocatedFU(VFUs::FUTypes FUType, unsigned Id) {
    // Sometimes there are several instructions allocated to the same instruction,
    // and it is ok to try to insert the same FUId more than once.
    AllocatedFUs.insert(std::make_pair(FUType, Id));
  }

  typedef mapped_iterator<FUIdMapTy::const_iterator, IdMapper> const_id_iterator;

  const_id_iterator id_begin(VFUs::FUTypes FUType = VFUs::AllFUType) const {
    assert(FUType != VFUs::AllFUType && "AllFUType not supported now!");
    
    return const_id_iterator(AllocatedFUs.lower_bound(FUType), IdMapper());
  }

  const_id_iterator id_end(VFUs::FUTypes FUType = VFUs::AllFUType) const {
    assert(FUType != VFUs::AllFUType && "AllFUType not supported now!");

    return const_id_iterator(AllocatedFUs.upper_bound(FUType), IdMapper());
  }

};

}

#endif
