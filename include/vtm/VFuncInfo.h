//====------ VFunInfo.h - Verilog target machine function info --*- C++ -*-===//
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

#include "vtm/FUInfo.h"

#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/Support/StringPool.h"
#include "llvm/ADT/STLExtras.h"

#include <set>
#include <map>

namespace llvm {
class MachineBasicBlock;

class VFuncInfo : public MachineFunctionInfo {
  // Information about slots.
  struct StateSlots{
    unsigned startSlot : 32;
    unsigned totalSlot : 16;
    unsigned IISlot        : 16;
  };
  std::map<const MachineBasicBlock*, StateSlots> StateSlotMap;

  // Information about allocated function unit.
  // Mapping FUTypes to allocate function unit identity number.
  std::set<FuncUnitId> AllocatedFUs[VFUs::NumFUs];

  struct FUActiveSlot {
    union {
      struct FUSlot {
        uint16_t Id;
        uint16_t Slot;
      } Struct;

      uint32_t data;
    } Union;

    inline bool operator==(const FUActiveSlot X) const {
      return Union.data == X.Union.data;
    }
    inline bool operator< (const FUActiveSlot X) const {
      return Union.data < X.Union.data;
    }

    FUActiveSlot(FuncUnitId Id = FuncUnitId(), unsigned Slot = 0) {
      Union.Struct.Id = Id.getData();
      Union.Struct.Slot = Slot;
    }
  };

  typedef std::set<FUActiveSlot> FUActiveSlotSetTy;
  FUActiveSlotSetTy ActiveSlotSet;

  StringPool SymbolPool;
  std::set<PooledStringPtr> Symbols;
public:
  explicit VFuncInfo(MachineFunction &MF) {}

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

  inline unsigned getIISlotFor(const MachineBasicBlock* MBB) const {
    std::map<const MachineBasicBlock*, StateSlots>::const_iterator
      at = StateSlotMap.find(MBB);

    assert(at != StateSlotMap.end() && "State not found!");
    return at->second.IISlot;
  }

  void remeberTotalSlot(const MachineBasicBlock* MBB,
                        unsigned startSlot, unsigned totalSlot, unsigned IISlot) {
    StateSlots SS;
    SS.startSlot = startSlot;
    SS.totalSlot = totalSlot;
    SS.IISlot = IISlot;
    StateSlotMap.insert(std::make_pair(MBB, SS));
  }

  /// Information for allocated function units.

  void rememberAllocatedFU(FuncUnitId Id, unsigned EmitSlot, unsigned FinshSlot) {
    // Sometimes there are several instructions allocated to the same instruction,
    // and it is ok to try to insert the same FUId more than once.
    AllocatedFUs[Id.getFUType()].insert(Id);
    for (unsigned i = EmitSlot; i < FinshSlot; ++i)
      remeberActiveSlot(Id, i);
  }

  typedef std::set<FuncUnitId>::const_iterator const_id_iterator;

  const_id_iterator id_begin(VFUs::FUTypes FUType = VFUs::AllFUType) const {
    assert(FUType != VFUs::AllFUType && "AllFUType not supported now!");
    assert(FUType < VFUs::NumFUs && "Bad FUType!");
    
    return AllocatedFUs[FUType].begin();
  }

  const_id_iterator id_end(VFUs::FUTypes FUType = VFUs::AllFUType) const {
    assert(FUType != VFUs::AllFUType && "AllFUType not supported now!");
    assert(FUType < VFUs::NumFUs && "Bad FUType!");

    return AllocatedFUs[FUType].end();
  }

  // FIXME: Consider pipelined loop.
  void remeberActiveSlot(FuncUnitId Id, unsigned Slot) {
    ActiveSlotSet.insert(FUActiveSlot(Id, Slot));
  }

  bool isFUActiveAt(FuncUnitId Id, unsigned Slot) {
    return ActiveSlotSet.count(FUActiveSlot(Id, Slot));
  }
  
  const char *allocateSymbol(const std::string &Str) {
    PooledStringPtr PSP = SymbolPool.intern(Str.c_str());
    Symbols.insert(PSP);
    return *PSP;
  }
};

}

#endif
