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

#include "vtm/SynSettings.h"
#include "vtm/FUInfo.h"
#include "vtm/VerilogAST.h"

#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/Support/StringPool.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/ADT/OwningPtr.h"

#include <set>
#include <map>

namespace llvm {
class MachineBasicBlock;
class VFInfo : public MachineFunctionInfo {
  // Information about slots.
  struct StateSlots{
    unsigned startSlot : 32;
    unsigned totalSlot : 16;
    unsigned IISlot    : 16;
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

  // Remember the scheduled slot of PHI nodes, it will lose after PHIElemination.
  typedef std::map<std::pair<unsigned, unsigned>, unsigned> CopySlotMapTy;
  CopySlotMapTy CopySlots;

  // Allocated physics registers in a MachineFunction/RTL module.
  // TODO: we need to perform per-BasicBlock register allocation to reduce
  // the length of interconnection.
  unsigned TotalRegs;
  static const unsigned fistPhyReg = 8;

public:
  // The data structure to describe the block ram.
  struct BRamInfo {
    unsigned NumElem, ElemSizeInBytes;

    BRamInfo(unsigned numElem, unsigned elemSizeInBytes)
      : NumElem(numElem), ElemSizeInBytes(elemSizeInBytes) {}
  };

private:
  typedef std::map<uint16_t, BRamInfo> BRamMapTy;
  BRamMapTy BRams;

  StringPool SymbolPool;
  std::set<PooledStringPtr> Symbols;
  const SynSettings *Info;
  // Rtl module.
  VASTModule Mod;
  // If bit width information annotated to the annotator?
  bool BitWidthAnnotated;
public:
  explicit VFInfo(MachineFunction &MF)
    : TotalRegs(fistPhyReg),
      Info(getSynSetting(MF.getFunction()->getName())),
      Mod(Info->getModName()), BitWidthAnnotated(true)
  {}

  bool isBitWidthAnnotated() const { return BitWidthAnnotated; }
  void removeBitWidthAnnotators() {
    assert(isBitWidthAnnotated() && "Annotators arealy removed!");
    BitWidthAnnotated = false;
  }

  const SynSettings &getInfo() const { return *Info; }

  /// Verilog module for the machine function.
  VASTModule *getRtlMod() const { return const_cast<VASTModule*>(&Mod); }

  /// Slots information for machine basicblock.
  unsigned getStartSlotFor(const MachineBasicBlock* MBB) const;
  unsigned getTotalSlotFor(const MachineBasicBlock *MBB) const;
  unsigned getEndSlotFor(const MachineBasicBlock *MBB) const {
    return getStartSlotFor(MBB) + getTotalSlotFor(MBB);
  }
  unsigned getIISlotFor(const MachineBasicBlock* MBB) const;
  unsigned getIIFor(const MachineBasicBlock *MBB) const {
    return getIISlotFor(MBB) - getStartSlotFor(MBB);
  }

  void rememberTotalSlot(const MachineBasicBlock* MBB,
                        unsigned startSlot,
                        unsigned totalSlot,
                        unsigned IISlot);

  void rememberCopySlot(unsigned SrcReg, unsigned DstReg, unsigned Slot) {
    std::pair<unsigned, unsigned> Key(SrcReg, DstReg);
    assert(!CopySlots.count(Key) && "CopySlot already exist!");
    CopySlots.insert(std::make_pair(Key, Slot));
  }

  unsigned lookupCopySlot(unsigned SrcReg, unsigned DstReg) const {
    CopySlotMapTy::const_iterator At =
      CopySlots.find(std::make_pair(SrcReg, DstReg));
    
    if (At == CopySlots.end())
      return 0;
    
    return At->second;
  }
  /// Information for allocated function units.

  void rememberAllocatedFU(FuncUnitId Id, unsigned EmitSlot, unsigned FinshSlot);

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

  // Get the number of used function units in the current MachineFunction.
  bool getNumFUs(VFUs::FUTypes FUType = VFUs::AllFUType) const {
    assert(FUType != VFUs::AllFUType && "AllFUType not supported now!");
    assert(FUType < VFUs::NumFUs && "Bad FUType!");

    return AllocatedFUs[FUType].size();
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

  // Block Ram management.
  void allocateBRam(uint16_t ID, unsigned NumElem, unsigned ElemSizeInBytes);

  const BRamInfo &getBRamInfo(uint16_t ID) const {
    BRamMapTy::const_iterator at = BRams.find(ID);
    assert(at != BRams.end() && "BRam not exists!");
    return at->second;
  }

  // Allocate a Physics register, its sizeInBytes can be 1/2/3/4
  unsigned allocatePhyReg(unsigned SizeInBytes) {
    unsigned ret = RoundUpToAlignment(TotalRegs, SizeInBytes);
    // The register should always align.
    TotalRegs = ret + SizeInBytes;
    return ret;
  }

  class phyreg_iterator : public std::iterator<std::forward_iterator_tag,
                                               unsigned> {
    unsigned i, sizeInBytes;
  public:
    phyreg_iterator(unsigned I, unsigned SizeInBytes)
      : i(I), sizeInBytes(SizeInBytes) {}

    inline bool operator==(const phyreg_iterator RHS) const {
      assert(sizeInBytes == RHS.sizeInBytes
             && "Can not compare phyreg_iterator with different sizeInBytes!");
      return i == RHS.i;
    }

    inline bool operator!=(const phyreg_iterator RHS) const {
      return !operator==(RHS);
    }

    inline bool operator<(const phyreg_iterator RHS) const {
      assert(sizeInBytes == RHS.sizeInBytes
        && "Can not compare phyreg_iterator with different sizeInBytes!");
      return i < RHS.i;
    }

    inline unsigned operator*() const { return i; }

    inline phyreg_iterator &operator++() {
      i += sizeInBytes;
      return *this;
    }

    inline phyreg_iterator operator++(int) {
      phyreg_iterator tmp = *this;
      ++*this;
      return tmp;
    }
  };

  phyreg_iterator phyreg_begin(unsigned sizeInByte) const {
    return phyreg_iterator(fistPhyReg,  sizeInByte);
  }

  phyreg_iterator phyreg_end(unsigned  sizeInByte) const {
    return phyreg_iterator(TotalRegs,  sizeInByte);
  }

  unsigned getOverlaps(unsigned R, unsigned Overlaps[5]) const;

  // Out of line virtual function to provide home for the class.
  virtual void anchor();
};

}

#endif
