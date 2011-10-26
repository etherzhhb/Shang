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

#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/Support/StringPool.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/ADT/OwningPtr.h"

#include <set>
#include <map>

namespace llvm {
class MachineBasicBlock;
class MachineInstr;
class VASTModule;

class VFInfo : public MachineFunctionInfo {
  // Information about slots.
  struct StateSlots{
    unsigned startSlot : 32;
    unsigned totalSlot : 16;
    unsigned IISlot    : 16;
  };
  std::map<const MachineBasicBlock*, StateSlots> StateSlotMap;

  // Remember the scheduled slot of PHI nodes, it will lose after PHIElemination.
  typedef std::map<const MachineInstr*,
                   std::pair<int, const MachineBasicBlock*> >
          PhiSlotMapTy;
  PhiSlotMapTy PHISlots;
public:
  // The data structure to describe the block ram.
  struct BRamInfo {
    const Value* Initializer;
    unsigned NumElem, ElemSizeInBytes;
    unsigned PhyRegNum;

    BRamInfo(const Value* Initializer, unsigned numElem, unsigned elemSizeInBytes)
      : Initializer(Initializer), NumElem(numElem), ElemSizeInBytes(elemSizeInBytes), PhyRegNum(0) {}
  };

  typedef std::map<uint16_t, BRamInfo> BRamMapTy;
  typedef BRamMapTy::const_iterator const_bram_iterator;

  struct PhyRegInfo {
    union {
      struct {
        unsigned RegClassId : 4;
        unsigned AliasSetId : 14;
        unsigned UB         : 7;
        unsigned LB         : 7;
      } SData;
      unsigned IData;
    } UData;

    PhyRegInfo(unsigned ClassId, unsigned AliasSet, unsigned U, unsigned L) {
      UData.SData.RegClassId = ClassId;
      UData.SData.AliasSetId = AliasSet;
      UData.SData.UB = U;
      UData.SData.LB = L;
    }

    /*implicit*/ PhyRegInfo(unsigned D) { UData.IData = D; }
    operator unsigned() const {return UData.IData; }

    unsigned getRegClass() const { return UData.SData.RegClassId; }
    unsigned getUB() const { return UData.SData.UB; }
    unsigned getLB() const { return UData.SData.LB; }
    unsigned getBitWidth() const { return getUB() - getLB(); }
    unsigned getAliasSetId() const { return UData.SData.AliasSetId; }
    unsigned getParentRegister() const { return UData.SData.AliasSetId; }
    bool isTopLevelReg(unsigned RegNum) const {
      return RegNum == getParentRegister();
    }
  };

private:
  typedef std::vector<unsigned> PhyRegVec;
  typedef std::set<unsigned> PhyRegSet;
  typedef std::map<unsigned, PhyRegSet> PhyRegAlaisMap;
  PhyRegVec PhyRegs;
  PhyRegAlaisMap PhyRegAliasInfo;

  BRamMapTy BRams;

  // Mapping Function unit number to callee function name.
  typedef StringMap<unsigned> FNMapTy;
  typedef StringMapEntry<unsigned> FNEntryTy;
  FNMapTy UsedFNs;
  const SynSettings *Info;
  // Rtl module.
  VASTModule *Mod;
  // If bit width information annotated to the annotator?
  bool BitWidthAnnotated;
public:
  explicit VFInfo(MachineFunction &MF);
  ~VFInfo();

  bool isBitWidthAnnotated() const { return BitWidthAnnotated; }
  void removeBitWidthAnnotators() {
    assert(isBitWidthAnnotated() && "Annotators arealy removed!");
    BitWidthAnnotated = false;
  }

  const SynSettings &getInfo() const { return *Info; }

  void setTotalSlots(unsigned Slots);

  /// Verilog module for the machine function.
  VASTModule *getRtlMod() const;

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

  void rememberPHISlot(const MachineInstr *PN, unsigned Slot,
                       bool Pipe = false) {
    int S = Pipe ? - Slot : Slot;
    bool success =
      PHISlots.insert(std::make_pair(PN,
                                     std::make_pair(S, PN->getParent()))).second;
    assert(success && "Insert the same phinode twice?");
    (void) success;
  }


  typedef FNMapTy::const_iterator const_fn_iterator;
  const_fn_iterator fn_begin() const { return UsedFNs.begin(); }
  const_fn_iterator fn_end() const { return UsedFNs.end(); }

  unsigned getOrCreateCalleeFN(StringRef FNName) {
    FNMapTy::iterator at = UsedFNs.find(FNName);
    if (at != UsedFNs.end()) return at->second;

    unsigned CalleeFNNum = UsedFNs.size() + 1;
    FNEntryTy *FN = FNEntryTy::Create(FNName.begin(), FNName.end());
    FN->second = CalleeFNNum;
    UsedFNs.insert(FN);
    return CalleeFNNum;
  }

  unsigned getCalleeFNNum(StringRef FNName) const {
    return UsedFNs.lookup(FNName);
  }

  //const Function *getCalleeFN(unsigned FNNum) const {
  //  assert(FNNum < UsedFNs.size() && "Invalid FNNum!");
  //  return UsedFNs[FNNum];
  //}

  std::pair<int, const MachineBasicBlock*>
    lookupPHISlot(const MachineInstr *PN) const;

  //const char *allocateSymbol(const std::string &Str) {
  //  PooledStringPtr PSP = SymbolPool.intern(Str.c_str());
  //  Symbols.insert(PSP);
  //  return *PSP;
  //}

  // Block Ram management.
  void allocateBRam(uint16_t ID, unsigned NumElem, unsigned ElemSizeInBytes, 
                                           const Value* Initializer = 0);

  const BRamInfo &getBRamInfo(uint16_t ID) const {
    BRamMapTy::const_iterator at = BRams.find(ID);
    assert(at != BRams.end() && "BRam not exists!");
    return at->second;
  }

  BRamInfo &getBRamInfo(uint16_t ID) {
    BRamMapTy::iterator at = BRams.find(ID);
    assert(at != BRams.end() && "BRam not exists!");
    return at->second;
  }

  const_bram_iterator bram_begin() const { return BRams.begin(); }
  const_bram_iterator bram_end() const { return BRams.end(); }

  // Physics register allocate information.
  unsigned allocatePhyReg(unsigned RegClassID, unsigned Width);
  unsigned getSubRegOf(unsigned Parent, unsigned UB, unsigned LB);
  unsigned allocateFN(unsigned FNClassID, unsigned Width = 0);

  PhyRegInfo getPhyRegInfo(unsigned RegNum) const;

  unsigned num_phyreg() const { return PhyRegs.size(); }

};

}

#endif
