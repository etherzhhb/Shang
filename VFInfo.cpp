//===-- VFInfo.cpp - VTM Per-Function Information Class Implementation ----===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the interfaces that VTM uses to lower LLVM code
// into a selection DAG.
//
//===----------------------------------------------------------------------===//
#include "vtm/VFInfo.h"
#include "vtm/VInstrInfo.h"

#include "llvm/ADT/STLExtras.h"
#include "llvm/Support/MathExtras.h"

using namespace llvm;

unsigned VFInfo::getTotalSlotFor(const MachineBasicBlock* MBB) const {
  std::map<const MachineBasicBlock*, StateSlots>::const_iterator
    at = StateSlotMap.find(MBB);

  assert(at != StateSlotMap.end() && "State not found!");
  return at->second.totalSlot;
}

unsigned VFInfo::getStartSlotFor(const MachineBasicBlock* MBB) const {
  std::map<const MachineBasicBlock*, StateSlots>::const_iterator
    at = StateSlotMap.find(MBB);

  assert(at != StateSlotMap.end() && "State not found!");
  return at->second.startSlot;
}

unsigned VFInfo::getIISlotFor(const MachineBasicBlock* MBB) const {
  std::map<const MachineBasicBlock*, StateSlots>::const_iterator
    at = StateSlotMap.find(MBB);

  assert(at != StateSlotMap.end() && "State not found!");
  return at->second.IISlot;
}

void VFInfo::rememberTotalSlot(const MachineBasicBlock* MBB, unsigned startSlot,
                              unsigned totalSlot, unsigned IISlot) {
  StateSlots SS;
  SS.startSlot = startSlot;
  SS.totalSlot = totalSlot;
  SS.IISlot = IISlot;
  StateSlotMap.insert(std::make_pair(MBB, SS));
}

unsigned VFInfo::lookupPHISlot(const MachineInstr *PN) const {
  PhiSlotMapTy::const_iterator At = PHISlots.find(PN);

  if (At == PHISlots.end()) return 0;

  return At->second;
}

void VFInfo::rememberAllocatedFU(FuncUnitId Id, unsigned EmitSlot,
                                 unsigned FinshSlot, MachineOperand Pred) {
  Pred.clearParent();
  // Sometimes there are several to allocated to the same instruction,
  // and it is ok to try to insert the same FUId more than once.
  AllocatedFUs[Id.getFUType() & 0xf].insert(Id);
  for (unsigned i = EmitSlot; i < FinshSlot; ++i)
    remeberActiveSlot(Id, i, Pred);
}

void VFInfo::allocateBRam(uint16_t ID, unsigned NumElem,
                          unsigned ElemSizeInBytes) {
  bool Inserted;
  BRamMapTy::iterator at;

  tie(at, Inserted)
    = BRams.insert(std::make_pair(ID, BRamInfo(NumElem, ElemSizeInBytes)));

  assert(Inserted && "BRam already existed!");
}

// Out of line virtual function to provide home for the class.
void VFInfo::anchor() {}

unsigned VFInfo::getOverlaps(unsigned R, unsigned Overlaps[5]) const {
  unsigned Idx = 0;
  unsigned TrailingZeros = CountTrailingZeros_32(R);
  for (unsigned i = TrailingZeros + 1; i < 4; ++i)
    Overlaps[Idx++] = R & (~0 << i);

  // We have at most 8 bytes register.
  unsigned NumSubRegs = std::min(1 << TrailingZeros, 8);
  for (unsigned i = 0; i < NumSubRegs; ++i)
    Overlaps[Idx++] = R + i;

  //Overlaps[Idx] = R & (~0 << 3);
  //if (Overlaps[0] != R) Overlaps[++Idx] = R;

  return Idx;
}

VFInfo::VFInfo(MachineFunction &MF) : TotalRegs(fistPhyReg),
  Info(getSynSetting(MF.getFunction()->getName())),
  Mod(Info->getModName()), BitWidthAnnotated(true) {
  // DirtyHack: Every Module use Memory bus 0.
  rememberAllocatedFU(FuncUnitId(VFUs::MemoryBus, 0), 0, 0,
                      MachineOperand::CreateImm(0));
}
