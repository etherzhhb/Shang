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
#include "vtm/VerilogAST.h"

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

std::pair<int, const MachineBasicBlock*>
VFInfo::lookupPHISlot(const MachineInstr *PN) const {
  PhiSlotMapTy::const_iterator At = PHISlots.find(PN);

  if (At == PHISlots.end())
    return std::make_pair(0, (const MachineBasicBlock*)0);

  return At->second;
}

void VFInfo::allocateBRam(uint16_t ID, unsigned NumElem,
                          unsigned ElemSizeInBytes, const Value* Initializer) {
  bool Inserted;
  BRamMapTy::iterator at;

  tie(at, Inserted)
    = BRams.insert(std::make_pair(ID, BRamInfo(Initializer, NumElem, ElemSizeInBytes)));

  assert(Inserted && "BRam already existed!");
}

VFInfo::VFInfo(MachineFunction &MF)
  : Info(getSynSetting(MF.getFunction()->getName())),
    Mod(new VASTModule(Info->getModName())),
    BitWidthAnnotated(true) {}

VFInfo::~VFInfo() { delete Mod; }

void VFInfo::setTotalSlots(unsigned Slots) {
  Mod->allocaSlots(Slots);
}

VASTModule *VFInfo::getRtlMod() const {
  return Mod;
}
