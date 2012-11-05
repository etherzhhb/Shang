//===-- VFInfo.cpp - VTM Per-Function Information Class Implementation ----===//
//
//                      The Shang HLS frameowrk                               //
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
  assert(IISlot <= startSlot + totalSlot && "Bad pipelined schedule!");
  SS.startSlot = startSlot;
  SS.totalSlot = totalSlot;
  SS.IISlot = IISlot;
  StateSlotMap.insert(std::make_pair(MBB, SS));
}

void VFInfo::allocateBRAM(uint16_t ID, unsigned NumElem,
                          unsigned ElemSizeInBytes, const Value* Initializer) {
  bool Inserted = BRams.insert(std::make_pair(ID, BRamInfo(NumElem,
                                                           ElemSizeInBytes,
                                                           Initializer))).second;
  assert(Inserted && "BRAM already existed!");
}

VFInfo::VFInfo(MachineFunction &MF)
  : TotalSlot(0), Info(getSynSetting(MF.getFunction()->getName())),
    BitWidthAnnotated(true) {}

VFInfo::~VFInfo() { }

void VFInfo::setTotalSlots(unsigned Slots) {
  TotalSlot = Slots;
}

void VFInfo::remapCallee(StringRef FNName, unsigned NewFNNum) {
  FNMapTy::iterator at = UsedFNs.find(FNName);
  assert(at != UsedFNs.end() && "Callee not exist!");
  assert(at->second != NewFNNum && "No need to remap FNNum!");
  at->second = NewFNNum;
}
