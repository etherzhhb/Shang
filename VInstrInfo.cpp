//===---------- VInstrInfo.cpp - VTM Instruction Information -----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the VTM implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "VTargetMachine.h"

#include "vtm/VFInfo.h"
#include "vtm/VInstrInfo.h"
#include "vtm/VTM.h"
#include "vtm/MicroState.h"

#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/Support/ErrorHandling.h"

#include "VGenInstrInfo.inc"

using namespace llvm;

VInstrInfo::VInstrInfo(const TargetData &TD, const TargetLowering &TLI)
  : TargetInstrInfoImpl(VTMInsts, array_lengthof(VTMInsts)), RI(*this, TD, TLI)
  {}


bool VInstrInfo::isReallyTriviallyReMaterializable(const MachineInstr *MI,
                                                   AliasAnalysis *AA) const {
  VIDesc Desc(MI->getDesc());
  return !Desc->isBarrier() && Desc.hasTrivialFU();
}

bool VInstrInfo::AnalyzeBranch(MachineBasicBlock &MBB, MachineBasicBlock *&TBB,
                               MachineBasicBlock *&FBB,
                               SmallVectorImpl<MachineOperand> &Cond,
                               bool AllowModify /* = false */) const {
  // TODO: Write code for this function.
  return true;
}

unsigned VInstrInfo::createPHIIncomingReg(unsigned DestReg,
                                             MachineRegisterInfo *MRI) const {
  const TargetRegisterClass *PHIRC = VTM::PHIRRegisterClass;
  return MRI->createVirtualRegister(PHIRC);
}

typedef MachineBasicBlock::iterator mbb_it;

MachineInstr *VInstrInfo::insertPHIImpDef(MachineBasicBlock &MBB,
                                          mbb_it InsertPos,
                                          MachineInstr *PN) const {
  return TargetInstrInfo::insertPHIImpDef(MBB, InsertPos, PN);
}

MachineInstr *VInstrInfo::insertPHIIcomingCopy(MachineBasicBlock &MBB,
                                               mbb_it InsertPos,
                                               MachineInstr *PN,
                                               unsigned IncomingReg) const {
  ucOperand &DefOp = cast<ucOperand>(PN->getOperand(0));
  ucState Ctrl(InsertPos);
  assert(Ctrl->getOpcode() == VTM::Control && "Unexpected instruction type!");
  // Simply build the copy in the first control slot.
  MachineInstrBuilder Builder(InsertPos);
  Builder.addOperand(ucOperand::CreateOpcode(VTM::VOpDefPhi, Ctrl.getSlot()));
  Builder.addOperand(ucOperand::CreatePredicate());
  ucOperand Dst = MachineOperand::CreateReg(DefOp.getReg(), true);
  Dst.setBitWidth(DefOp.getBitWidth());
  Builder.addOperand(Dst);
  ucOperand Src = MachineOperand::CreateReg(IncomingReg, false);
  Src.setBitWidth(DefOp.getBitWidth());
  Builder.addOperand(Src);
  return &*Builder;
}

MachineInstr *VInstrInfo::insertPHICopySrc(MachineBasicBlock &MBB,
                                           mbb_it InsertPos, MachineInstr *PN,
                                           unsigned IncomingReg,
                                           unsigned SrcReg, unsigned SrcSubReg)
                                           const {
  ucOperand &DefOp = cast<ucOperand>(PN->getOperand(0));
  // Get the last slot.
  InsertPos = prior(InsertPos);

  VFInfo *VFI = MBB.getParent()->getInfo<VFInfo>();
  unsigned Slot = VFI->lookupPHISlot(PN);
  unsigned StartSlot = VFI->getStartSlotFor(&MBB);
  // If the phi scheduled into this MBB, insert the copy to the right control
  // slot.
  if (Slot > StartSlot && Slot <= VFI->getEndSlotFor(&MBB)) {
    unsigned II = VFI->getIIFor(&MBB);
    unsigned ModuloSlot = (Slot - StartSlot) % II + StartSlot;
    // If modulo slot is 0, insert the copy in the last control slot.
    // Otherwise, iterate over the BB to find the match slot.
    if (ModuloSlot != StartSlot) {
      while(ucState(InsertPos).getSlot() != ModuloSlot) {
        --InsertPos; // Skip the current control slot.
        --InsertPos; // Skip the current datapath slot.
      }
    }
  }

  ucState Ctrl(InsertPos);
  assert(Ctrl->getOpcode() == VTM::Control && "Unexpected instruction type!");

  MachineInstrBuilder Builder(InsertPos);
  Builder.addOperand(ucOperand::CreateOpcode(VTM::VOpMvPhi, Slot));
  Builder.addOperand(ucOperand::CreatePredicate());
  ucOperand Dst = MachineOperand::CreateReg(IncomingReg, true);
  Dst.setBitWidth(DefOp.getBitWidth());
  Builder.addOperand(Dst);

  MachineRegisterInfo &MRI = MBB.getParent()->getRegInfo();
  MachineRegisterInfo::def_iterator DI = MRI.def_begin(SrcReg);
  assert (++MRI.def_begin(SrcReg) == MRI.def_end() && "Not in SSA From!");
  ucOp WriteOp = ucOp::getParent(DI);
  // We need to forward the wire copy if the source register is written
  // in the same slot, otherwise we will read a out of date value.
  if (WriteOp->getParent() == &*Ctrl) {
    if (WriteOp->getOpcode() == VTM::COPY)
      Builder.addOperand(MachineOperand(WriteOp.getOperand(1)));
    // FIXME: Handle others case.
  }

  ucOperand Src = MachineOperand::CreateReg(SrcReg, false);
  Src.setSubReg(SrcSubReg);
  Src.setBitWidth(DefOp.getBitWidth());
  Builder.addOperand(Src);
  return &*Builder;
}

FuncUnitId VIDesc::getPrebindFUId()  const {
  // Dirty Hack: Bind all memory access to channel 0 at this moment.
  if (getTID().Opcode == VTM::VOpMemTrans)
    return FuncUnitId(VFUs::MemoryBus, 0);

  if (getTID().Opcode == VTM::VOpBRam) {
    unsigned Id = get().getOperand(5).getImm();
    return FuncUnitId(VFUs::BRam, Id);
  }

  return FuncUnitId();
}


BitWidthAnnotator::BitWidthAnnotator(MachineInstr &MI)
  : MO(&MI.getOperand(MI.getNumOperands() - 1)) {
  assert(hasBitWidthInfo() && "Bitwidth not available!");
  BitWidths = MO->getImm();
}

void BitWidthAnnotator::updateBitWidth() {
  assert(MO && hasBitWidthInfo() && "Cannot update bit width!");
  MO->setImm(BitWidths);
}

bool BitWidthAnnotator::hasBitWidthInfo() const {
  assert(MO && "MachineOperand not available!");
  return MO->isImm();
}

void BitWidthAnnotator::changeToDefaultPred() {
  MO->ChangeToRegister(0, false);
}

bool VIDesc::mayLoad() const {
  switch (getTID().Opcode) {
  default: return false;
  // There is a "isLoad" flag in memory access operation.
  case VTM::VOpMemTrans: return !get().getOperand(3).getImm();
  }
}

bool VIDesc::mayStore() const {
  switch (getTID().Opcode) {
  default: return false;
    // There is a "isLoad" flag in memory access operation.
  case VTM::VOpMemTrans: return get().getOperand(3).getImm();
  }
}

bool VIDesc::canCopyBeFused() const {
  const MachineInstr &I = get();
  assert(I.isCopy() && "canCopyBeFused called on the wrong instruction!");
  if (I.getOperand(1).isImm()) return true;

  assert(I.getParent() && "Expected instruction embedded in machine function!");
  const MachineRegisterInfo &MRI = I.getParent()->getParent()->getRegInfo();
  unsigned DstReg = I.getOperand(0).getReg(),
           SrcReg = I.getOperand(1).getReg();

  // Later pass can not eliminate the non-trivial copy, so it should be fused.
  return MRI.getRegClass(DstReg) != MRI.getRegClass(SrcReg);
}

// Out of line virtual function to provide home for the class.
void VIDesc::anchor() {}
