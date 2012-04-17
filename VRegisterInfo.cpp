//===---------- VRegisterInfo.cpp - VTM Register Information ------*- C++ -*-===//
//
// Copyright: 2011 by SYSU EDA Group. all rights reserved.
// IMPORTANT: This software is supplied to you by Hongbin Zheng in consideration
// of your agreement to the following terms, and your use, installation,
// modification or redistribution of this software constitutes acceptance
// of these terms.  If you do not agree with these terms, please do not use,
// install, modify or redistribute this software. You may not redistribute,
// install copy or modify this software without written permission from
// Hongbin Zheng.
//
//===----------------------------------------------------------------------===//
//
// This file contains the VTM implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#include "vtm/Passes.h"
#include "vtm/VerilogBackendMCTargetDesc.h"
#include "vtm/VRegisterInfo.h"
#include "vtm/VInstrInfo.h"

#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Target/TargetData.h"
#include "llvm/Target/TargetLowering.h"
#include "llvm/Type.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/ADT/STLExtras.h"

namespace llvm {
  extern const MCRegisterDesc VTMRegDesc[];
  extern const uint16_t VTMRegLists[];
}

using namespace llvm;

VRegisterInfo::VRegisterInfo() : VTMGenRegisterInfo(0) {
  // Dirty hack: Allocate enough physical register for the backend.
  InitMCRegisterInfo(VTMRegDesc, MaxPhyRegs, 0,
                     VTMMCRegisterClasses, 14,
                     VTMRegLists,
                     NULL, 0);
}

const uint16_t*
VRegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const {
  static const uint16_t CalleeSavedRegs[] = {0};
  return  CalleeSavedRegs;
}

BitVector
VRegisterInfo::getReservedRegs(const MachineFunction &MF) const {
  BitVector Reserved(getNumRegs());
  return Reserved;
}

void VRegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator II,
                                        int SPAdj,
                                        RegScavenger *RS /*= NULL*/ ) const {
}

void VRegisterInfo::emitPrologue(MachineFunction &MF) const {}

void VRegisterInfo::emitEpilogue(MachineFunction &MF,
                                 MachineBasicBlock &MBB) const {}

unsigned
VRegisterInfo::getFrameRegister(const MachineFunction &MF) const {
  llvm_unreachable("No return address register in VTM");
  return 0;
}

unsigned VRegisterInfo::getEHExceptionRegister() const {
  llvm_unreachable("What is the exception register");
  return 0;
}

unsigned VRegisterInfo::getEHHandlerRegister() const {
  llvm_unreachable("What is the exception handler register");
  return 0;
}

int VRegisterInfo::getDwarfRegNum(unsigned RegNum, bool isEH) const {
  llvm_unreachable("What is the dwarf register number");
  return -1;
}

int VRegisterInfo::getDwarfRegNumFull(unsigned RegNum, unsigned Flavour) const {
  llvm_unreachable("Unknown DWARF flavour");
  return -1;
}

bool VRegisterInfo::needsStackRealignment(const MachineFunction &) const {
  return false;
}


#define GET_REGINFO_TARGET_DESC
#include "VerilogBackendGenRegisterInfo.inc"

const TargetRegisterClass *
VRegisterInfo::getPointerRegClass(unsigned Kind) const {
  return VTM::DRRegisterClass;
}

bool VRegisterInfo::IsWire(unsigned RegNo, const MachineRegisterInfo *MRI) {
  if (!TargetRegisterInfo::isVirtualRegister(RegNo))
    return false;

  const TargetRegisterClass *RC = MRI->getRegClass(RegNo);
  return RC != VTM::DRRegisterClass;
}

unsigned VRegisterInfo::allocatePhyReg(unsigned RegClassID, unsigned Width) {
  unsigned RegNum = PhyRegs.size() + 1;
  PhyRegs.push_back(PhyRegInfo(RegClassID, RegNum, Width, 0));
  return RegNum;
}

unsigned VRegisterInfo::getSubRegOf(unsigned Parent, unsigned UB, unsigned LB) {
  PhyRegInfo Info = getPhyRegInfo(Parent);
  unsigned AliasSetId = Info.getAliasSetId();
  unsigned SubRegNum = PhyRegs.size() + 1;
  PhyRegInfo SubRegInfo = PhyRegInfo(Info.getRegClass(), AliasSetId, UB, LB);
  PhyRegs.push_back(SubRegInfo);
  // TODO: Check if the sub register exist?
  PhyRegAliasInfo[AliasSetId].insert(SubRegNum);
  return SubRegNum;
}

unsigned VRegisterInfo::allocateFN(unsigned FNClassID, unsigned Width /* = 0 */) {
  return allocatePhyReg(FNClassID, Width);
}

VRegisterInfo::PhyRegInfo VRegisterInfo::getPhyRegInfo(unsigned RegNum) const {
  return PhyRegs[RegNum - 1];
}

void VRegisterInfo::resetPhyRegAllocation() {
  PhyRegs.clear();
  PhyRegAliasInfo.clear();
}

unsigned VRegisterInfo::getSubReg(unsigned RegNo, unsigned Index) const {
  unsigned SubReg = RegNo + Index;
  assert(getPhyRegInfo(SubReg).getParentRegister() == RegNo
         && "Bad subreg index!");
  return SubReg;
}


const TargetRegisterClass *VRegisterInfo::getRepRegisterClass(unsigned OpCode){
  switch (OpCode) {
  default:                  return VTM::WireRegisterClass;
  case VTM::VOpAdd:         return VTM::RADDRegisterClass;
  case VTM::VOpSRA:         return VTM::RASRRegisterClass;
  case VTM::VOpSRL:         return VTM::RLSRRegisterClass;
  case VTM::VOpSHL:         return VTM::RSHLRegisterClass;
  case VTM::VOpMult:        return VTM::RMULRegisterClass;
  case VTM::VOpMultLoHi:    return VTM::RMULLHRegisterClass;
  case VTM::VOpMemTrans:    return VTM::RINFRegisterClass;
  case VTM::VOpInternalCall:return VTM::RCFNRegisterClass;
  case VTM::VOpBRAMTrans:   return VTM::RBRMRegisterClass;
    // allocate unsigned comparison fu by default.
  case VTM::VOpICmp:        return VTM::RUCMPRegisterClass;
  case VTM::VOpDstMux:      return VTM::RMUXRegisterClass;
  case VTM::VOpPipelineStage: return VTM::RPIPERegisterClass;
  }

  return 0;
}
