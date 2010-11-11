//===---------- VRegisterInfo.cpp - VTM Register Information ------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the VTM implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#include "VTM.h"
#include "VRegisterInfo.h"

#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineLocation.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/Target/TargetData.h"
#include "llvm/Target/TargetFrameInfo.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetOptions.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/Target/TargetLowering.h"
#include "llvm/Type.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/ADT/STLExtras.h"
using namespace llvm;

VRegisterInfo::VRegisterInfo(const TargetInstrInfo &tii, const TargetData &td,
                             const TargetLowering &tli)
  : VTMGenRegisterInfo(), TII(tii), TD(td), TLI(tli) {}

const unsigned*
VRegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const {
  static const unsigned CalleeSavedRegs[] = {0};
  return  CalleeSavedRegs;
}

BitVector
VRegisterInfo::getReservedRegs(const MachineFunction &MF) const {
  BitVector Reserved;
  return Reserved;
}

bool VRegisterInfo::hasFP(const MachineFunction &MF) const {
  return false;
}

void VRegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator II,
                                        int SPAdj,
                                        RegScavenger *RS /*= NULL*/ ) const {
}

// Emit a prologue that sets up a stack frame.
// On function entry, R0-R2 and P0 may hold arguments.
// R3, P1, and P2 may be used as scratch registers
void VRegisterInfo::emitPrologue(MachineFunction &MF) const {}

void VRegisterInfo::emitEpilogue(MachineFunction &MF,
                                 MachineBasicBlock &MBB) const {}

unsigned VRegisterInfo::getRARegister() const {
  llvm_unreachable("No return address register in VTM");
  return 0;
}

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

#include "VGenRegisterInfo.inc"

const TargetRegisterClass *
VRegisterInfo::getPointerRegClass(unsigned Kind) const {
  MVT PtrVT = MVT::getIntegerVT(TD.getPointerSizeInBits());
  return TLI.getRegClassFor(PtrVT);
}
