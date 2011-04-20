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

#include "vtm/Passes.h"
#include "vtm/VTM.h"
#include "vtm/VRegisterInfo.h"

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
using namespace llvm;

const unsigned*
VRegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const {
  static const unsigned CalleeSavedRegs[] = {0};
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

int VRegisterInfo::getDwarfRegNumFull(unsigned RegNum, unsigned Flavour) const {
  assert(0 && "Unknown DWARF flavour");
  return -1;
}

bool VRegisterInfo::needsStackRealignment(const MachineFunction &) const {
  return false;
}


#include "VGenRegisterInfo.inc"

VRegisterInfo::VRegisterInfo(const TargetInstrInfo &tii, const TargetData &td,
                             const TargetLowering &tli)
  : TargetRegisterInfo(RegisterDescriptors, 4096, 
                       RegisterClasses, RegisterClasses+1,
                       SubRegIndexTable,
                       -1, -1,
                       SubregHashTable, SubregHashTableSize,
                       AliasesHashTable, AliasesHashTableSize),
    TII(tii), TD(td), TLI(tli) {}

const TargetRegisterClass *
VRegisterInfo::getPointerRegClass(unsigned Kind) const {
  MVT PtrVT = MVT::getIntegerVT(TD.getPointerSizeInBits());
  return TLI.getRegClassFor(PtrVT);
}
