//===------ VRegisterInfo.h - VTM Register Information -----------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the VTM implementation of the TargetRegisterInfo
// class.
//
//===----------------------------------------------------------------------===//

#ifndef VINREGISTERINFO_H
#define VINREGISTERINFO_H

#include "llvm/Target/TargetRegisterInfo.h"
#include "VGenRegisterInfo.h.inc"

namespace llvm {

class TargetInstrInfo;
class Type;

struct VRegisterInfo : public VTMGenRegisterInfo {
  const TargetInstrInfo &TII;

  VRegisterInfo(const TargetInstrInfo &tii);

  /// Code Generation virtual methods...
  const unsigned *getCalleeSavedRegs(const MachineFunction *MF = 0) const;

  BitVector getReservedRegs(const MachineFunction &MF) const;

  // getSubReg implemented by tablegen

  const TargetRegisterClass *getPointerRegClass(unsigned Kind = 0) const {
    return &VTM::PR64RegClass;
  }

  bool hasFP(const MachineFunction &MF) const;
  
  void eliminateFrameIndex(MachineBasicBlock::iterator II,
    int SPAdj, RegScavenger *RS = NULL) const;

  void emitPrologue(MachineFunction &MF) const;
  void emitEpilogue(MachineFunction &MF, MachineBasicBlock &MBB) const;

  unsigned getFrameRegister(const MachineFunction &MF) const;
  unsigned getRARegister() const;

  // Exception handling queries.
  unsigned getEHExceptionRegister() const;
  unsigned getEHHandlerRegister() const;

  int getDwarfRegNum(unsigned RegNum, bool isEH) const;

  void loadConstant(MachineBasicBlock &MBB,
                    MachineBasicBlock::iterator I,
                    DebugLoc DL,
                    unsigned Reg,
                    int value) const;
};

} // end namespace llvm

#endif
