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

VRegisterInfo::VRegisterInfo(const TargetInstrInfo &tii, const TargetData &td,
                             const TargetLowering &tli)
  : VTMGenRegisterInfo(), TII(tii), TD(td), TLI(tli),
    MRI(0) {
  // Dirty Hack.
  NumRegs = 0;
}

const TargetRegisterDesc &VRegisterInfo::operator[](unsigned RegNo) const {
  assert(RegNo < NumRegs &&
    "Attempting to access record for invalid register number!");
  if (VTM::DR1RegClass.count(RegNo))
    return Desc[VTM::D1];
  
  if (VTM::DR8RegClass.count(RegNo))
    return Desc[VTM::D8];

  if (VTM::DR16RegClass.count(RegNo))
    return Desc[VTM::D16];

  if (VTM::DR32RegClass.count(RegNo))
    return Desc[VTM::D32];

  if (VTM::DR64RegClass.count(RegNo))
    return Desc[VTM::D64];

  assert(0 && "Bad register!");
  return Desc[RegNo];
}

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

bool VRegisterInfo::hasFP(const MachineFunction &MF) const {
  return false;
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

#include "VGenRegisterInfo.inc"

const TargetRegisterClass *
VRegisterInfo::getPointerRegClass(unsigned Kind) const {
  MVT PtrVT = MVT::getIntegerVT(TD.getPointerSizeInBits());
  return TLI.getRegClassFor(PtrVT);
}

void llvm::VRegisterInfo::resetPhyRegs() {
  VTM::DR1RegClass.clear();
  VTM::DR8RegClass.clear();
  VTM::DR16RegClass.clear();
  VTM::DR32RegClass.clear();
  VTM::DR64RegClass.clear();
  NumRegs = 0;
}

bool VRegisterInfo::createPhyRegs(MachineRegisterInfo &mri) {
  MRI = &mri;
  
  resetPhyRegs();

  createPhyRegs(VTM::DR1RegClass);
  createPhyRegs(VTM::DR8RegClass);
  createPhyRegs(VTM::DR16RegClass);
  createPhyRegs(VTM::DR32RegClass);
  createPhyRegs(VTM::DR64RegClass);
  // The last register number is NumRegs, so we have NumRegs + 1 registers.
  ++NumRegs;
  // Notice the MachineRegisterInfo after physics registers changed.
  mri.updatePhyRegsInfo();
  return false;
}

// We should not need to publish the initializer as long as no other passes
// require RAOptimalSSA.
#if 0 // disable INITIALIZE_PASS
INITIALIZE_PASS(DynCreatePhyRegs, "dynamic-create-phyregs",
                "Create the Physics Registers on demand.", false, true);
#endif // disable INITIALIZE_PASS


namespace {
struct DynPhyRegsBuilder : public MachineFunctionPass {
  static char ID;
  VRegisterInfo &VRI;
  DynPhyRegsBuilder(VRegisterInfo &vri) : MachineFunctionPass(ID), VRI(vri) {}

  void getAnalysisUsage(AnalysisUsage &AU) const {
    MachineFunctionPass::getAnalysisUsage(AU);
    AU.setPreservesAll();
  }

  bool runOnMachineFunction(MachineFunction &MF) {
    VRI.createPhyRegs(MF.getRegInfo());
    return false;
  }
};
}

char DynPhyRegsBuilder::ID = 0;

FunctionPass *llvm::createDynPhyRegsBuilderPass(VRegisterInfo &VRI) {
  return new DynPhyRegsBuilder(VRI);
}
