//===-- VerilogBackendMCTargetDesc.cpp - Cell VerilogBackend Target Descriptions -----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides Cell VerilogBackend specific target descriptions.
//
//===----------------------------------------------------------------------===//

#include "vtm/VerilgoBackendMCTargetDesc.h"
//#include "VerilogBackendMCAsmInfo.h"
#include "llvm/MC/MachineLocation.h"
#include "llvm/MC/MCCodeGenInfo.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/TargetRegistry.h"

#define GET_INSTRINFO_MC_DESC
#include "VerilogBackendGenInstrInfo.inc"

#define GET_SUBTARGETINFO_MC_DESC
#include "VerilogBackendGenSubtargetInfo.inc"

#define GET_REGINFO_MC_DESC
#include "VerilogBackendGenRegisterInfo.inc"

using namespace llvm;

static MCInstrInfo *createVerilogBackendMCInstrInfo() {
  MCInstrInfo *X = new MCInstrInfo();
  InitVerilogBackendMCInstrInfo(X);
  return X;
}

static MCRegisterInfo *createVerilogBackendMCRegisterInfo(StringRef TT) {
  MCRegisterInfo *X = new MCRegisterInfo();
  InitVerilogBackendMCRegisterInfo(X, VerilogBackend::R0);
  return X;
}

static MCSubtargetInfo *createVerilogBackendMCSubtargetInfo(StringRef TT,
  StringRef CPU,
  StringRef FS) {
    MCSubtargetInfo *X = new MCSubtargetInfo();
    InitVerilogBackendMCSubtargetInfo(X, TT, CPU, FS);
    return X;
}

static MCAsmInfo *createVerilogBackendMCAsmInfo(const Target &T, StringRef TT) {
  MCAsmInfo *MAI = new VerilogBackendMCAsmInfo(T, TT);

  // VirtualFP = (R30 + #0).
  MachineLocation Dst(MachineLocation::VirtualFP);
  MachineLocation Src(VerilogBackend::R30, 0);
  MAI->addInitialFrameState(0, Dst, Src);

  return MAI;
}

static MCCodeGenInfo *createVerilogBackendMCCodeGenInfo(StringRef TT, Reloc::Model RM,
  CodeModel::Model CM,
  CodeGenOpt::Level OL) {
    MCCodeGenInfo *X = new MCCodeGenInfo();
    // For the time being, use static relocations, since there's really no
    // support for PIC yet.
    X->InitMCCodeGenInfo(Reloc::Static, CM, OL);
    return X;
}

// Force static initialization.
extern "C" void LLVMInitializeVerilogBackendTargetMC() {
  // Register the MC asm info.
  RegisterMCAsmInfoFn X(TheVerilogBackendTarget, createVerilogBackendMCAsmInfo);

  // Register the MC codegen info.
  TargetRegistry::RegisterMCCodeGenInfo(TheVerilogBackendTarget,
    createVerilogBackendMCCodeGenInfo);

  // Register the MC instruction info.
  TargetRegistry::RegisterMCInstrInfo(TheVerilogBackendTarget, createVerilogBackendMCInstrInfo);

  // Register the MC register info.
  TargetRegistry::RegisterMCRegInfo(TheVerilogBackendTarget,
    createVerilogBackendMCRegisterInfo);

  // Register the MC subtarget info.
  TargetRegistry::RegisterMCSubtargetInfo(TheVerilogBackendTarget,
    createVerilogBackendMCSubtargetInfo);
}
