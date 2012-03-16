//===-- VerilogBackendMCTargetDesc.cpp - Cell VerilogBackend Target Descriptions -----*- C++ -*-===//
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
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/TargetRegistry.h"

#define GET_INSTRINFO_MC_DESC
#include "VerilogBackendGenInstrInfo.inc"

#define GET_REGINFO_MC_DESC
#include "VerilogBackendGenRegisterInfo.inc"

using namespace llvm;

static MCInstrInfo *createVerilogBackendMCInstrInfo() {
  MCInstrInfo *X = new MCInstrInfo();
  InitVTMMCInstrInfo(X);
  return X;
}

static MCRegisterInfo *createVerilogBackendMCRegisterInfo(StringRef TT) {
  MCRegisterInfo *X = new MCRegisterInfo();
  InitVTMMCRegisterInfo(X, VTM::R);
  return X;
}

static MCAsmInfo *createVerilogBackendMCAsmInfo(const Target &T, StringRef TT) {
  //MCAsmInfo *MAI = new VerilogBackendMCAsmInfo(T, TT);

  //// VirtualFP = (R30 + #0).
  //MachineLocation Dst(MachineLocation::VirtualFP);
  //MachineLocation Src(VerilogBackend::R30, 0);
  //MAI->addInitialFrameState(0, Dst, Src);

  return new MCAsmInfo();//MAI;
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
  RegisterMCAsmInfoFn X(TheVBackendTarget, createVerilogBackendMCAsmInfo);

  // Register the MC codegen info.
  TargetRegistry::RegisterMCCodeGenInfo(TheVBackendTarget,
                                        createVerilogBackendMCCodeGenInfo);

  // Register the MC instruction info.
  TargetRegistry::RegisterMCInstrInfo(TheVBackendTarget,
                                      createVerilogBackendMCInstrInfo);

  // Register the MC register info.
  TargetRegistry::RegisterMCRegInfo(TheVBackendTarget,
                                    createVerilogBackendMCRegisterInfo);
}

namespace llvm {
  Target TheVBackendTarget;
}

extern "C" void LLVMInitializeVerilogBackendTargetInfo() { 
  RegisterTarget<> X(TheVBackendTarget, "verilog", "Verilog backend");
}

