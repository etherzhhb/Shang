//===- VerilogTargetMachine.h - TargetMachine for Verilog Backend -*- C++ -*-===//
//
//                      The Shang HLS frameowrk                               //
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares the Verilog TargetMachine, we can leverage existing llvm
// low level optimization with Verilog TargetMachine by:
//   Translate LLVM IR to Verilog TargetMachine Code, perform low level
//     optimization.
//   Translate Verilog TargetMachine code to schedule units and perform schedule.
//   Perform register allocation with existing register allocation passes.
//
//===----------------------------------------------------------------------===//
#ifndef VTARGETMACHINE_H
#define VTARGETMACHINE_H

#include "VSelectionDAGInfo.h"
#include "VFrameLowering.h"
#include "VIntrinsicsInfo.h"

#include "vtm/VInstrInfo.h"
#include "vtm/VISelLowering.h"
// TODO:
// #include "VIntrinsicInfo.h"
#include "llvm/Target/TargetSubtargetInfo.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetData.h"

#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetData.h"


namespace llvm {
class VFrameInfo;

struct VDummySubtarget : public TargetSubtargetInfo {
  VDummySubtarget() : TargetSubtargetInfo() {}
};

class VTargetMachine : public LLVMTargetMachine {
  // FIXME Delete this.
  const TargetData DataLayout;
  VTargetLowering TLInfo;
  VSelectionDAGInfo TSInfo;
  VInstrInfo InstrInfo;
  VIntrinsicInfo IntrinsicInfo;
  VFrameInfo FrameInfo;
  // FIXME
  // VIntrinsicInfo IntrinsicInfo;
  
  // Dummy subtarget.
  VDummySubtarget ST;
public:
  VTargetMachine(const Target &T, StringRef TT,StringRef CPU,
                 StringRef FS, TargetOptions Options, Reloc::Model RM,
                 CodeModel::Model CM, CodeGenOpt::Level OL);

  virtual const VInstrInfo *getInstrInfo() const { return &InstrInfo; }
  // virtual const TargetFrameInfo *getFrameInfo() const { return &FrameInfo; }

  virtual const TargetSubtargetInfo *getSubtargetImpl() const {
    return &ST;
  }

/*  virtual const InstrItineraryData *getInstrItineraryData() const {
    return &InstrItins;
  }*/

  virtual const VRegisterInfo *getRegisterInfo() const {
    return &InstrInfo.getRegisterInfo();
  }

  virtual const VTargetLowering* getTargetLowering() const {
    return &TLInfo;
  }

  virtual const VSelectionDAGInfo* getSelectionDAGInfo() const {
    return &TSInfo;
  }

  virtual const TargetFrameLowering *getFrameLowering() const {
    return &FrameInfo;
  }

  virtual const TargetIntrinsicInfo *getIntrinsicInfo() const {
    return &IntrinsicInfo;
  }

  virtual const TargetData *getTargetData() const { return &DataLayout; }

  virtual bool addInstSelector(PassManagerBase &PM);

  //const TargetIntrinsicInfo *getIntrinsicInfo() const {
  //  return &IntrinsicInfo;
  //}

  /// createPassConfig - Create a pass configuration object to be used by
  /// addPassToEmitX methods for generating a pipeline of CodeGen passes.
  virtual TargetPassConfig *createPassConfig(PassManagerBase &PM);

  bool addPassesToEmitFile(PassManagerBase &PM,
                           formatted_raw_ostream &Out,
                           CodeGenFileType FileType,
                           bool DisableVerify = true);
};
extern Target TheVBackendTarget;

} // End llvm namespace


#endif
