//===- VerilogTargetMachine.h - TargetMachine for Verilog Backend -*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares the Verilog TargetMachine, we can leverage existing llvm
// low level optimization with Verilog TargetMachine by:
//   Translate LLVM IR to Verilog TargetMachine Code, perform low level
//     optimization
//   Translate Verilog TargetMachine Code to HWAtom.
//
//===----------------------------------------------------------------------===//
#ifndef VTARGETMACHINE_H
#define VTARGETMACHINE_H

#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetData.h"

#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetData.h"
#include "llvm/Target/TargetFrameInfo.h"

#include "VInstrInfo.h"
#include "VISelLowering.h"
#include "VSelectionDAGInfo.h"
#include "VSubtarget.h"
// TODO:
// #include "VIntrinsicInfo.h"

namespace llvm {

struct VTargetMachine : public LLVMTargetMachine {
  // FIXME
  const TargetData DataLayout;
  VSubtarget Subtarget;
  VTargetLowering TLInfo;
  VSelectionDAGInfo TSInfo;
  VInstrInfo InstrInfo;

  // FIXME
  // TargetFrameInfo FrameInfo;
  // VIntrinsicInfo IntrinsicInfo;

  VTargetMachine(const Target &T, const std::string &TT, const std::string &FS);


  virtual const VInstrInfo *getInstrInfo() const { return &InstrInfo; }
  // virtual const TargetFrameInfo *getFrameInfo() const { return &FrameInfo; }
  
  virtual const VSubtarget *getSubtargetImpl() const {
    return &Subtarget;
  }

  virtual const VRegisterInfo *getRegisterInfo() const {
    return &InstrInfo.getRegisterInfo();
  }
  virtual const VTargetLowering* getTargetLowering() const {
    return &TLInfo;
  }
  virtual const VSelectionDAGInfo* getSelectionDAGInfo() const {
    return &TSInfo;
  }
  virtual const TargetData *getTargetData() const { return &DataLayout; }

  virtual bool addInstSelector(PassManagerBase &PM, CodeGenOpt::Level OptLevel);

  //const TargetIntrinsicInfo *getIntrinsicInfo() const {
  //  return &IntrinsicInfo;
  //}

  bool addPassesToEmitFile(PassManagerBase &, formatted_raw_ostream &,
                           CodeGenFileType, CodeGenOpt::Level,
                           bool /* = true */);
};
extern Target TheVBackendTarget;

} // End llvm namespace


#endif
