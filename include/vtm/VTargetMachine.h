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
//     optimization.
//   Translate Verilog TargetMachine code to schedule units and perform schedule.
//   Perform register allocation with existing register allocation passes.
//
//===----------------------------------------------------------------------===//
#ifndef VTARGETMACHINE_H
#define VTARGETMACHINE_H

#include "vtm/VInstrInfo.h"
#include "vtm/VISelLowering.h"
#include "vtm/VSelectionDAGInfo.h"
#include "vtm/VSubtarget.h"
// TODO:
// #include "VIntrinsicInfo.h"

#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetData.h"

#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetData.h"
#include "llvm/Target/TargetFrameInfo.h"


namespace llvm {
class tool_output_file;

struct VTargetMachine : public LLVMTargetMachine {
  // FIXME
  const TargetData DataLayout;
  VSubtarget Subtarget;
  VTargetLowering TLInfo;
  VSelectionDAGInfo TSInfo;
  VInstrInfo InstrInfo;
  InstrItineraryData  InstrItins;

  /// mapping allocated instences to atom
  VFUDesc *ResSet[VFUs::NumCommonFUs];

  // FIXME
  // TargetFrameInfo FrameInfo;
  // VIntrinsicInfo IntrinsicInfo;

  VTargetMachine(const Target &T, const std::string &TT, const std::string &FS);

  ~VTargetMachine();

  virtual const VInstrInfo *getInstrInfo() const { return &InstrInfo; }
  // virtual const TargetFrameInfo *getFrameInfo() const { return &FrameInfo; }
  
  virtual const VSubtarget *getSubtargetImpl() const {
    return &Subtarget;
  }

  virtual const InstrItineraryData *getInstrItineraryData() const {
    return &InstrItins;
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

  void initializeTarget();

  // Configuration function.
  void setupMemBus(unsigned latency, unsigned startInt, unsigned totalRes);

  template<class BinOpResType>
  void setupBinOpRes(unsigned latency, unsigned startInt, unsigned totalRes);

  template<class ResType>
  ResType *getFUDesc() const {
    return cast<ResType>(getFUDesc(ResType::getType()));
  }

  VFUDesc *getFUDesc(enum VFUs::FUTypes T) const {
    unsigned idx = (unsigned)T - (unsigned)VFUs::FirstFUType;
    assert(ResSet[idx] && "Bad resource!");
    return ResSet[idx];
  }

  typedef VFUDesc *const * iterator;
  typedef const VFUDesc *const * const_iterator;

  iterator begin() { return &ResSet[0]; }
  const_iterator begin() const { return &ResSet[0]; }

  iterator end() { 
    return begin() + (size_t)VFUs::LastCommonFUType -
      (size_t)VFUs::FirstFUType;
  }

  const_iterator end() const { 
    return begin() + (size_t)VFUs::LastCommonFUType -
      (size_t)VFUs::FirstFUType;
  }
};
extern Target TheVBackendTarget;

} // End llvm namespace


#endif
