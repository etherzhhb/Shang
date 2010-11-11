//===------------ VInstrInfo.h - VTM  Instruction Information ----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the Verilog Target Machine implementation of the
// TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef VINSTRUCTIONINFO_H
#define VINSTRUCTIONINFO_H

#include "llvm/Target/TargetInstrInfo.h"
#include "VRegisterInfo.h"

namespace llvm {
class VTMConfig;

class VInstrInfo : public TargetInstrInfoImpl {
  const VRegisterInfo RI;
public:
  enum FUTypes {
    Trivial = 0,
    MemoryBus = 1,
    SHL = 2,
    ASR = 3,
    LSR = 4,
    AddSub = 5,
    Mult = 6,

    FirstResourceType = Trivial,
    LastResourceType = Mult
  };


  VInstrInfo();

  /// getRegisterInfo - TargetInstrInfo is a superset of MRegister info.  As
  /// such, whenever a client has an instance of instruction info, it should
  /// always be able to get register info as well (through this method).
  virtual const VRegisterInfo &getRegisterInfo() const { return RI; }
};


class VTIDReader {
  enum TSFlagsBitFields {
    ResTypeMask = 0x7,
    ResTypeShiftAmount = 0x0,

    TrivialLatencyMask = 0xf,
    TrivialLatencyShiftAmount = 0x4,

    ReadAtEmitMask = 0x1,
    ReadAtEmitShiftAmount = 0x3
  };

  const TargetInstrDesc &TID;
public:
  explicit VTIDReader(const TargetInstrDesc &T) : TID(T) {}

  inline VInstrInfo::FUTypes getHWResType() const {
    return (VInstrInfo::FUTypes)
      ((TID.TSFlags >> ResTypeShiftAmount) & ResTypeMask);
  }

  inline unsigned getTrivialLatency() const {
    assert(getHWResType() == VInstrInfo::Trivial && "Bad resource Type!");
    return ((TID.TSFlags >> TrivialLatencyShiftAmount) & TrivialLatencyMask);
  }
  
  // Get the latency of a specific type of Instruction.
  unsigned getLatency(const VTMConfig &VTC) const;

  inline bool isReadAtEmit() const {
    return TID.TSFlags & (ReadAtEmitMask << ReadAtEmitShiftAmount);
  }

  const TargetInstrDesc* operator->() const { return &TID; }

};

} // end namespace llvm

#endif
