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

#include "VFunctionUnit.h"
#include "VRegisterInfo.h"

#include "llvm/Target/TargetInstrInfo.h"
namespace llvm {
class VTMConfig;
class TargetData;
class TargetLowering;

class VInstrInfo : public TargetInstrInfoImpl {
  const VRegisterInfo RI;
public:
  VInstrInfo(const TargetData &TD, const TargetLowering &TLI);

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

  const MachineInstr *Instr;
public:
  // Note that passing null to I is allow.
  explicit VTIDReader(const MachineInstr *I) : Instr(I) {}

  inline VFUs::FUTypes getFUType() const {
    return (VFUs::FUTypes)
      ((Instr->getDesc().TSFlags >> ResTypeShiftAmount) & ResTypeMask);
  }

  bool hasTrivialFU() const { return getFUType() == VFUs::Trivial; }

  inline unsigned getTrivialLatency() const {
    assert(getFUType() == VFUs::Trivial && "Bad resource Type!");
    return ((Instr->getDesc().TSFlags >> TrivialLatencyShiftAmount)
             & TrivialLatencyMask);
  }

  // Get the latency of a specific type of Instruction.
  unsigned getLatency(const VTMConfig &VTC) const;

  inline bool isReadAtEmit() const {
    return Instr->getDesc().TSFlags & (ReadAtEmitMask << ReadAtEmitShiftAmount);
  }

  const TargetInstrDesc* operator->() const { return &Instr->getDesc(); }

  // Function unit id.
  static const unsigned TrivialFUId = ~0;

  unsigned getPrebindFUId() const;
  bool isFUBinded() const;
};

} // end namespace llvm

#endif
