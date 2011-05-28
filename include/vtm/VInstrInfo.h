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

#include "vtm/FUInfo.h"
#include "vtm/VRegisterInfo.h"

#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/Target/TargetOpcodes.h"
namespace llvm {
class TargetData;
class TargetLowering;

class VInstrInfo : public TargetInstrInfoImpl {
  const VRegisterInfo RI;
public:
  VInstrInfo(const TargetData &TD, const TargetLowering &TLI);

  virtual bool AnalyzeBranch(MachineBasicBlock &MBB,
                             MachineBasicBlock *&TBB, MachineBasicBlock *&FBB,
                             SmallVectorImpl<MachineOperand> &Cond,
                             bool AllowModify /* = false */);

  /// getRegisterInfo - TargetInstrInfo is a superset of MRegister info.  As
  /// such, whenever a client has an instance of instruction info, it should
  /// always be able to get register info as well (through this method).
  virtual const VRegisterInfo &getRegisterInfo() const { return RI; }
};

// Helper class for manipulating bit width operand.
class BitWidthAnnotator {
  MachineOperand *MO;
  uint64_t BitWidths;
public:
  explicit BitWidthAnnotator(uint64_t O = 0) : MO(0), BitWidths(O){}
  explicit BitWidthAnnotator(MachineInstr &MI);

  uint64_t get() const { return BitWidths; }

  BitWidthAnnotator setBitWidth(uint8_t width, unsigned Idx) {
    assert(Idx < sizeof(uint64_t) && "Index out of range!");
    uint64_t w = width;
    // Clear the corresponding bit slice.
    BitWidths &= ~((uint64_t)0xff << (Idx * 8));
    // Assign the value to the bit slice.
    BitWidths |= w << (Idx * 8);
    return *this;
  }

  uint8_t getBitWidth(unsigned Idx) const {
    assert(Idx < sizeof(uint64_t) && "Index out of range!");
    uint64_t w = BitWidths;
    return w >> (Idx * 8);
  }

  void updateBitWidth();

  bool hasBitWidthInfo() const;

  // The BitWidthAnnotator is defined as predicate operand in fact, after we
  // read the bitwidth information, change it back to predicate operand.
  void changeToDefaultPred();
};

class VInstr {
  enum TSFlagsBitFields {
    ResTypeMask = 0x7,
    ResTypeShiftAmount = 0x0,

    TrivialLatencyMask = 0xf,
    TrivialLatencyShiftAmount = 0x6,

    ReadAtEmitMask = 0x1,
    ReadAtEmitShiftAmount = 0x3,

    WriteUntilFinishMask = 0x1,
    WriteUntilFinishShiftAmount = 0x4,

    DatapathMask = 0x1,
    DatapathShiftAmount = 0x5
  };
  
  const MachineInstr &I;
  const TargetInstrDesc &getTID() const { return I.getDesc(); }
  uint64_t getTSFlags() const { return getTID().TSFlags; }
public:
  /*implicit*/ VInstr(const MachineInstr &MI) : I(MI) {}

  MachineInstr &get() const { return const_cast<MachineInstr&>(I); }

  // TODO: Add method to help users manipulate the bit width flag.

  inline VFUs::FUTypes getFUType() const {
    return (VFUs::FUTypes)
      ((getTSFlags() >> ResTypeShiftAmount) & ResTypeMask);
  }

  // Can the copy be fused into control block?
  bool canCopyBeFused() const;

  bool hasTrivialFU() const { return getFUType() == VFUs::Trivial; }

  inline unsigned getTrivialLatency() const {
    assert(getFUType() == VFUs::Trivial && "Bad resource Type!");
    return ((getTSFlags() >> TrivialLatencyShiftAmount)
             & TrivialLatencyMask);
  }

  // Get the latency of a specific type of Instruction.
  unsigned getLatency() const {
    VFUs::FUTypes ResTy = getFUType();

    if (ResTy == VFUs::Trivial)
      return getTrivialLatency();

    return getFUDesc(ResTy)->getLatency();
  }

  inline bool isReadAtEmit() const {
    switch (getTID().getOpcode()) {
    default:
      return getTSFlags() & (ReadAtEmitMask << ReadAtEmitShiftAmount);
    case TargetOpcode::COPY:
    case TargetOpcode::PHI:
      return true;
    }
  }

  bool mayLoad() const;
  bool mayStore() const;

  inline bool isWriteUntilFinish() const {
    switch (getTID().getOpcode()) {
    default:
      return getTSFlags()
             & (WriteUntilFinishMask << WriteUntilFinishShiftAmount);
    case TargetOpcode::COPY:
    case TargetOpcode::PHI:
      return true;
    }
  }

  inline bool hasDatapath() const {
    return getTSFlags() & (DatapathMask << DatapathShiftAmount);
  }

  const TargetInstrDesc* operator->() const { return &getTID(); }

  FuncUnitId getPrebindFUId() const;

  // Out of line virtual function to provide home for the class.
  virtual void anchor();
};

} // end namespace llvm

#endif
