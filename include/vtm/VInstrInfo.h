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
#include "llvm/ADT/PointerUnion.h"

namespace llvm {
class TargetData;
class TargetLowering;

class VInstrInfo : public TargetInstrInfoImpl {
  const VRegisterInfo RI;
public:
  VInstrInfo(const TargetData &TD, const TargetLowering &TLI);

  static const unsigned PredInvertFlag = 0x2;
  virtual bool isPredicable(MachineInstr *MI) const;
  virtual bool isPredicated(const MachineInstr *MI) const;

  virtual bool isUnpredicatedTerminator(const MachineInstr *MI) const;
  virtual bool AnalyzeBranch(MachineBasicBlock &MBB,
                             MachineBasicBlock *&TBB, MachineBasicBlock *&FBB,
                             SmallVectorImpl<MachineOperand> &Cond,
                             bool AllowModify /* = false */) const;
  virtual bool DefinesPredicate(MachineInstr *MI,
                                std::vector<MachineOperand> &Pred) const;
  virtual bool PredicateInstruction(MachineInstr *MI,
                                    const SmallVectorImpl<MachineOperand> &Pred)
                                    const;
  virtual unsigned RemoveBranch(MachineBasicBlock &MBB) const;
  virtual unsigned InsertBranch(MachineBasicBlock &MBB,
                                MachineBasicBlock *TBB, MachineBasicBlock *FBB,
                                const SmallVectorImpl<MachineOperand> &Cond,
                                DebugLoc DL) const;
  virtual bool ReverseBranchCondition(SmallVectorImpl<MachineOperand> &Cond)
    const;

  virtual bool isReallyTriviallyReMaterializable(const MachineInstr *MI,
                                                 AliasAnalysis *AA) const;
  /// getRegisterInfo - TargetInstrInfo is a superset of MRegister info.  As
  /// such, whenever a client has an instance of instruction info, it should
  /// always be able to get register info as well (through this method).
  virtual const VRegisterInfo &getRegisterInfo() const { return RI; }

  virtual unsigned createPHIIncomingReg(unsigned DestReg,
                                           MachineRegisterInfo *MRI) const;
  virtual MachineInstr *insertPHIImpDef(MachineBasicBlock &MBB,
    MachineBasicBlock::iterator InsertPos,
    MachineInstr *PN) const;
  virtual MachineInstr *insertPHIIcomingCopy(MachineBasicBlock &MBB,
    MachineBasicBlock::iterator InsertPos,
    MachineInstr *PN,
    unsigned IncomingReg) const;

  virtual MachineInstr *insertPHICopySrc(MachineBasicBlock &MBB,
    MachineBasicBlock::iterator InsertPos,
    MachineInstr *PN, unsigned IncomingReg,
    unsigned SrcReg,
    unsigned SrcSubReg) const;
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

class VIDesc {
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
    DatapathShiftAmount = 0x5,

    LazyEmitMask = 0x1,
    LazyEmitShiftAmount = 0xa
  };
  
  PointerUnion<const MachineInstr*, const TargetInstrDesc*> Data;
  const TargetInstrDesc &getTID() const {
    if (const MachineInstr *MI = Data.dyn_cast<const MachineInstr*>())
      return MI->getDesc();
    
    return *Data.get<const TargetInstrDesc*>();
  }
  uint64_t getTSFlags() const { return getTID().TSFlags; }
public:
  /*implicit*/ VIDesc(const MachineInstr &MI) : Data(&MI) {}
  /*implicit*/ VIDesc(const TargetInstrDesc &TID) : Data(&TID) {}

  MachineInstr &get() const {
    return *const_cast<MachineInstr*>(Data.get<const MachineInstr*>());
  }

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

  inline bool isLazyEmit() const {
    return getTSFlags() & (LazyEmitMask << LazyEmitShiftAmount);
  }

  const TargetInstrDesc* operator->() const { return &getTID(); }

  FuncUnitId getPrebindFUId() const;

  // Out of line virtual function to provide home for the class.
  virtual void anchor();
};

} // end namespace llvm

#endif
