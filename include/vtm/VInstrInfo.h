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
                             bool AllowModify = false) const;
  virtual bool DefinesPredicate(MachineInstr *MI,
                                std::vector<MachineOperand> &Pred) const;
  virtual bool PredicateInstruction(MachineInstr *MI,
                                    const SmallVectorImpl<MachineOperand> &Pred)
                                    const;
  static MachineInstr *
  PredicatePseudoInstruction(MachineInstr *MI, const TargetInstrInfo *TII,
                             const SmallVectorImpl<MachineOperand> &Pred);

  virtual unsigned RemoveBranch(MachineBasicBlock &MBB) const;
  virtual unsigned InsertBranch(MachineBasicBlock &MBB,
                                MachineBasicBlock *TBB, MachineBasicBlock *FBB,
                                const SmallVectorImpl<MachineOperand> &Cond,
                                DebugLoc DL) const;
  static void ReversePredicateCondition(MachineOperand &Cond);
  static bool isUnConditionalBranch(MachineInstr *MI);
  virtual bool ReverseBranchCondition(SmallVectorImpl<MachineOperand> &Cond)
    const;
  virtual bool isProfitableToDupForIfCvt(MachineBasicBlock &MBB,
                                         unsigned NumCyles, float Probability,
                                         float Confidence) const;
  virtual bool isProfitableToIfCvt(MachineBasicBlock &MBB, unsigned NumCyles,
                                   unsigned ExtraPredCycles,
                                    float Probability, float Confidence) const;
  virtual bool isProfitableToIfCvt(MachineBasicBlock &TMBB,
                                   unsigned NumTCycles,
                                   unsigned ExtraTCycles,
                                   MachineBasicBlock &FMBB,
                                   unsigned NumFCycles,
                                   unsigned ExtraFCycles,
                                   float Probability, float Confidence) const;

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


  static void mergePHISrc(MachineBasicBlock *Succ, MachineBasicBlock *FromBB,
                          MachineBasicBlock *ToBB, MachineRegisterInfo &MRI,
                          const SmallVectorImpl<MachineOperand> &FromBBCnd);

  static const MachineOperand *getPredOperand(const MachineInstr *MI);
  static MachineOperand *getPredOperand(MachineInstr *MI);

  // Build machine instructions for a = Pred ? IfTrueVal : IfFalseVal in MBB,
  // and return the register that holding this value.
  static MachineInstr &BuildSelect(MachineBasicBlock *MBB, MachineOperand &Res,
                                   const SmallVectorImpl<MachineOperand> &Pred,
                                   MachineOperand IfTrueVal,
                                   MachineOperand IfFalseVal);
  static MachineInstr &BuildSelect(MachineBasicBlock *MBB, MachineOperand &Res,
                                   MachineOperand Pred,
                                   MachineOperand IfTrueVal,
                                   MachineOperand IfFalseVal);

  static MachineInstr&
  BuildConditionnalMove(MachineBasicBlock &MBB, MachineBasicBlock::iterator IP,
                        MachineOperand &Res,
                        const SmallVectorImpl<MachineOperand> &Pred,
                        MachineOperand IfTrueVal, const TargetInstrInfo *TII);

  static bool isCopyLike(unsigned Opcode, bool IncludeMoveImm = true);
  static bool isBrCndLike(unsigned Opcode);
  static bool isWireOp(const TargetInstrDesc &TID);

  static unsigned computeLatency(const MachineInstr *SrcInstr,
                                 const MachineInstr *DstInstr);
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

  bool isCopyLike(bool IncludeMoveImm = true) const {
    return VInstrInfo::isCopyLike(getTID().getOpcode());
  }

  bool isBrCndLike() const {
    return VInstrInfo::isBrCndLike(getTID().getOpcode());
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

    // DiryHack: Latency of CalleeFN is 1.
    if (ResTy == VFUs::CalleeFN) return 1;

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
