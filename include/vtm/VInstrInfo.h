//===------------ VInstrInfo.h - VTM  Instruction Information ----*- C++ -*-===//
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
// This file contains the Verilog Target Machine implementation of the
// TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef VINSTRUCTIONINFO_H
#define VINSTRUCTIONINFO_H

#include "vtm/FUInfo.h"
#include "vtm/VRegisterInfo.h"

#include "llvm/CodeGen/MachineInstrBundle.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/Target/TargetOpcodes.h"
#include "llvm/ADT/PointerUnion.h"
#include <map>

#define GET_INSTRINFO_HEADER
#include "VerilogBackendGenInstrInfo.inc"

namespace llvm {
class TargetData;
class TargetLowering;

class VInstrInfo : public VTMGenInstrInfo {
  VRegisterInfo RI;
public:
  VInstrInfo();

  VRegisterInfo &getRegisterInfo() { return RI; }
  const VRegisterInfo &getRegisterInfo() const { return RI; }

  static const unsigned BitwidthMask = 0xff;
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
  PredicatePseudoInstruction(MachineInstr *MI,
                             const SmallVectorImpl<MachineOperand> &Pred);

  virtual MachineInstr *commuteInstruction(MachineInstr *MI,
                                           bool NewMI = false) const;

  static void ChangeCopyToMove(MachineInstr *CopyMI);

  virtual unsigned RemoveBranch(MachineBasicBlock &MBB) const;
  virtual unsigned InsertBranch(MachineBasicBlock &MBB,
                                MachineBasicBlock *TBB, MachineBasicBlock *FBB,
                                const SmallVectorImpl<MachineOperand> &Cond,
                                DebugLoc DL) const;
  static void ReversePredicateCondition(MachineOperand &Cond);
  static bool isUnConditionalBranch(MachineInstr *MI);

  // Advance version of AnalyzeBranch.
  typedef std::map<MachineBasicBlock*, MachineOperand> JT;
  static bool extractJumpTable(MachineBasicBlock &BB, JT &Table,
                               bool BrOnly = true);
  static void insertJumpTable(MachineBasicBlock &BB, JT &Table, DebugLoc dl);

  static void MergeBranches(MachineBasicBlock *PredFBB,
                            SmallVectorImpl<MachineOperand> &Pred,
                            MachineBasicBlock *&CndTBB,
                            MachineBasicBlock *&CndFBB,
                            SmallVectorImpl<MachineOperand> &Cnd,
                            const TargetInstrInfo *TII);

  static bool isAlwaysTruePred(const MachineOperand &MO);

  static MachineOperand MergePred(MachineOperand OldCndMO,
                                  MachineOperand NewCndMO,
                                  MachineBasicBlock &MBB,
                                  MachineBasicBlock::iterator IP,
                                  MachineRegisterInfo *MRI,
                                  const TargetInstrInfo *TII,
                                  unsigned MergeOpC);

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

  virtual bool shouldAvoidSinking(MachineInstr *MI) const;

  virtual bool isReallyTriviallyReMaterializable(const MachineInstr *MI,
                                                 AliasAnalysis *AA) const;

  static MachineInstr *getEdgeCndAndInsertPos(MachineBasicBlock *From,
                                              MachineBasicBlock *To,
                                              MachineOperand &Pred);
  static MachineInstr *getInsertPosBeforTerminator(MachineBasicBlock *MBB);

  static const MachineOperand *getPredOperand(const MachineInstr *MI);
  static MachineOperand *getPredOperand(MachineInstr *MI);

  static MachineInstr&
  BuildConditionnalMove(MachineBasicBlock &MBB, MachineBasicBlock::iterator IP,
                        MachineOperand &Res,
                        const SmallVectorImpl<MachineOperand> &Pred,
                        MachineOperand IfTrueVal);

  static bool isCopyLike(unsigned Opcode);
  static bool isBrCndLike(unsigned Opcode);
private:
  enum TSFlagsBitFields {
    ResTypeMask = 0xf,
    ResTypeShiftAmount = 0x0,

    ReadAtEmitMask = 0x1,
    ReadAtEmitShiftAmount = 0x4,

    WriteUntilFinishMask = 0x1,
    WriteUntilFinishShiftAmount = 0x5,

    DatapathMask = 0x1,
    DatapathShiftAmount = 0x6
  };

public:
  // Return the information that encoded into the TSFlags
  static bool isWriteUntilFinish(unsigned OpC);
  static bool isDatapath(unsigned OpC);
  static bool isControl(unsigned OpC) { return !isDatapath(OpC); }
  static VFUs::FUTypes getFUType(unsigned OpC);
  static bool hasTrivialFU(unsigned OpC) {
    return getFUType(OpC) == VFUs::Trivial;
  }

  //static unsigned getTrivialLatency(unsigned OpC);
  static bool isReadAtEmit(unsigned OpC);

  static FuncUnitId getPreboundFUId(const MachineInstr *MI);
  static bool mayLoad(const MachineInstr *MI);
  static bool mayStore(const MachineInstr *MI);

  static const MCInstrDesc &getDesc(unsigned Opcode);
  static unsigned countNumRegUses(const MachineInstr *MI);
  static float getDetialLatency(const MachineInstr *MI);

  static MachineInstr *getBundleHead(MachineInstr *MI);

  static inline unsigned getBundleSlot(MachineInstr *MI) {
    return getBundleHead(MI)->getOperand(0).getImm();
  }

  static inline unsigned getInstrSlotNum(MachineInstr *MI) {
    assert(MI->isInsideBundle() && "Cannot get InstrSlot!");
    return VInstrInfo::getTraceOperand(MI)->getImm();
  }

  static void setInstrSlotNum(MachineInstr *MI, unsigned InstrSlot) {
    VInstrInfo::getTraceOperand(MI)->ChangeToImmediate(InstrSlot);
    VInstrInfo::getTraceOperand(MI)->setTargetFlags(0xff);
  }

  static bool isCtrlBundle(MachineInstr *MI);

  static bool isDatapathBundle(MachineInstr *MI);

  static MachineBasicBlock::instr_iterator getCtrlBundleEnd(MachineInstr *MI);

  static bool isPredicateInverted(const MachineOperand &MO) {
    return (MO.getTargetFlags() & VInstrInfo::PredInvertFlag) != 0;
  }

  static unsigned getBitWidthOrZero(const MachineOperand &MO) {
    assert((MO.isImm() || MO.isReg() || MO.isSymbol() || MO.isGlobal())
      && "Unsupported operand type!");
    return MO.getTargetFlags() & VInstrInfo::BitwidthMask;
  }

  static unsigned getBitWidth(const MachineOperand &MO) {
    unsigned BitWidth = getBitWidthOrZero(MO);
    assert(BitWidth && "Bit width information not available!");
    return BitWidth;
  }

  static void setBitWidth(MachineOperand &MO, unsigned BitWidth) {
    unsigned TF = MO.getTargetFlags();
    TF &= ~VInstrInfo::BitwidthMask;
    TF |= BitWidth & VInstrInfo::BitwidthMask;
    MO.setTargetFlags(TF);
    assert(getBitWidthOrZero(MO) == BitWidth && "Bit width overflow!");
  }

  static bool isAllZeros(const MachineOperand &MO);
  static bool isAllOnes(const MachineOperand &MO);

  static MachineOperand CreateReg(unsigned RegNum, unsigned BitWidth,
                                  bool IsDef = false);
  static MachineOperand CreateImm(int64_t Val, unsigned BitWidth);

  static MachineOperand CreatePredicate(unsigned Reg = 0);

  static MachineOperand CreateTrace();
  static MachineOperand *getTraceOperand(MachineInstr *MI) {
    MachineOperand *Pred = getPredOperand(MI);

    return Pred ? Pred + 1 : 0;
  }

  static const MachineOperand *getTraceOperand(const MachineInstr *MI) {
    return getTraceOperand(const_cast<MachineInstr*>(MI));
  }

  static void ResetTrace(MachineInstr *MI) {
    getTraceOperand(MI)->ChangeToImmediate(~UINT64_C(0));
  }

  static bool isPredicateMutex(const MachineInstr *LHS, const MachineInstr *RHS);
};
//MachineOperandExpressionTrait - Special DenseMapInfo traits to compare
//MachineOperand* by *value* of the instruction rather than by pointer value.
//The hashing and equality testing functions ignore definitions so this is
//useful for CSE, etc.
struct VMachineOperandValueTrait : DenseMapInfo<MachineOperand> {
  static inline MachineOperand getEmptyKey() {
    MachineOperand M = MachineOperand::CreateReg(0, false);
    M.setTargetFlags(~0);
    return M;
  }

  static inline MachineOperand getTombstoneKey() {
    MachineOperand M = MachineOperand::CreateReg(0, false);
    M.setTargetFlags(0x7f);
    return M;
  }

  static unsigned getHashValue(MachineOperand Op);

  static bool isEqual(const MachineOperand &LHS, const MachineOperand &RHS) {
    return LHS.isIdenticalTo(RHS);
  }
};

/// VMachineInstrExpressionTrait - Special DenseMapInfo traits to compare
/// MachineInstr* by *value* of the instruction rather than by pointer value.
/// The hashing and equality testing functions ignore definitions so this is
/// useful for CSE, etc.
struct VMachineInstrExpressionTrait : DenseMapInfo<MachineInstr*> {
  static inline MachineInstr *getEmptyKey() {
    return 0;
  }

  static inline MachineInstr *getTombstoneKey() {
    return reinterpret_cast<MachineInstr*>(-1);
  }

  static unsigned getHashValue(const MachineInstr* const &MI);

  static bool isEqual(const MachineInstr* const &LHS,
                      const MachineInstr* const &RHS) {
    if (RHS == getEmptyKey() || RHS == getTombstoneKey() ||
        LHS == getEmptyKey() || LHS == getTombstoneKey())
      return LHS == RHS;
    return LHS->isIdenticalTo(RHS, MachineInstr::IgnoreVRegDefs);
  }
};

// Helper class for manipulating bit width operand.
class BitWidthAnnotator {
  MachineOperand *MO;
  uint64_t BitWidths;
public:
  BitWidthAnnotator(uint64_t O = 0) : MO(0), BitWidths(O){}
  explicit BitWidthAnnotator(MachineInstr &MI);

  uint64_t get() const { return BitWidths; }

  static unsigned size() { return sizeof(uint64_t); }

  BitWidthAnnotator setBitWidth(uint8_t width, unsigned Idx) {
    assert(Idx < size() && "Index out of range!");
    uint64_t w = width;
    // Clear the corresponding bit slice.
    BitWidths &= ~((uint64_t)0xff << (Idx * 8));
    // Assign the value to the bit slice.
    BitWidths |= w << (Idx * 8);
    return *this;
  }

  uint8_t getBitWidth(unsigned Idx) const {
    assert(Idx < size() && "Index out of range!");
    uint64_t w = BitWidths;
    return w >> (Idx * 8);
  }

  uint8_t getBitWidthOrZero(unsigned Idx) const {
    if (Idx < sizeof(uint64_t)) return getBitWidth(Idx);

    return 0;
  }

  void updateBitWidth();

  bool hasBitWidthInfo() const;

  // The BitWidthAnnotator is defined as predicate operand in fact, after we
  // read the bitwidth information, change it back to predicate operand.
  void changeToDefaultPred();
};

} // end namespace llvm

#endif
