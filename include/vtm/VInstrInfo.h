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
  virtual bool FoldImmediate(MachineInstr *UseMI, MachineInstr *DefMI,
                             unsigned Reg, MachineRegisterInfo *MRI) const;
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

  static bool isAlwaysTruePred(MachineOperand &MO);

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

  /// @brief Merge the source of the PHINodes in Succ after FromBB is predicated
  ///        and merged to ToBB.
  ///
  /// @param Succ       The successor block of FromBB
  /// @param FromBB     The BB that predicated and merged
  /// @param ToBB       The BB that FromBB merged to.
  /// @param MRI
  /// @param FromBBCnd  The condition that jumping from ToBB to FromBB
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
                        MachineOperand IfTrueVal);
  // We need to identify the signals connect to clock enable network, which
  // have big latency if connected to a multiplexer (this introduce by resource
  // sharing algorithm) and likly become critical path.
  static float getOperandLatency(const MachineInstr *MI, unsigned MOIdx);

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

  static MachineInstr *getBundleHead(MachineInstr *MI);

  static inline unsigned getBundleSlot(MachineInstr *MI) {
    return getBundleHead(MI)->getOperand(0).getImm();
  }

  static inline unsigned getInstrSlotNum(MachineInstr *MI) {
    assert(MI->isInsideBundle() && "Cannot get InstrSlot!");
    return VInstrInfo::getPredOperand(MI)[1].getImm();
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

  static MachineOperand CreateReg(unsigned RegNum, unsigned BitWidth,
                                  bool IsDef = false);
  static MachineOperand CreateImm(int64_t Val, unsigned BitWidth);

  static MachineOperand CreatePredicate(unsigned Reg = 0);

  static MachineOperand CreateTrace(MachineBasicBlock *MBB);
};
//ucOperandExpressionTrait - Special DenseMapInfo traits to compare
//ucOperand* by *value* of the instruction rather than by pointer value.
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

// Compute the detail ctrlop to ctrlop latency (in cycle ratio) infromation of
// a given MBB.
class DetialLatencyInfo {
public:
  // The latency of MSB and LSB from a particular operation to the current
  // operation.
  typedef std::map<const MachineInstr*, std::pair<float, float> > DepLatInfoTy;
  static float getLatency(DepLatInfoTy::value_type v) {
    return std::max(v.second.first, v.second.second);
  }

  const static float DeltaLatency;

  MachineRegisterInfo &MRI;
private:
  static float getDetialLatency(const MachineInstr *MI);
  // Cache the computational delay for every instruction.
  typedef std::map<const MachineInstr*, float> CachedLatMapTy;
  CachedLatMapTy CachedLatencies;
  float computeLatencyFor(const MachineInstr *MI);
  CachedLatMapTy::mapped_type getCachedLatencyResult(const MachineInstr *MI) const {
    CachedLatMapTy::const_iterator at = CachedLatencies.find(MI);
    assert(at != CachedLatencies.end() && "Latency not calculated!");
    return at->second;
  }

  // The latency from all register source through the datapath to a given
  // wire/register define by a datapath/control op
  typedef std::map<const MachineInstr*, DepLatInfoTy> LatencyMapTy;

  LatencyMapTy LatencyMap;
  // Add the latency information from SrcMI to CurLatInfo.
  template<bool IsCtrlDep>
  bool buildDepLatInfo(const MachineInstr *SrcMI, const MachineInstr *DstMI,
                       DepLatInfoTy &CurLatInfo, unsigned OperandWidth,
                       float OperandDelay);
  // Also remember the operations that do not use by any others operations in
  // the same bb.
  std::set<const MachineInstr*> ExitMIs;
protected:
  const DepLatInfoTy &addInstrInternal(const MachineInstr *MI,
                                       bool IgnorePHISrc);
public:
  DetialLatencyInfo(MachineRegisterInfo &mri) : MRI(mri) {}

  static const MachineInstr *const EntryMarker;

  // Add the a machine instruction and compute the corresponding latency
  // information, return true if the MI is a control operation, false otherwise.
  void addInstr(const MachineInstr *MI) {
    addInstrInternal(MI, false);
  }

  // Build the back-edge latency information of PHIs.
  const DepLatInfoTy &buildPHIBELatInfo(const MachineInstr *MI) {
    assert(MI->isPHI() && "Expect PHI!");
    // Simply add the back-edge dependence information of the which not
    // available at the first scan, but already available now.
    // Because PHINodes not depends on PHINodes in the same BB, we should ignore
    // PHINodes if it appear as an operand.
    return addInstrInternal(MI, true);
  }

  // Get the source register and the corresponding latency to DstMI
  const DepLatInfoTy *getDepLatInfo(const MachineInstr *DstMI) const {
    LatencyMapTy::const_iterator at = LatencyMap.find(DstMI);
    return at == LatencyMap.end() ? 0 : &at->second;
  }

  // All operation must finish before the BB exit, this function build the
  // information about the latency from instruction to the BB exit.
  void buildExitMIInfo(const MachineInstr *ExitMI, DepLatInfoTy &Info);

  // Erase the instructions from exit set.
  void eraseFromExitSet(const MachineInstr *MI) {
    ExitMIs.erase(MI);
  }

  void addDummyLatencyEntry(const MachineInstr *MI, float l = 0.0f) {
    CachedLatencies.insert(std::make_pair(MI, l));
  }

  void reset() {
    CachedLatencies.clear();
    LatencyMap.clear();
    ExitMIs.clear();
  }

  float getMaxLatency(const MachineInstr *MI) const {
    return getCachedLatencyResult(MI);
  }

  unsigned getStepsToFinish(const MachineInstr *MI) const {
    return ceil(getMaxLatency(MI));
  }


  // Return the edge latency between SrcInstr and DstInstr considering chaining
  // effect.
  float getChainingLatency(const MachineInstr *SrcInstr,
                           const MachineInstr *DstInstr) const;

  static unsigned getStepsFromEntry(const MachineInstr *DstInstr);

  template<bool IsValDep>
  unsigned getCtrlStepBetween(const MachineInstr *SrcInstr,
                              const MachineInstr *DstInstr) {
    if (!SrcInstr) return getStepsFromEntry(DstInstr);

    if (IsValDep) return ceil(getChainingLatency(SrcInstr, DstInstr));

    return getStepsToFinish(SrcInstr);
  }
};

// Compute the cycle latency of a given MBB.
class CycleLatencyInfo : public DetialLatencyInfo {
  typedef std::map<const MachineInstr*, unsigned> DepLatencyMap;
  DepLatencyMap DepInfo;

  typedef std::map<unsigned, std::pair<const MachineInstr*, unsigned> >
          FULatencyInfo;
  FULatencyInfo FUInfo;

  unsigned updateFULatency(unsigned FUId, unsigned Latency, MachineInstr *MI);

public:
  CycleLatencyInfo(MachineRegisterInfo &MRI) : DetialLatencyInfo(MRI) {
    reset();
  }

  unsigned computeLatency(MachineBasicBlock &MBB, bool reset = false);

  void reset() {
    DetialLatencyInfo::reset();
    FUInfo.clear();
    DepInfo.clear();
    DepInfo.insert(std::make_pair(DetialLatencyInfo::EntryMarker, 0));
  }
};
} // end namespace llvm

#endif
