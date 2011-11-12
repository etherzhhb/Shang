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
#include <map>

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
  PredicatePseudoInstruction(MachineInstr *MI,
                             const SmallVectorImpl<MachineOperand> &Pred);

  virtual unsigned RemoveBranch(MachineBasicBlock &MBB) const;
  virtual unsigned InsertBranch(MachineBasicBlock &MBB,
                                MachineBasicBlock *TBB, MachineBasicBlock *FBB,
                                const SmallVectorImpl<MachineOperand> &Cond,
                                DebugLoc DL) const;
  static void ReversePredicateCondition(MachineOperand &Cond);
  static bool isUnConditionalBranch(MachineInstr *MI);

  // Advance version of AnalyzeBranch.
  typedef std::map<MachineBasicBlock*, MachineOperand> JT;
  static bool extractJumpTable(MachineBasicBlock &BB, JT &Table);
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

  static bool isCopyLike(unsigned Opcode);
  static bool isBrCndLike(unsigned Opcode);

  static unsigned getCtrlStepBetween(const MachineInstr *SrcInstr,
                                     const MachineInstr *DstInstr);
  // Return the latency of a MachineInstr in cycle ratio.
  static double getDetialLatency(const MachineInstr *MI);
  static unsigned getStepsToFinish(const MachineInstr *MI) {
    return unsigned(ceil(getDetialLatency(MI)));
  }
  // Return the edge latency between SrcInstr and DstInstr considering chaining
  // effect.
  static double getChainingLatency(const MachineInstr *SrcInstr,
                                   const MachineInstr *DstInstr);
  // Return the latency from the entry of the MachineBasicBlock to DstInstr.
  static unsigned getStepsFromEntry(const MachineInstr *DstInstr);

  static bool isCmdSeq(unsigned Cmd);
  static bool isInSameCmdSeq(const MachineInstr *PrevMI, const MachineInstr *MI);
  static bool isCmdSeqBegin(const MachineInstr *MI);
  static bool isCmdSeqEnd(const MachineInstr *MI);

private:
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

public:
  // Return the information that encoded into the TSFlags
  static bool isWriteUntilFinish(unsigned OpC);
  static bool isDatapath(unsigned OpC);
  static bool isControl(unsigned OpC) { return !isDatapath(OpC); }
  static bool isLazyEmit(unsigned OpC);
  static VFUs::FUTypes getFUType(unsigned OpC);
  static bool hasTrivialFU(unsigned OpC) {
    return getFUType(OpC) == VFUs::Trivial;
  }

  //static unsigned getTrivialLatency(unsigned OpC);
  static bool isReadAtEmit(unsigned OpC);

  static bool isPrebound(unsigned Opcode);
  static FuncUnitId getPrebindFUId(const MachineInstr *MI);
  static bool mayLoad(const MachineInstr *MI);
  static bool mayStore(const MachineInstr *MI);
};

// Helper class for manipulating bit width operand.
class BitWidthAnnotator {
  MachineOperand *MO;
  uint64_t BitWidths;
public:
  explicit BitWidthAnnotator(uint64_t O = 0) : MO(0), BitWidths(O){}
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
  // The latency from a given operation to the current operation.
  typedef std::map<const MachineInstr*, double> DepLatInfoTy;
private:
  // The latency from all register source through the datapath to a given
  // wire/register define by a datapath/control op
  typedef std::map<const MachineInstr*, DepLatInfoTy> LatencyMapTy;

  LatencyMapTy LatencyMap;
  MachineRegisterInfo &MRI;

  // Update the latency entry in the latency information table.
  static void updateLatency(DepLatInfoTy &CurLatInfo,
                            const MachineInstr *SrcMI,
                            double CurLatency);

  // Add the latency information from SrcMI to CurLatInfo.
  bool buildDepLatInfo(const MachineInstr *SrcMI, const MachineInstr *DstMI,
                       DepLatInfoTy &CurLatInfo);

  // Forward all datapath latencies so the latency information table only
  // contains control to control latency.
  void accumulateDatapathLatencies(DepLatInfoTy &CurLatInfo,
                                   const DepLatInfoTy &SrcLatInfo,
                                   double CurLatency);

  const DepLatInfoTy &addInstrInternal(const MachineInstr *MI,
                                           bool IgnorePHISrc);
  // Also remember the operations that do not use by any others operations in
  // the same bb.
  std::set<const MachineInstr*> ExitMIs;
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
  const DepLatInfoTy *getOperandLatInfo(const MachineInstr *DstMI) const {
    LatencyMapTy::const_iterator at = LatencyMap.find(DstMI);
    return at == LatencyMap.end() ? 0 : &at->second;
  }

  // All operation must finish before the BB exit, this function build the
  // information about the latency from instruction to the BB exit.
  void buildExitMIInfo(ArrayRef</*const */MachineInstr*> Terminators,
                       DepLatInfoTy &Info);

  void reset() {
    LatencyMap.clear();
    ExitMIs.clear();
  }
};

// Compute the cycle latency of a given MBB.
class CycleLatencyInfo {
  typedef std::map<unsigned, std::pair<MachineInstr*, unsigned> > DepLatencyMap;
  DepLatencyMap DepInfo;

  unsigned getLatencyFrom(unsigned Reg, MachineInstr *MI) const ;

  typedef std::map<unsigned, std::pair<MachineInstr*, unsigned> > FULatencyInfo;
  FULatencyInfo FUInfo;

  unsigned updateFULatency(unsigned FUId, unsigned Latency, MachineInstr *MI);

public:
  CycleLatencyInfo() {}

  unsigned computeLatency(MachineBasicBlock &MBB);

  void reset() {
    DepInfo.clear();
    FUInfo.clear();
  }
};
} // end namespace llvm

#endif
