#include "vtm/DetailLatencyInfo.h"

using namespace llvm;

char DetialLatencyInfo::ID = 0;

INITIALIZE_PASS(DetialLatencyInfo, "Detail-latency-info",
                "detail latency info",
                false, true)

DetialLatencyInfo::DetialLatencyInfo() : MachineFunctionPass(ID), MRI(0), 
                                        WaitAllOps(false) {
  initializeDetialLatencyInfoPass(*PassRegistry::getPassRegistry());
}

Pass *llvm::createDetialLatencyInfoPass() {
  return new DetialLatencyInfo();
}

void DetialLatencyInfo::getAnalysisUsage(AnalysisUsage &AU) const {
  MachineFunctionPass::getAnalysisUsage(AU);
  AU.setPreservesAll();
}

typedef DetialLatencyInfo::DepLatInfoTy DepLatInfoTy;
typedef DepLatInfoTy::mapped_type LatInfoTy;

static cl::opt<bool>
EnableBLC("vtm-enable-blc", cl::desc("Enable bit level chaining"),
  cl::init(true));

static void updateLatency(DepLatInfoTy &CurLatInfo, InstPtrTy Src,
                          float MSBLatency, float LSBLatency) {
  // Latency from a control operation is simply the latency of the control
  // operation.
  // We may have dependency like:
  //  other op
  //    |   \
  //    |   other op
  //    |   /
  // current op
  // We should update the latency if we get a bigger latency.
  DepLatInfoTy::mapped_type &V = CurLatInfo[Src];
  float &OldLSBLatency = V.second;
  OldLSBLatency = std::max(OldLSBLatency, LSBLatency);
  //assert(LSBLatency <= MSBLatency && "Broken latency pair!");
  float &OldMSBLatency = V.first;
  OldMSBLatency = std::max(OldMSBLatency, MSBLatency);
}

static LatInfoTy getMSB2LSBLatency(float SrcMSBLatency, float SrcLSBLatency,
                                   float TotalLatency, float PerBitLatency) {
  float MSBLatency = PerBitLatency + SrcMSBLatency;
  float LSBLatency = std::max(PerBitLatency + SrcLSBLatency,
    TotalLatency + SrcMSBLatency);
  return std::make_pair(MSBLatency, LSBLatency);
}

static LatInfoTy getCmpLatency(float SrcMSBLatency, float SrcLSBLatency,
                               float TotalLatency, float PerBitLatency) {
  LatInfoTy LatInfo = getMSB2LSBLatency(SrcMSBLatency, SrcLSBLatency,
    TotalLatency, PerBitLatency);
  // We need to get the worst delay because the cmps only have 1 bit output.
  float WorstLat = std::max(LatInfo.first, LatInfo.second);
  return std::make_pair(WorstLat, WorstLat);
}

static LatInfoTy getLSB2MSBLatency(float SrcMSBLatency, float SrcLSBLatency,
                                   float TotalLatency, float PerBitLatency) {
  float MSBLatency = std::max(TotalLatency + SrcLSBLatency,
    PerBitLatency + SrcMSBLatency);
  float LSBLatency = PerBitLatency + SrcLSBLatency;
  return std::make_pair(MSBLatency, LSBLatency);
}

template<typename FuncTy>
static void accumulateDatapathLatency(DepLatInfoTy &CurLatInfo,
                                      const DepLatInfoTy *SrcLatInfo,
                                      float SrcMSBLatency, float PerBitLatency,
                                      FuncTy F) {
  typedef DepLatInfoTy::const_iterator src_it;
  // Compute minimal delay for all possible pathes.
  for (src_it I = SrcLatInfo->begin(), E = SrcLatInfo->end(); I != E; ++I) {
    float MSBLatency, LSBLatency;
    tie(MSBLatency, LSBLatency) = F(I->second.first, I->second.second,
      SrcMSBLatency, PerBitLatency);
    updateLatency(CurLatInfo, I->first, MSBLatency, LSBLatency);
  }
}

static bool NeedExtraStepToLatchResult(const MachineInstr *MI,
                                       const MachineRegisterInfo &MRI,
                                       float Latency) {
  if (MI->getNumOperands() == 0) return false;

  const MachineOperand &MO = MI->getOperand(0);
  if (!MO.isReg() || !MO.isDef()) return false;

  assert(MO.getReg() && "Broken instruction defining register 0!");
  return Latency != 0.0f && VInstrInfo::isWriteUntilFinish(MI->getOpcode())
    && !MRI.use_empty(MO.getReg());
}

namespace {
struct BitSliceLatencyFN {
  unsigned OperandSize, UB, LB;

  BitSliceLatencyFN(const MachineInstr *BitSliceOp)
    : OperandSize(VInstrInfo::getBitWidth(BitSliceOp->getOperand(1))),
    UB(BitSliceOp->getOperand(2).getImm()),
    LB(BitSliceOp->getOperand(3).getImm()) {
      assert(BitSliceOp->getOpcode() == VTM::VOpBitSlice && "Not a bitslice!");
  }

  BitSliceLatencyFN(unsigned operandSize, unsigned ub)
    : OperandSize(operandSize), UB(ub), LB(0) {}

  LatInfoTy operator()(float SrcMSBLatency, float SrcLSBLatency,
    float /*TotalLatency*/, float /*PerBitLatency*/) {
      return getBitSliceLatency(OperandSize, UB, LB, SrcMSBLatency, 
                                SrcLSBLatency);
  }

  static
    LatInfoTy getBitSliceLatency(unsigned OperandSize, unsigned UB, unsigned LB,
    float SrcMSBLatency, float SrcLSBLatency) {
      assert(OperandSize && "Unexpected zero size operand!");
      // Time difference between MSB and LSB.
      float MSB2LSBDelta = SrcMSBLatency - SrcLSBLatency;
      float DeltaPerBit = MSB2LSBDelta / OperandSize;
      // Compute the latency of LSB/MSB by assuming the latency is increasing linear
      float MSBLatency = SrcLSBLatency + UB * DeltaPerBit,
        LSBLatency = SrcLSBLatency + LB * DeltaPerBit;
      return std::make_pair(MSBLatency, LSBLatency);
  }

  static float getBitSliceLatency(unsigned OperandSize, unsigned UB,
    float SrcMSBLatency) {
      assert(OperandSize && "Unexpected zero size operand!");
      // Compute the latency of MSB by assuming the latency is increasing linear
      float DeltaPerBit = SrcMSBLatency / OperandSize;
      return UB * DeltaPerBit;
  }
};
}

static LatInfoTy getWorstLatency(float SrcMSBLatency, float SrcLSBLatency,
                                 float TotalLatency, float /*PerBitLatency*/) {
  float MSBLatency = TotalLatency + SrcMSBLatency;
  float LSBLatency = TotalLatency + SrcLSBLatency;
  float WorstLatency = std::max(MSBLatency, LSBLatency);
  return std::make_pair(WorstLatency, WorstLatency);
}

static LatInfoTy getParallelLatency(float SrcMSBLatency, float SrcLSBLatency,
                                    float TotalLatency, float /*PerBitLatency*/) {
  float MSBLatency = TotalLatency + SrcMSBLatency;
  float LSBLatency = TotalLatency + SrcLSBLatency;
  return std::make_pair(MSBLatency, LSBLatency);
}

static float adjustChainingLatency(float Latency, const MachineInstr *SrcInstr,
                                   const MachineInstr *DstInstr) {
  assert(DstInstr && SrcInstr && "Dst and Src Instr should not be null!");
  assert(SrcInstr != DstInstr && "Computing latency of self loop?");
  const MCInstrDesc &DstTID = DstInstr->getDesc();
  unsigned DstOpC = DstTID.getOpcode();
  const MCInstrDesc &SrcTID = SrcInstr->getDesc();
  unsigned SrcOpC = SrcTID.getOpcode();

  bool SrcWriteUntilFInish = VInstrInfo::isWriteUntilFinish(SrcOpC);
  bool DstReadAtEmit = VInstrInfo::isReadAtEmit(DstOpC);

  float Delta = DetialLatencyInfo::DeltaLatency;

  if (DstReadAtEmit && SrcWriteUntilFInish) {
    if (SrcOpC == VTM::VOpMvPhi) {
      assert((DstOpC == TargetOpcode::PHI || DstOpC == VTM::VOpMvPhi
        || VInstrInfo::getDesc(DstOpC).isTerminator())
        && "VOpMvPhi should only used by PHIs or terminators!!");
      // The latency from VOpMvPhi to PHI is exactly 0, because the VOpMvPhi is
      // simply identical to the PHI at next iteration.
      return 0.0f;
    } else
      // If the edge is reg->reg, the result is ready after the clock edge, add
      // a delta to make sure DstInstr not schedule to the moment right at the
      // SrcInstr finish
      return ceil(Latency) + Delta;
  }

  // If the value is written to register, it has a delta latency
  if (SrcWriteUntilFInish) return Latency + Delta;

  // Chain the operations if dst not read value at the edge of the clock.
  return std::max(0.0f, Latency - Delta);
}

float DetialLatencyInfo::computeLatencyFor(const MachineInstr *MI) {
  float TotalLatency = VInstrInfo::getDetialLatency(MI);
  // Remember the latency from all MI's dependence leaves.
  CachedLatencies.insert(std::make_pair(MI, TotalLatency));
  return TotalLatency;
}

bool DetialLatencyInfo::propagateFromLSB2MSB(unsigned Opcode) {
  switch (Opcode) {
  default: break;
  case VTM::VOpAdd_c:
  case VTM::VOpMultLoHi_c:
  case VTM::VOpMult_c:
  case VTM::VOpAdd:
  case VTM::VOpMult:
  case VTM::VOpMultLoHi:
    return true;
  }

  return false;
}

template<bool IsCtrlDep>
bool DetialLatencyInfo::buildDepLatInfo(const MachineInstr *SrcMI,
                                        const MachineInstr *DstMI,// Not needed
                                        DepLatInfoTy &CurLatInfo,
                                        unsigned OperandWidth,
                                        float OperandDelay) {
  const DepLatInfoTy *SrcLatInfo = getDepLatInfo(SrcMI);
  // Latency information not available, the SrcMI maybe in others BB, no need
  // to compute cross BB latency.
  if (SrcLatInfo == 0) return false;

  float SrcMSBLatency = getCachedLatencyResult(SrcMI);
  if (!IsCtrlDep || NeedExtraStepToLatchResult(SrcMI, *MRI, SrcMSBLatency)) {
    SrcMSBLatency = adjustChainingLatency(SrcMSBLatency, SrcMI, DstMI);
    // If we are only reading the lower part of the result of SrcMI, and the
    // LSB of the result of SrcMI are available before SrcMI completely finish,
    // we can read the subword before SrcMI finish.
    if (OperandWidth && propagateFromLSB2MSB(SrcMI->getOpcode())) {
      unsigned SrcSize = VInstrInfo::getBitWidth(SrcMI->getOperand(0));
      // DirtyHack: Ignore the invert flag.
      if (OperandWidth != SrcSize && SrcSize != 1 && OperandWidth != 3) {
        assert(OperandWidth < SrcSize && "Bad implicit bitslice!");
        SrcMSBLatency =
          BitSliceLatencyFN::getBitSliceLatency(SrcSize, OperandWidth,
          SrcMSBLatency);
      }
    }
  } else // IsCtrlDep
    SrcMSBLatency = std::max(0.0f, SrcMSBLatency - DetialLatencyInfo::DeltaLatency);

  // Try to compute the per-bit latency.
  float PerBitLatency = 0.0f;
  if (OperandWidth)
    PerBitLatency = std::max(SrcMSBLatency / OperandWidth, VFUs::LutLatency);

  unsigned Opcode = VTM::INSTRUCTION_LIST_END;
  bool isCtrl = VInstrInfo::isControl(SrcMI->getOpcode());
  if (EnableBLC) Opcode = SrcMI->getOpcode();

  switch (Opcode) {
  default:
    if (isCtrl)
      updateLatency(CurLatInfo, SrcMI, SrcMSBLatency, SrcMSBLatency);
    else
      accumulateDatapathLatency(CurLatInfo, SrcLatInfo, SrcMSBLatency,
      PerBitLatency, getWorstLatency);
    break;
    // Result bits are computed from LSB to MSB.
  case VTM::VOpAdd_c:
  case VTM::VOpMultLoHi_c:
  case VTM::VOpMult_c:
    accumulateDatapathLatency(CurLatInfo, SrcLatInfo, SrcMSBLatency,
      PerBitLatency, getLSB2MSBLatency);
    break;
  case VTM::VOpAdd:
  case VTM::VOpMult:
  case VTM::VOpMultLoHi:
    updateLatency(CurLatInfo, SrcMI, SrcMSBLatency, PerBitLatency);
    break;
    // Each bits are compute independently.
  case VTM::VOpLUT:
  case VTM::VOpAnd:
  case VTM::VOpOr:
  case VTM::VOpXor:
  case VTM::VOpNot:
  case VTM::VOpBitCat:
    accumulateDatapathLatency(CurLatInfo, SrcLatInfo, SrcMSBLatency,
      PerBitLatency, getParallelLatency);
    break;
  case VTM::VOpBitSlice:
    accumulateDatapathLatency(CurLatInfo, SrcLatInfo, SrcMSBLatency,
      PerBitLatency, BitSliceLatencyFN(SrcMI));
    break;
  case VTM::VOpICmp_c:
    accumulateDatapathLatency(CurLatInfo, SrcLatInfo, SrcMSBLatency,
      PerBitLatency, getCmpLatency);
    break;
  case VTM::VOpICmp:
    // Result bits are computed from MSB to LSB.
    updateLatency(CurLatInfo, SrcMI, PerBitLatency, SrcMSBLatency);
    break;
  }

  return true;
}

void DetialLatencyInfo::eraseFromWaitSet(const MachineInstr *MI) {
  MIsToWait.erase(MI);
  MIsToRead.erase(MI);
}

const DetialLatencyInfo::DepLatInfoTy &
DetialLatencyInfo::addInstrInternal(const MachineInstr *MI, bool IgnorePHISrc) {
  DepLatInfoTy &CurLatInfo = LatencyMap[MI];
  const MachineBasicBlock *CurMBB = MI->getParent();

  const MCInstrDesc &TID = MI->getDesc();
  bool IsControl = VInstrInfo::isControl(TID.getOpcode());

  // Iterate from use to define.
  for (unsigned i = 0, e = MI->getNumOperands(); i != e; ++i) {
    const MachineOperand &MO = MI->getOperand(i);

    // Only care about a use register.
    if (!MO.isReg() || MO.isDef() || MO.getReg() == 0)
      continue;

    unsigned SrcReg = MO.getReg();
    MachineInstr *SrcMI = MRI->getVRegDef(SrcReg);
    assert(SrcMI && "Virtual register use without define!");

    // Do we ignore phi as dependence? Also ignore self loop.
    if ((SrcMI->isPHI() && IgnorePHISrc) || SrcMI == MI) continue;
    unsigned OpSize = VInstrInfo::getBitWidth(MO);

    float OpDelay = 0.0f;
    if (i < TID.getNumOperands() && TID.OpInfo[i].isPredicate()) {
      OpDelay = VFUs::ClkEnSelLatency;
    } else
      OpDelay = VInstrInfo::getOperandLatency(MI, i);

    if (!buildDepLatInfo<false>(SrcMI, MI, CurLatInfo, OpSize, OpDelay))
      continue;

    // If we build the Latency Info for SrcMI successfully, that means SrcMI
    // have user now.
    if (CurMBB != SrcMI->getParent()) continue;

    // Now MI is actually depends on SrcMI in this MBB, no need to wait them
    // explicitly.
    MIsToWait.erase(SrcMI);

    // SrcMI is read by a data-path operation, we need to wait its result before
    // exiting the BB if there is no other control operation read it.
    if (VInstrInfo::isControl(SrcMI->getOpcode()) && !IsControl)
      MIsToRead.insert(SrcMI);
  }

  // Find all MIs that are read by other control operation, and we do not need
  // to read them explicitly.
  if (IsControl)
    for (DepLatInfoTy::iterator I = CurLatInfo.begin(), E = CurLatInfo.end();
      I != E; ++I) {
        const MachineInstr *SrcMI = I->first;
        if (SrcMI == 0 || CurMBB != SrcMI->getParent())
          continue;

        MIsToRead.erase(SrcMI);
    }

    // Align the compute the latency of MI.
    float Latency = computeLatencyFor(MI);

    // Assume MI do not have any user in the same BB, if it has, it will be
    // deleted later.
    if (IsControl || WaitAllOps)
      MIsToWait.insert(MI);

    // We will not get any latency information if a datapath operation do not
    // depends any control operation in the same BB.
    if (CurLatInfo.empty() && !IsControl) {
      float latency = std::max(Latency, DetialLatencyInfo::DeltaLatency);
      CurLatInfo.insert(std::make_pair(CurMBB, std::make_pair(latency, latency)));
    }

    return CurLatInfo;
}

void DetialLatencyInfo::buildExitMIInfo(const MachineInstr *ExitMI,
                                        DepLatInfoTy &Info) {
  typedef std::set<const MachineInstr*>::const_iterator exit_it;
  // Exiting directly, no need to read the result fore fore exting.
  for (exit_it I = MIsToWait.begin(), E = MIsToWait.end(); I != E; ++I)
    buildDepLatInfo<true>(*I, ExitMI, Info, 0, 0.0);

  // Exiting via data-path operation, the value need to be read before exiting.
  for (exit_it I = MIsToRead.begin(), E = MIsToRead.end();
    I != E; ++I)
    buildDepLatInfo<false>(*I, ExitMI, Info, 0, 0.0);
}

const float DetialLatencyInfo::DeltaLatency = FLT_EPSILON * 8.0f;

unsigned DetialLatencyInfo::getStepsFromEntry(const MachineInstr *DstInstr) {
  assert(DstInstr && "DstInstr should not be null!");
  const MCInstrDesc &DstTID = DstInstr->getDesc();
  unsigned DstOpC = DstTID.getOpcode();

  //// Set latency of Control operation and entry root to 1, so we can prevent
  //// scheduling control operation to the first slot.
  //// Do not worry about PHI Nodes, their will be eliminated at the register
  //// allocation pass.
  if (DstInstr->getOpcode() == VTM::PHI) return 0;

  // Schedule datapath operation right after the first control slot.
  if (VInstrInfo::isDatapath(DstOpC)) return 0;

  // Do not schedule function unit operation to the first state at the moment
  // there may be potential resource conflict: The end slot may be at the middle
  // of a BB in a pipelined loop body, in that case, any FU can be actived by
  // the alias slot.
  if (!VInstrInfo::hasTrivialFU(DstOpC) || VInstrInfo::countNumRegUses(DstInstr))
    return 1;

  return 0;
}

float DetialLatencyInfo::getChainingLatency(const MachineInstr *SrcInstr,
                                            const MachineInstr *DstInstr) const{
  // Compute the latency correspond to detail slot.
  float latency = getMaxLatency(SrcInstr);
  return adjustChainingLatency(latency, SrcInstr, DstInstr);
}
