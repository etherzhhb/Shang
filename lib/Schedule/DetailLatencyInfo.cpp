//===------------ VSUnit.cpp - Translate LLVM IR to VSUnit  -----*- C++ -*-===//
//
//                      The Shang HLS frameowrk                               //
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Compute the detail ctrlop to ctrlop latency (in cycle ratio) information.
//
//===----------------------------------------------------------------------===//
#include "vtm/DetailLatencyInfo.h"
#include "llvm/ADT/PostOrderIterator.h"
#define DEBUG_TYPE "detail-latency"
#include "llvm/Support/Debug.h"

using namespace llvm;

static cl::opt<bool>
DisableBLC("vtm-disable-blc",
          cl::desc("Disable bit-level chaining"),
          cl::init(false));

INITIALIZE_PASS_BEGIN(DetialLatencyInfo, "detail-latency-info",
                      "Calculating the latency of instructions",
                      false, true)
  INITIALIZE_PASS_DEPENDENCY(MachineBasicBlockTopOrder)
INITIALIZE_PASS_END(DetialLatencyInfo, "detail-latency-info",
                    "Calculating the latency of instructions",
                    false, true)

char DetialLatencyInfo::ID = 0;
const float DetialLatencyInfo::DeltaLatency = FLT_EPSILON * 8.0f;

DetialLatencyInfo::DetialLatencyInfo() : MachineFunctionPass(ID), MRI(0) {
  initializeDetialLatencyInfoPass(*PassRegistry::getPassRegistry());
}

Pass *llvm::createDetialLatencyInfoPass() {
  return new DetialLatencyInfo();
}

void DetialLatencyInfo::getAnalysisUsage(AnalysisUsage &AU) const {
  MachineFunctionPass::getAnalysisUsage(AU);
  AU.addRequiredID(MachineBasicBlockTopOrderID);
  AU.setPreservesAll();
}

typedef DetialLatencyInfo::DepLatInfoTy DepLatInfoTy;
typedef DepLatInfoTy::mapped_type LatInfoTy;

static void updateLatency(DepLatInfoTy &CurLatInfo, InstPtrTy Src,
                          LatInfoTy Latency) {
  float MSBLatency = Latency.first, LSBLatency = Latency.second;
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

static
LatInfoTy getMSB2LSBLatency(LatInfoTy SrcLatency, LatInfoTy Inc, float BitInc) {
  float SrcMSBLatency = SrcLatency.first,
        SrcLSBLatency = SrcLatency.second;
  float MSBInc = Inc.first, LSBInc = Inc.second;

  float MSBLatency = MSBInc + SrcMSBLatency;
  float LSBLatency = std::max(BitInc + SrcLSBLatency,
                              LSBInc + SrcMSBLatency);
  return std::make_pair(MSBLatency, LSBLatency);
}

static
LatInfoTy getCmpLatency(LatInfoTy SrcLatency, LatInfoTy Inc, float BitInc) {
  LatInfoTy LatInfo = getMSB2LSBLatency(SrcLatency, Inc, BitInc);
  // We need to get the worst delay because the cmps only have 1 bit output.
  float WorstLat = std::max(LatInfo.first, LatInfo.second);
  return std::make_pair(WorstLat, WorstLat);
}

static
LatInfoTy getLSB2MSBLatency(LatInfoTy SrcLatency, LatInfoTy Inc, float BitInc) {
  float SrcMSBLatency = SrcLatency.first,
        SrcLSBLatency = SrcLatency.second;
  float MSBInc = Inc.first, LSBInc = Inc.second;

  float MSBLatency = std::max(MSBInc + SrcLSBLatency,
                              BitInc + SrcMSBLatency);
  float LSBLatency = LSBInc + SrcLSBLatency;
  return std::make_pair(MSBLatency, LSBLatency);
}

static
LatInfoTy getWorstLatency(LatInfoTy SrcLatency, LatInfoTy Inc, float /*BitInc*/) {
  float SrcMSBLatency = SrcLatency.first,
        SrcLSBLatency = SrcLatency.second;
  float MSBInc = Inc.first, LSBInc = Inc.second;

  float MSBLatency = MSBInc + SrcMSBLatency;
  float LSBLatency = LSBInc + SrcLSBLatency;
  float WorstLatency = std::max(MSBLatency, LSBLatency);
  return std::make_pair(WorstLatency, WorstLatency);
}

static
LatInfoTy getParallelLatency(LatInfoTy SrcLatency, LatInfoTy Inc, float /*BitInc*/) {
  float SrcMSBLatency = SrcLatency.first,
        SrcLSBLatency = SrcLatency.second;
  float MSBInc = Inc.first, LSBInc = Inc.second;
  float MSBLatency = MSBInc + SrcMSBLatency;
  float LSBLatency = LSBInc + SrcLSBLatency;

  return std::make_pair(MSBLatency, LSBLatency);
}

template<typename FuncTy>
static void accumulateDatapathLatency(DepLatInfoTy &CurLatInfo,
                                      const DepLatInfoTy *SrcLatInfo,
                                      LatInfoTy Inc, float BitInc, FuncTy F) {
  typedef DepLatInfoTy::const_iterator src_it;
  // Compute minimal delay for all possible pathes.
  for (src_it I = SrcLatInfo->begin(), E = SrcLatInfo->end(); I != E; ++I)
    updateLatency(CurLatInfo, I->first, F(I->second, Inc, BitInc));
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

static LatInfoTy getBitSliceLatency(unsigned OperandSize,
                                    unsigned UB, unsigned LB,
                                    LatInfoTy SrcLatency) {
  float SrcMSBLatency = SrcLatency.first, SrcLSBLatency = SrcLatency.second;
  assert(OperandSize && "Unexpected zero size operand!");
  // Time difference between MSB and LSB.
  float MSB2LSBDelta = SrcMSBLatency - SrcLSBLatency;
  float DeltaPerBit = MSB2LSBDelta / OperandSize;
  // Compute the latency of LSB/MSB by assuming the latency is increasing linear
  float MSBLatency = SrcLSBLatency + UB * DeltaPerBit,
        LSBLatency = SrcLSBLatency + LB * DeltaPerBit;
  return std::make_pair(MSBLatency, LSBLatency);
}

static float adjustChainedLatency(float Latency, unsigned SrcOpcode,
                                  unsigned DstOpcode) {
  bool SrcWriteUntilFInish = VInstrInfo::isWriteUntilFinish(SrcOpcode);
  bool DstReadAtEmit = VInstrInfo::isReadAtEmit(DstOpcode);

  float Delta = DetialLatencyInfo::DeltaLatency;

  if (DstReadAtEmit && SrcWriteUntilFInish) {
    if (SrcOpcode == VTM::VOpMvPhi) {
      assert((DstOpcode == TargetOpcode::PHI || DstOpcode == VTM::VOpMvPhi
              || VInstrInfo::getDesc(DstOpcode).isTerminator())
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

// Ensure all latency are not smaller than the elemental latency,
// i.e. the latency of a single LUT.
static LatInfoTy ensureElementalLatency(LatInfoTy L) {
  return
    std::make_pair(L.first  == 0.0f ? 0.0f : std::max(L.first, VFUs::LutLatency),
                   L.second == 0.0f ? 0.0f : std::max(L.second, VFUs::LutLatency));
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
LatInfoTy DetialLatencyInfo::getLatencyToDst(const MachineInstr *SrcMI,
                                             unsigned DstOpcode,
                                             unsigned UB, unsigned LB) {
  float MSBLatency = getCachedLatencyResult(SrcMI);
  float LSBLatency = MSBLatency;
  if (!IsCtrlDep || NeedExtraStepToLatchResult(SrcMI, *MRI, MSBLatency)) {
    LSBLatency = MSBLatency
      = adjustChainedLatency(MSBLatency, SrcMI->getOpcode(), DstOpcode);
    // If we are only reading the lower part of the result of SrcMI, and the
    // LSB of the result of SrcMI are available before SrcMI completely finish,
    // we can read the subword before SrcMI finish.
    if (UB && propagateFromLSB2MSB(SrcMI->getOpcode())) {
      unsigned SrcSize = VInstrInfo::getBitWidth(SrcMI->getOperand(0));
      LSBLatency = MSBLatency / SrcSize;
      // DirtyHack: Ignore the invert flag.
      if (SrcSize != 1 && UB != 3) {
        assert(UB <= SrcSize && UB > LB  && "Bad bitslice!");
        tie(MSBLatency, LSBLatency)
          = getBitSliceLatency(SrcSize, UB, LB,
                               LatInfoTy(MSBLatency, LSBLatency));
      }
    }
  } else {
    // IsCtrlDep
    LSBLatency = MSBLatency
      = std::max(0.0f, MSBLatency - DetialLatencyInfo::DeltaLatency);
  }

  return std::make_pair(MSBLatency, LSBLatency);
}

template<bool IsCtrlDep>
void DetialLatencyInfo::buildDepLatInfo(const MachineInstr *SrcMI,
                                        DepLatInfoTy &CurLatInfo,
                                        unsigned UB, unsigned LB,
                                        unsigned DstOpcode){
  const DepLatInfoTy *SrcLatInfo = getDepLatInfo(SrcMI);
  assert(SrcLatInfo && "SrcMI not visited yet?");

  LatInfoTy SrcLatency = getLatencyToDst<IsCtrlDep>(SrcMI, DstOpcode, UB, LB);

  // Try to compute the per-bit latency.
  float BitLatency = 0.0f;
  if (unsigned Size = UB - LB)
    BitLatency = std::max((SrcLatency.first - SrcLatency.second) / Size,
                          VFUs::LutLatency);

  SrcLatency = ensureElementalLatency(SrcLatency);
  unsigned Opcode = SrcMI->getOpcode();
  bool isCtrl = VInstrInfo::isControl(SrcMI->getOpcode());
  if (DisableBLC && Opcode != VTM::VOpBitSlice)
    Opcode = VTM::INSTRUCTION_LIST_END;

  switch (Opcode) {
  default:
    updateLatency(CurLatInfo, SrcMI, SrcLatency);
    if (!isCtrl)
      accumulateDatapathLatency(CurLatInfo, SrcLatInfo, SrcLatency, BitLatency,
                                getWorstLatency);
    return;
    // Result bits are computed from LSB to MSB.
  case VTM::VOpAdd_c:
  case VTM::VOpMultLoHi_c:
  case VTM::VOpMult_c:
    accumulateDatapathLatency(CurLatInfo, SrcLatInfo, SrcLatency, BitLatency,
                              getLSB2MSBLatency);
    /* FALL THOUGH */
  case VTM::VOpAdd:
  case VTM::VOpMult:
  case VTM::VOpMultLoHi:
    updateLatency(CurLatInfo, SrcMI, SrcLatency);
    return;
  case VTM::VOpBitSlice:
    // Forward the latency from the source of the bitslice, and increase the
    // MSBLatency and LSBLatency according to the upper bound and lowerbound
    // of the bitslice.
    if (SrcMI->getOperand(1).isReg()) {
      unsigned SrcReg = SrcMI->getOperand(1).getReg();
      MachineInstr *BitSliceSrc = MRI->getVRegDef(SrcReg);
      assert(BitSliceSrc && "The source MachineInstr for BitSlice not found!");
      // Update SrcMSBLatency and SrcLSBLatency according to the upper bound
      // and the lower bound of the bitslice.
      UB = SrcMI->getOperand(2).getImm();
      LB = SrcMI->getOperand(3).getImm();
      // Create the entry for the bitslice, the latency of the bitslice is the
      // same as the scaled BitSliceSrc.
      SrcLatency = getLatencyToDst<IsCtrlDep>(BitSliceSrc, DstOpcode, UB, LB);
      SrcLatency = ensureElementalLatency(SrcLatency);
      updateLatency(CurLatInfo, SrcMI, SrcLatency);

      buildDepLatInfo<IsCtrlDep>(BitSliceSrc, CurLatInfo, UB, LB, DstOpcode);
      return;
    }
    /* FALL THOUGH */
  // Each bits are compute independently.
  case VTM::VOpSel:
  case VTM::VOpLUT:
  case VTM::VOpAnd:
  case VTM::VOpOr:
  case VTM::VOpXor:
  case VTM::VOpNot:
  case VTM::VOpBitCat:
    accumulateDatapathLatency(CurLatInfo, SrcLatInfo, SrcLatency, BitLatency,
                              getParallelLatency);
    updateLatency(CurLatInfo, SrcMI, SrcLatency);
    return;
  case VTM::VOpICmp_c:
    // The result of ICmp is propagating from MSB to LSB.
    SrcLatency.first = BitLatency;
    accumulateDatapathLatency(CurLatInfo, SrcLatInfo, SrcLatency, BitLatency,
                              getCmpLatency);
    /* FALL THOUGH */
  case VTM::VOpICmp:
    // Result bits are computed from MSB to LSB.
    updateLatency(CurLatInfo, SrcMI, SrcLatency);
    return;
  }
}

const DetialLatencyInfo::DepLatInfoTy &
DetialLatencyInfo::addInstrInternal(const MachineInstr *MI,
                                    DepLatInfoTy &CurLatInfo) {
  const MachineBasicBlock *CurMBB = MI->getParent();
  unsigned Opcode = MI->getOpcode();
  bool IsControl = VInstrInfo::isControl(Opcode);

  // Iterate from use to define, ignore the the incoming value of PHINodes.
  // Because the incoming value may be not visited yet.
  for (unsigned i = 0, e = MI->isPHI() ? 1 : MI->getNumOperands(); i != e; ++i){
    const MachineOperand &MO = MI->getOperand(i);

    // Only care about a use register.
    if (!MO.isReg() || MO.isDef() || MO.getReg() == 0)
      continue;

    unsigned SrcReg = MO.getReg();
    MachineInstr *SrcMI = MRI->getVRegDef(SrcReg);
    assert(SrcMI && "Virtual register use without define!");

    // Do we ignore phi as dependence? Also ignore self loop.
    assert(SrcMI != MI && "Unexpected self-loop!");

    unsigned OpSize = VInstrInfo::getBitWidth(MO);

    if (Opcode == VTM::VOpBitSlice) {
      // Directly copy the dependencies latency information, because when
      // calculating latency, we treat it as the alias of SrcMI, exepct the
      // latencies are scaled according to the lower bound and upper bound of
      // the bitslice.
      CurLatInfo = *getDepLatInfo(SrcMI);
      // The latency of SrcMI is included into the latency of the bitslice.
      // Hence we need to set the latency of SrcMI to 0.0f to avoid accumulating
      // it more than once.
      CurLatInfo[SrcMI] = LatInfoTy(0.0f, 0.0f);
      continue;
    }

    buildDepLatInfo<false>(SrcMI, CurLatInfo, OpSize, 0, Opcode);
  }

  // Compute the latency of MI.
  float Latency = computeLatencyFor(MI);

  // We will not get any latency information if a datapath operation do not
  // depends any control operation in the same BB.
  if (CurLatInfo.empty() && (!IsControl || MI->isPHI())) {
    Latency = std::max(Latency, DetialLatencyInfo::DeltaLatency);
    CurLatInfo.insert(std::make_pair(CurMBB, std::make_pair(Latency, Latency)));
  }

  return CurLatInfo;
}

void DetialLatencyInfo::buildExitMIInfo(const MachineInstr *ExitMI,
                                        DepLatInfoTy &Info,
                                        MISetTy &MIsToWait, MISetTy &MIsToRead){
  typedef MISetTy::const_iterator exit_it;
  // Exiting directly, no need to read the result fore fore exting.
  for (exit_it I = MIsToWait.begin(), E = MIsToWait.end(); I != E; ++I)
    buildDepLatInfo<true>(*I, Info, 0, 0.0, ExitMI->getOpcode());

  // Exiting via data-path operation, the value need to be read before exiting.
  for (exit_it I = MIsToRead.begin(), E = MIsToRead.end(); I != E; ++I)
    buildDepLatInfo<false>(*I, Info, 0, 0.0, ExitMI->getOpcode());
}

float DetialLatencyInfo::getChainedLatency(const MachineInstr *SrcInstr,
                                            const MachineInstr *DstInstr) const{
  // Compute the latency correspond to detail slot.
  float latency = getMaxLatency(SrcInstr);
  return adjustChainedLatency(latency, SrcInstr->getOpcode(),
                              DstInstr->getOpcode());
}

bool DetialLatencyInfo::runOnMachineFunction(MachineFunction &MF) {
  MRI = &MF.getRegInfo();

  typedef MachineFunction::iterator iterator;
  typedef MachineBasicBlock::instr_iterator instr_iterator;
  for (iterator BI = MF.begin(), BE = MF.end(); BI != BE; ++BI)
    for (instr_iterator I = BI->instr_begin(), E = BI->instr_end(); I != E; ++I)
      addInstrInternal(I,  LatencyMap[I]);

  return false;
}
