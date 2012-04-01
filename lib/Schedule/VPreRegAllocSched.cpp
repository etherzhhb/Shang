//===------------ VSUnit.cpp - Translate LLVM IR to VSUnit  -----*- C++ -*-===//
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
// This file implement the VPreRegAllocSched pass, which construct the VSUnit
// from LLVM IR.
//
//===----------------------------------------------------------------------===//

#include "VSUnit.h"
#include "SchedulingBase.h"
#include "vtm/MicroState.h"

#include "vtm/Passes.h"
#include "vtm/VFInfo.h"
#include "vtm/VerilgoBackendMCTargetDesc.h"

#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/Analysis/ScalarEvolution.h"
#include "llvm/Analysis/ScalarEvolutionExpressions.h"
#include "llvm/Analysis/ValueTracking.h"
#include "llvm/Target/TargetData.h"
#include "llvm/CodeGen/ISDOpcodes.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/MachineLoopInfo.h"
#include "llvm/CodeGen/LiveVariables.h"
#include "llvm/CodeGen/PseudoSourceValue.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/Support/Allocator.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/GraphWriter.h"
#include "llvm/Support/MathExtras.h"
#define DEBUG_TYPE "vtm-sgraph"
#include "llvm/Support/Debug.h"
using namespace llvm;

//===----------------------------------------------------------------------===//
namespace {
/// @brief Schedule the operations.
///
struct VPreRegAllocSched : public MachineFunctionPass {
  const TargetInstrInfo *TII;
  // The loop Info
  MachineRegisterInfo *MRI;
  VFInfo *FInfo;

  TargetData *TD;

  MachineLoopInfo *LI;
  LoopInfo *IRLI;
  AliasAnalysis *AA;
  ScalarEvolution *SE;

  // Total states
  // Cycle is start from 1 because  cycle 0 is reserve for idle state.
  unsigned short totalCycle;
  // Remember the last cmd seq.
  VSUnit *LastCmdSeq;

  VPreRegAllocSched() : MachineFunctionPass(ID), totalCycle(1), LastCmdSeq(0) {}

  //===--------------------------------------------------------------------===//
  // Loop memory dependence information.
  struct LoopDep {
    enum MemDepTypes {
      TrueDep, AntiDep, OutputDep, NoDep
    };

    unsigned Dep    : 2;
    unsigned ItDst  : 30;

    LoopDep(MemDepTypes dep, unsigned itDst)
      : Dep(dep), ItDst(itDst) {}

    LoopDep() : Dep(LoopDep::NoDep), ItDst(0) {}

    bool hasDep() const {
      return Dep != LoopDep::NoDep ;
    }

    unsigned getItDst() const { return ItDst; }

    MemDepTypes getDepType() const {
      return (MemDepTypes)Dep;
    }
  };

  LoopDep analyzeLoopDep(Value *SrcAddr, Value *DstAddr, bool SrcLoad,
                         bool DstLoad, Loop &L, bool SrcBeforeDest);


  LoopDep advancedLoopDepsAnalysis(Value *SrcAddr, Value *DstAddr,
                                   bool SrcLoad, bool DstLoad, Loop &L,
                                   bool SrcBeforeDest, unsigned ElSizeInByte);

  LoopDep createLoopDep(bool SrcLoad, bool DstLoad, bool SrcBeforeDest,
                        int Diff = 0);

  VDMemDep *getMemDepEdge(VSUnit *Src, unsigned Latency, unsigned Diff) {
    return new VDMemDep(Src, Latency, Diff);
  }

  // We need to iterate over the operand latency table.
  typedef DetialLatencyInfo::DepLatInfoTy::const_iterator src_it;

  void addSchedDepForSU(VSUnit *A, VSchedGraph &CurState,
                        bool isExit = false);
  typedef const DetialLatencyInfo::DepLatInfoTy DepLatInfoTy;
  template<typename DepEdgeTy>
  void addSchedDepForMI(MachineInstr *MI, int MIOffset, VSUnit *A,
                        VSchedGraph &CurState, DepLatInfoTy &LatInfo);
  // Add the dependence from the incoming value of PHI to PHI.
  void addIncomingDepForPHI(VSUnit *PN, VSchedGraph &CurState);

  void addValDep(VSchedGraph &CurState, VSUnit *A);

  VSUnit *getDefSU(const MachineOperand &MO, VSchedGraph &CurState,
                   MachineInstr *&DepSrc) {
    // Only care about the register dependences.
    // FIXME: What about chains?
    if (!MO.isReg()) return 0;

    // The instruction do not depend the register defined by itself.
    if (MO.isDef()) return 0;

    unsigned Reg = MO.getReg();

    // It seems that sometimes the Register will be 0?
    if (!Reg) return 0;
    assert(TargetRegisterInfo::isVirtualRegister(Reg)
           && "Unexpected physics register!");

    DepSrc = MRI->getVRegDef(Reg);
    assert(DepSrc && "Register use without define?");
    /// Only add the dependence if DepSrc is in the same MBB with MI.
    return CurState.lookupSUnit(DepSrc);
  }

  void clear();

  void buildMemDepEdges(VSchedGraph &CurState);

  bool couldBePipelined(const MachineBasicBlock *MBB);

  typedef VSchedGraph::iterator su_it;
  void buildControlPathGraph(VSchedGraph &State);
  void buildDataPathGraph(VSchedGraph &State);

  void buildPipeLineDepEdges(VSchedGraph &State);

  typedef MachineBasicBlock::iterator instr_it;
  void buildExitRoot(VSchedGraph &CurState, MachineInstr *FirstTerminator);

  template<int AllowHanging>
  void buildExitDeps( VSchedGraph &CurState );

  void buildSUnit(MachineInstr *MI, VSchedGraph &CurState);

  void mergeDstMux(VSUnit *U, VSchedGraph &CurState);

  bool mergeUnaryOp(MachineInstr *MI, unsigned OpIdx, VSchedGraph &CurState);

  /// @name FunctionPass interface
  //{
  static char ID;

  ~VPreRegAllocSched();
  bool runOnMachineFunction(MachineFunction &MF);

  // Remove redundant code after schedule emitted.
  void cleanUpSchedule();
  bool cleanUpRegisterClass(unsigned RegNum, const TargetRegisterClass *RC);

  bool doInitialization(Module &M) {
    TD = getAnalysisIfAvailable<TargetData>();
    assert(TD && "TargetData will always available in a machine function pass!");
    return false;
  }

  void releaseMemory();
  void getAnalysisUsage(AnalysisUsage &AU) const;
  void print(raw_ostream &O, const Module *M) const;
  //}

  unsigned getTotalCycle() const {
    return totalCycle;
  }

  void setTotalCycle(unsigned Cyc) {
    totalCycle = Cyc;
  }

  unsigned getTotalCycleBitWidth() const {
    return Log2_32_Ceil(totalCycle);
  }

  const char *getPassName() const {
    return "Schedule Hardware Operations for Verilog Backend";
  }
};
}

//===----------------------------------------------------------------------===//
char VPreRegAllocSched::ID = 0;

Pass *llvm::createVPreRegAllocSchedPass() {
  return new VPreRegAllocSched();
}

void VPreRegAllocSched::getAnalysisUsage(AnalysisUsage &AU) const {
  MachineFunctionPass::getAnalysisUsage(AU);
  AU.addRequired<LoopInfo>();
  AU.addPreserved<LoopInfo>();
  AU.addRequired<ScalarEvolution>();
  AU.addPreserved<ScalarEvolution>();
  AU.addRequired<MachineLoopInfo>();
  AU.addRequired<AliasAnalysis>();
  AU.addPreserved<AliasAnalysis>();
  AU.setPreservesCFG();
}

bool VPreRegAllocSched::runOnMachineFunction(MachineFunction &MF) {
  TII = MF.getTarget().getInstrInfo();
  MRI = &MF.getRegInfo();
  FInfo = MF.getInfo<VFInfo>();
  AA = &getAnalysis<AliasAnalysis>();
  LI = &getAnalysis<MachineLoopInfo>();
  IRLI = &getAnalysis<LoopInfo>();
  SE = &getAnalysis<ScalarEvolution>();

  for (MachineFunction::iterator I = MF.begin(), E = MF.end();
       I != E; ++I) {
    MachineBasicBlock *MBB = &*I;
    VSchedGraph State(*MRI, MBB, couldBePipelined(MBB), getTotalCycle());
    buildControlPathGraph(State);
    DEBUG(State.viewGraph());
    State.scheduleCtrl();
    buildDataPathGraph(State);
    DEBUG(State.viewGraph());
    State.scheduleDatapath();
    setTotalCycle(State.getEndSlot() + 1);
    DEBUG(State.viewGraph());
    State.emitSchedule();
  }

  FInfo->setTotalSlots(totalCycle);

  cleanUpSchedule();

  return true;
}

void VPreRegAllocSched::clear() {
  // Reset total Cycle
  totalCycle = 1;
}

void VPreRegAllocSched::releaseMemory() {
  clear();
}


void VPreRegAllocSched::print(raw_ostream &O, const Module *M) const {}

VPreRegAllocSched::~VPreRegAllocSched() {
  clear();
}

//===----------------------------------------------------------------------===//

VPreRegAllocSched::LoopDep
VPreRegAllocSched::analyzeLoopDep(Value *SrcAddr, Value *DstAddr,
                                  bool SrcLoad, bool DstLoad,
                                  Loop &L, bool SrcBeforeDest) {
  uint64_t SrcSize = AliasAnalysis::UnknownSize;
  Type *SrcElTy = cast<PointerType>(SrcAddr->getType())->getElementType();
  if (SrcElTy->isSized()) SrcSize = AA->getTypeStoreSize(SrcElTy);

  uint64_t DstSize = AliasAnalysis::UnknownSize;
  Type *DstElTy = cast<PointerType>(DstAddr->getType())->getElementType();
  if (DstElTy->isSized()) DstSize = AA->getTypeStoreSize(DstElTy);

  if (L.isLoopInvariant(SrcAddr) && L.isLoopInvariant(DstAddr)) {
    // FIXME: What about nested loops?
    // Loop Invariant, let AA decide.
    if (!AA->isNoAlias(SrcAddr, SrcSize, DstAddr, DstSize))
      return createLoopDep(SrcLoad, DstLoad, SrcBeforeDest);
    else
      return LoopDep();
  }

  // TODO: Use "getUnderlyingObject" implemented in ScheduleInstrs?
  // Get the underlying object directly, SCEV will take care of the
  // the offsets.
  Value *SGPtr = GetUnderlyingObject(SrcAddr),
        *DGPtr = GetUnderlyingObject(DstAddr);

  switch(AA->alias(SGPtr, SrcSize, DGPtr, DstSize)) {
  case AliasAnalysis::MustAlias:
    // We can only handle two access have the same element size.
    if (SrcSize == DstSize)
      return advancedLoopDepsAnalysis(SrcAddr, DstAddr, SrcLoad, DstLoad,
                                      L, SrcBeforeDest, SrcSize);
    // FIXME: Handle pointers with difference size.
    // Fall though.
  case AliasAnalysis::MayAlias:
    return createLoopDep(SrcLoad, DstLoad, SrcBeforeDest);
  default:  break;
  }

  return LoopDep();
}

VPreRegAllocSched::LoopDep
VPreRegAllocSched::advancedLoopDepsAnalysis(Value *SrcAddr, Value *DstAddr,
                                            bool SrcLoad, bool DstLoad,
                                            Loop &L, bool SrcBeforeDest,
                                            unsigned ElSizeInByte) {
  const SCEV *SSAddr = SE->getSCEVAtScope(SrcAddr, &L),
             *SDAddr = SE->getSCEVAtScope(DstAddr, &L);
  DEBUG(dbgs() << *SSAddr << " and " << *SDAddr << '\n');
  // Use SCEV to compute the dependencies distance.
  const SCEV *Distance = SE->getMinusSCEV(SSAddr, SDAddr);
  // TODO: Get range.
  if (const SCEVConstant *C = dyn_cast<SCEVConstant>(Distance)) {
    int ItDistance = C->getValue()->getSExtValue();
    if (ItDistance >= 0)
      // The pointer distance is in Byte, but we need to get the distance in
      // Iteration.
      return createLoopDep(SrcLoad, DstLoad, SrcBeforeDest,
                           ItDistance / ElSizeInByte);
    else
      return LoopDep();
  }

  return createLoopDep(SrcLoad, DstLoad, SrcBeforeDest);
}

VPreRegAllocSched::LoopDep
VPreRegAllocSched::createLoopDep(bool SrcLoad, bool DstLoad, bool SrcBeforeDest,
                                 int Diff) {
   if (!SrcBeforeDest && (Diff == 0))
     Diff = 1;

   assert(Diff >= 0 && "Do not create a dependence with diff small than 0!");
   assert(!(SrcLoad && DstLoad) && "Do not create a RAR dep!");

   // WAW
   if (!SrcLoad && !DstLoad )
     return LoopDep(LoopDep::OutputDep, Diff);

   if (!SrcLoad && DstLoad)
     SrcBeforeDest = !SrcBeforeDest;

   return LoopDep(SrcBeforeDest ? LoopDep::AntiDep : LoopDep::TrueDep, Diff);
}

static inline bool mayAccessMemory(const MCInstrDesc &TID) {
  return TID.mayLoad() || TID.mayStore() || TID.isCall();
}

void VPreRegAllocSched::buildMemDepEdges(VSchedGraph &CurState) {
  // The schedule unit and the corresponding memory operand.
  typedef std::vector<std::pair<Value*, VSUnit*> > MemOpMapTy;
  MemOpMapTy VisitedMemOps;
  Loop *IRL = IRLI->getLoopFor(CurState.getMachineBasicBlock()->getBasicBlock());

  typedef VSchedGraph::sched_iterator it;
  for (it I = CurState.sched_begin(), E = CurState.sched_end(); I != E; ++I) {
    VSUnit *DstU = *I;
    MachineInstr *DstMI = DstU->getRepresentativeInst();
    // Skip the non-memory operation and non-call operation.
    if (DstMI == 0) continue;

    if (!mayAccessMemory(DstMI->getDesc())) continue;

    bool isDstLoad = VInstrInfo::mayLoad(DstMI);

    // Dirty Hack: Is the const_cast safe?
    Value *DstMO = 0;
    uint64_t DstSize = AliasAnalysis::UnknownSize;
    // TODO: Also try to get the address information for call instruction.
    if (!DstMI->memoperands_empty() && !DstMI->hasVolatileMemoryRef()) {
      assert(DstMI->hasOneMemOperand() && "Can not handle multiple mem ops!");
      assert(!DstMI->hasVolatileMemoryRef() && "Can not handle volatile op!");
      
      // FIXME: DstMO maybe null in a VOpCmdSeq
      if ((DstMO =
             const_cast<Value*>((*DstMI->memoperands_begin())->getValue()))){
        Type *DstElemTy
          = cast<SequentialType>(DstMO->getType())->getElementType();
        DstSize = TD->getTypeStoreSize(DstElemTy);
        assert(!isa<PseudoSourceValue>(DstMO) && "Unexpected frame stuffs!");
      }
    }

    for (MemOpMapTy::iterator I = VisitedMemOps.begin(), E = VisitedMemOps.end();
         I != E; ++I) {
      Value *SrcMO = I->first;
      VSUnit *SrcU = I->second;

      MachineInstr *SrcMI = SrcU->getRepresentativeInst();

      // Handle unanalyzable memory access.
      if (DstMO == 0 || SrcMO == 0) {
        // Build the Src -> Dst dependence.
        unsigned Latency = VInstrInfo::getCtrlStepBetween<false>(SrcMI, DstMI);
        DstU->addDep(getMemDepEdge(SrcU, Latency, 0));

        // Build the Dst -> Src (in next iteration) dependence.
        if (CurState.enablePipeLine()) {
          Latency = VInstrInfo::getCtrlStepBetween<false>(SrcMI, DstMI);
          SrcU->addDep(getMemDepEdge(DstU, Latency, 1));
        }
        // Go on handle next visited SUnit.
        continue;
      }

      bool isSrcLoad = VInstrInfo::mayLoad(SrcMI);

      // Ignore RAR dependence.
      if (isDstLoad && isSrcLoad) continue;

      if (CurState.enablePipeLine()) {
        assert(IRL && "Can not handle machine loop without IR loop!");
        // Compute the iterate distance.
        LoopDep LD = analyzeLoopDep(SrcMO, DstMO, isSrcLoad, isDstLoad, *IRL, true);

        if (LD.hasDep()) {
          unsigned Latency = VInstrInfo::getCtrlStepBetween<false>(SrcMI, DstMI);
          VDMemDep *MemDep = getMemDepEdge(SrcU, Latency, LD.getItDst());
          DstU->addDep(MemDep);
        }

        // We need to compute if Src depend on Dst even if Dst not depend on Src.
        // Because dependence depends on execute order.
        LD = analyzeLoopDep(DstMO, SrcMO, isDstLoad, isSrcLoad, *IRL, false);

        if (LD.hasDep()) {
          unsigned Latency = VInstrInfo::getCtrlStepBetween<false>(SrcMI, DstMI);
          VDMemDep *MemDep = getMemDepEdge(DstU, Latency, LD.getItDst());
          SrcU->addDep(MemDep);
        }
      } else {
        Type *SrcElemTy
          = cast<SequentialType>(SrcMO->getType())->getElementType();
        size_t SrcSize = TD->getTypeStoreSize(SrcElemTy);

        if (AA->isNoAlias(SrcMO, SrcSize, DstMO, DstSize)) continue;

        // Ignore the No-Alias pointers.
        unsigned Latency = VInstrInfo::getCtrlStepBetween<false>(SrcMI, DstMI);
        VDMemDep *MemDep = getMemDepEdge(SrcU, Latency, 0);
        DstU->addDep(MemDep);
      }
    }

    // Add the schedule unit to visited map.
    VisitedMemOps.push_back(std::make_pair(DstMO, DstU));
  }
}

//===----------------------------------------------------------------------===//
template<typename DepEdgeTy>
void VPreRegAllocSched::addSchedDepForMI(MachineInstr *MI, int MIOffset,
                                         VSUnit *A, VSchedGraph &CurState,
                                         DepLatInfoTy &LatInfo) {
  assert(MI && "Unexpected entry root!");
  // Dirt
  MIOffset = std::min(MIOffset, 0);

  // FIXME: If several SrcMIs merged into a same SUnit, we may adding edges
  // from the same source.
  for (src_it I = LatInfo.begin(), E = LatInfo.end(); I != E; ++I) {
    MachineInstr *SrcMI = const_cast<MachineInstr*>(I->first);
    // Get the latency from SrcMI to MI.
    float DetailLatency = DetialLatencyInfo::getLatency(*I);
    int Latency = int(ceil(DetailLatency)) - MIOffset;

    assert(SrcMI && "Unexpected null SrcMI!");
    // LatencyInfo use a special marker to mark the current MI have some latency
    // from entry of the MBB.
    if (SrcMI == DetialLatencyInfo::EntryMarker) {
      assert(!A->isPHI() && "Unexpected datapath between PHI and entry node!");
      // Since there are some datapath between current schedule unit and the
      // entry node, we cannot schedule current schedule unit to the same slot
      // with the entry root.
      Latency = std::max(1, Latency);
      A->addDep(DepEdgeTy::CreateDep(CurState.getEntryRoot(), Latency));
      continue;
    }

    VSUnit *SrcSU = CurState.lookupSUnit(SrcMI);
    assert(SrcSU && "Src SUnit not found!");
    assert(SrcSU->isControl() && "Datapath dependence should be forwarded!");
    // Avoid the back-edge or self-edge.
    if (SrcSU->getIdx() >= A->getIdx()) continue;
    // Step between MI and its dependent.
    unsigned S = VInstrInfo::getCtrlStepBetween<DepEdgeTy::IsValDep>(SrcMI, MI);
    // Adjust the step between SrcMI and MI.
    Latency = std::max(int(S), Latency);
    // Call getLatencyTo to accumulate the intra-unit latency.
    Latency = SrcSU->getLatencyFrom(SrcMI, Latency);
    A->addDep(DepEdgeTy::CreateDep(SrcSU, Latency));
  }
}

void VPreRegAllocSched::addIncomingDepForPHI(VSUnit *PHISU, VSchedGraph &CurState){
  assert(PHISU->isPHI() && "Expect PHI in addIncomingDepForPHI!");
  MachineInstr *PN = PHISU->getRepresentativeInst();

  // Find the incoming copy.
  MachineInstr *IncomingCopy = PHISU->getInstrAt(1);
  assert(IncomingCopy->getOpcode() == VTM::VOpMvPhi && "Expect PHI move!");
  DepLatInfoTy *LatInfo = CurState.getDepLatInfo(IncomingCopy);
  assert(LatInfo && "Latency information for incoming copy not avaiable!");

  for (src_it I = LatInfo->begin(), E = LatInfo->end(); I != E; ++I) {
    MachineInstr *SrcMI = const_cast<MachineInstr*>(I->first);
    // Get the latency from SrcMI to MI.
    float DetailLatency = DetialLatencyInfo::getLatency(*I);
    int Latency = int(ceil(DetailLatency));

    assert(SrcMI && "Unexpected null SrcMI!");
    // Simply ignore the edge from entry, it is not an anti-dependence.
    if (SrcMI == DetialLatencyInfo::EntryMarker) continue;
    
    VSUnit *SrcSU = CurState.lookupSUnit(SrcMI);
    assert(SrcSU && "Src SUnit not found!");
    assert(SrcSU->isControl() && "Datapath dependence should be forwarded!");

    // Avoid self-edge.
    if (SrcSU == PHISU) continue;
    
    //assert(SrcSU->getIdx() > PHISU->getIdx() && "Expect back-edge!");

    // Adjust the step between SrcMI and MI.
    Latency = std::max(int(VInstrInfo::getCtrlStepBetween<false>(SrcMI, PN)),
                       Latency);
    // Call getLatencyTo to accumulate the intra-unit latency.
    Latency = SrcSU->getLatencyFrom(SrcMI, Latency);
    PHISU->addDep(VDMemDep::CreateDep<1>(SrcSU, Latency));
  }
}

void VPreRegAllocSched::addValDep(VSchedGraph &CurState, VSUnit *A) {
  typedef VSUnit::instr_iterator it;
  bool isCtrl = A->isControl();
  unsigned NumValDep = 0;

  for (unsigned I = 0, E = A->num_instrs(); I < E; ++I) {
    MachineInstr *MI = A->getInstrAt(I);
    int IntraSULatency = I ? A->getLatencyAt(I) : 0;

    assert(MI && "Unexpected entry root!");
    for (unsigned i = 0, e = MI->getNumOperands(); i != e; ++i) {
      MachineInstr *DepSrc = 0;
      const ucOperand &MO = cast<ucOperand>(MI->getOperand(i));
      VSUnit *Dep = getDefSU(MO, CurState, DepSrc);
      // Avoid self-edge
      if (Dep == 0 || Dep->getIdx() == A->getIdx()) continue;

      // Dirty Hack: Get the detail latency.
      float DetailLatency = VInstrInfo::getChainingLatency(DepSrc, MI);
      DetailLatency += VInstrInfo::getOperandLatency(MI, i);
      // Compute the latency from DepSrc to the repinst of the SU.
      DetailLatency -= std::min(0.0f, IntraSULatency - VInstrInfo::DeltaLatency);
      // All control operations are read at emit, wait until the datapath
      // operations finish if destination is control operation.
      int Latency = isCtrl ? ceil(DetailLatency) : floor(DetailLatency);
      Latency = Dep->getLatencyFrom(DepSrc, Latency);

      // If we got a back-edge, that should be a phinode.
      if (Dep->getIdx() > A->getIdx()) {
        assert(A->getRepresentativeInst()->isPHI()
               && "Expected backedge for PHI!");
        // Cross iteration dependences do not make sense in normal loops.
        if (CurState.isPipelined())
          // The iterate distance for back-edge to PHI is always 1.
          A->addDep(VDMemDep::CreateDep<1>(Dep, Latency));
      } else {
        A->addDep(VDValDep::CreateDep(Dep, Latency));
        ++NumValDep;
      }
    }
  }

  // If the atom depend on nothing and it must has some dependence edge,
  // make it depend on the entry node.
  if (NumValDep == 0) {
    int Latency =  A->getMaxLatencyFromEntry();

    A->addDep(VDCtrlDep::CreateDep(CurState.getEntryRoot(), Latency));
    return;
  }

  // For pipelined loop, take care of the Anti-dependence from PHI.
  if (CurState.isPipelined() && !isCtrl) {
    typedef VSUnit::dep_iterator it;
    for (it I = A->dep_begin(), E = A->dep_end(); I != E; ++I) {
      VSUnit *DepSU = *I;

      if (!DepSU->isPHI()) continue;

      // Data-path op must read the value from PHI before it is refreshed, so
      // add the back-edge to constraint the ALAP step of the operation.
      // Although it is possible to detect and handle this situation in
      // ScheduleEmitter, but that solution is more complex.
      DepSU->addDep(VDMemDep::CreateDep<1>(A, 0));
    }
  }
}

void VPreRegAllocSched::addSchedDepForSU(VSUnit *A, VSchedGraph &CurState,
                                         bool isExit) {
  // Build the dependence edge.
  typedef VSUnit::instr_iterator it;
  assert(A->isControl() && "Unexpected data-path schedule unit!");
  for (unsigned I = 0, E = A->num_instrs(); I != E; ++I) {
    MachineInstr *MI = A->getInstrAt(I);    
    assert(MI && "Unexpected entry root!");
    const DetialLatencyInfo::DepLatInfoTy *DepLat =
      CurState.getDepLatInfo(MI);
    assert(DepLat && "Operand latency information not available!");
    addSchedDepForMI<VDValDep>(MI, I ? A->getLatencyAt(I) : 0, A, CurState,
                               *DepLat);
  }

  // If the atom depend on nothing and it must has some dependence edge,
  // make it depend on the entry node.
  if (A->dep_empty() && !isExit) {
    int Latency = A->getMaxLatencyFromEntry();
    A->addDep(VDCtrlDep::CreateDep(CurState.getEntryRoot(), Latency));
  }
}

bool VPreRegAllocSched::couldBePipelined(const MachineBasicBlock *MBB) {
  MachineLoop *L = LI->getLoopFor(MBB);
  // Not in any loop.
  if (!L) return false;
  // Dirty Hack: Only support one block loop at this moment.
  if (L->getBlocks().size() != 1) return false;

  for (MachineBasicBlock::const_iterator I = MBB->begin(), E = MBB->end();
       I != E; ++I) {
    if (I->isPHI()) {
      for (unsigned i = 1, e = I->getNumOperands(); i < e; i +=2) {
        MachineBasicBlock *TargetBB = I->getOperand(i + 1).getMBB();
        if (TargetBB != MBB) continue;
        // Dirty Hack: PHI depends on PHI is not supported at the moment.
        if (MRI->getVRegDef(I->getOperand(i).getReg())->isPHI())
          return false;
      }
      continue;
    }

    // Do not pipeline the loops with call.
    if (I->getDesc().isCall()) return false;
  }

  return FInfo->getInfo().enablePipeLine();
}

void VPreRegAllocSched::buildPipeLineDepEdges(VSchedGraph &State) {
  MachineBasicBlock *CurBB = State.getMachineBasicBlock();
  VSUnit *LoopOp = State.getLoopOp();
  assert(LoopOp && "Not in loop?");
  assert(LoopOp != State.getExitRoot() && "Pipeline not enable!");

  for (instr_it I = CurBB->begin(), E = CurBB->end();I != E && I->isPHI(); ++I) {
    MachineInstr &PN = *I;
    VSUnit *PHISU = State.lookupSUnit(&PN);
    assert(PHISU && "Can not find SUnit for PHI!");

    // Add dependence from PHI incoming value:
    // PHI_incoming -(RAW dep)-> PHI_at_next_iteration.
    addIncomingDepForPHI(PHISU, State);
    // Add a anti-dependence edge from users of PHI to PHI because we must
    // have:
    // PHI -(RAW dep)-> PHI_user -(WAR dep)-> PHI_at_next_iteration.
    typedef VSUnit::use_iterator use_it;
    for (use_it UI = PHISU->use_begin(), UE = PHISU->use_end(); UI != UE; ++UI){
      VSUnit *PHIUser = *UI;
      if (PHIUser != State.getExitRoot())
        PHISU->addDep(getMemDepEdge(PHIUser, 0, 1));
    }

    // Add the dependence edge PHI -> Loop back -> PHI_at_iteration.
    PHISU->addDep(getMemDepEdge(LoopOp, 0, 1));
    //LoopOp->addDep(VDValDep::CreateValDep(PHISU, 0));
  }
}

bool VPreRegAllocSched::mergeUnaryOp(MachineInstr *MI, unsigned OpIdx,
                                     VSchedGraph &CurState) {
  MachineInstr *SrcMI = 0;
  // Try to merge it into the VSUnit that defining its source operand.
  if (VSUnit *SrcSU = getDefSU(MI->getOperand(OpIdx), CurState, SrcMI))
    return CurState.mapMI2SU(MI, SrcSU, SrcSU->getLatencyTo<true>(SrcMI, MI));

  // Try to merge it into the VSUnit that defining its predicate operand.
  if (const MachineOperand *Pred = VInstrInfo::getPredOperand(MI))
    if (VSUnit *SrcSU = getDefSU(*Pred, CurState, SrcMI))
      return CurState.mapMI2SU(MI, SrcSU, SrcSU->getLatencyTo<true>(SrcMI, MI));

  // Merge it into the EntryRoot.
  return CurState.mapMI2SU(MI, CurState.getEntryRoot(),
                           VInstrInfo::getStepsFromEntry(MI));
}

void VPreRegAllocSched::mergeDstMux(VSUnit * U, VSchedGraph &CurState) {
  // Look for the source value form distributed multiplexers.
  SmallVector<std::pair<MachineInstr*, unsigned>, 4> WorkStack;
  WorkStack.push_back(std::make_pair(U->getRepresentativeInst(), 0));

  while (!WorkStack.empty()) {
    MachineInstr *CurMI = WorkStack.back().first;
    unsigned Idx = WorkStack.back().second;

    // All child visited.
    if (Idx == CurMI->getNumOperands()) {
      WorkStack.pop_back();
      continue;
    }

    MachineOperand &MO = CurMI->getOperand(Idx);
    ++WorkStack.back().second;

    // They do defined by machine instructions.
    if (!MO.isReg() || MO.isDef() || MO.getReg() == 0) continue;

    MachineInstr *ChildMI = MRI->getVRegDef(MO.getReg());
    assert(ChildMI && "Reg not defined by any MI?");

    // We only merge Muxs.
    if (ChildMI->getOpcode() != VTM::VOpDstMux) continue;

    int Level = int(WorkStack.size());
    CurState.mapMI2SU(ChildMI, U, -Level);

    // The operand 0 is a define, simply ignore it.
    WorkStack.push_back(std::make_pair(ChildMI, 1));
  }
}

void VPreRegAllocSched::buildSUnit(MachineInstr *MI,  VSchedGraph &CurState) {
  assert(!MI->getDesc().isTerminator() && "Unexpected terminator!");
  bool isCmdSeq = false;
  switch (MI->getOpcode()) {
  default: break;
  case VTM::VOpMove:
    if (mergeUnaryOp(MI, 1, CurState))
      return;
    break;
  case VTM::VOpDisableFU: {
    MachineInstr *SrcMI = 0;
    // Try to merge it into the VSUnit that defining its source operand.
    VSUnit *SrcSU = getDefSU(MI->getOperand(0), CurState, SrcMI);
    assert(SrcSU && "Expected source schedule unit!");
    // Disable the FU at next state.
    bool merged = CurState.mapMI2SU(MI, SrcSU, 1);
    assert(merged && "DisableFU not merged?");
    (void) merged;
    return;
  }
  case VTM::PHI:
    // Merge the the PHI into entry root if the BB is not pipelined.
    if (!CurState.enablePipeLine()) {
      CurState.mapMI2SU(MI, CurState.getEntryRoot(),
                        VInstrInfo::getStepsFromEntry(MI));
      return;
    }
    break;
  case VTM::VOpReadReturn:
    if (mergeUnaryOp(MI, 1, CurState))
      return;
    break;
  case VTM::VOpMoveArg:
    CurState.mapMI2SU(MI, CurState.getEntryRoot(), 0);
    return;
  case VTM::VOpCmdSeq:
    isCmdSeq = true;
    // Merge the command sequence.
    if (!VInstrInfo::isCmdSeqBegin(MI)) {
      MachineInstr *PrevMI = LastCmdSeq->instr_back();
      if (VInstrInfo::isInSameCmdSeq(PrevMI, MI)) {
        VSUnit *PrevSU = CurState.lookupSUnit(PrevMI);
        VSUnit *NewSU =
          CurState.createVSUnit(MI, VInstrInfo::getPreboundFUId(MI).getFUNum());
        // Increase the latency
        NewSU->setLatency(PrevSU->getLatency() + 1);
        // There maybe some SU between PrevSU and NewSU, if we simply merge
        // current MI into PrevSU, we may read the result from a SU with bigger
        // index than PrevSU which is not acceptable.
        CurState.mergeSU(PrevSU, NewSU, /*DirtyHack*/1);
        return;
      }
    }
    break;
    // The VOpDstMux should be merged to its user.
    case VTM::VOpDstMux: return;
    case VTM::VOpMvPhi:
      if (CurState.isLoopPHIMove(MI)) {
        unsigned Reg = MI->getOperand(0).getReg();
        assert(MRI->hasOneUse(Reg) && "Incoming copy has more than one use?");
        MachineInstr *PN = &*MRI->use_begin(Reg);
        assert(PN && PN->isPHI() && "Bad user of incoming copy!");
        VSUnit *PHISU = CurState.lookupSUnit(PN);
        assert(PHISU && "Schedule unit for PHI node not found!");
        CurState.mapMI2SU(MI, PHISU, 0);
      }
    return;
  }

  // TODO: Remember the register that live out this MBB.
  // and the instruction that only produce a chain.
  FuncUnitId Id = VInstrInfo::getPreboundFUId(MI);
  VSUnit *U = CurState.createVSUnit(MI, Id.getFUNum());
  if (Id.isBound()) mergeDstMux(U, CurState);

  // Remember the new command sequence.
  if (isCmdSeq) LastCmdSeq = U;
}

template<int AllowHanging>
void VPreRegAllocSched::buildExitDeps(VSchedGraph &CurState) {
  typedef VSchedGraph::sched_iterator it;
  VSUnit *ExitRoot = CurState.getExitRoot();
  MachineInstr *ExitMI = ExitRoot->getRepresentativeInst();
  for (it I = CurState.sched_begin(), E = CurState.sched_end(); I != E; ++I) {
    VSUnit *VSU = *I;
    // Since the exit root already added to state sunit list, skip the
    // exit itself.
    if (VSU->getNumUses() == 0 && VSU != ExitRoot) {
      assert((AllowHanging || CurState.isLoopOp(VSU->getRepresentativeInst()))
             && "Unexpected handing node!");
      // A PHIMove can be scheduled to the same slot with the exit root.
      unsigned Latency = VSU->getMaxLatencyTo<false>(ExitMI);
      // We do not need to wait the trivial operation finish before exiting the
      // state, because the first control slot of next state will only contains
      // PHI copies, and the PHIElimination Hook will take care of the data
      // dependence and try to forward the wire value in last control slot
      // if possible, so they can take the time of the last control slot.
      //VIDesc VID(*Instr);
      //if (VID.hasTrivialFU() && !VID.hasDatapath() && Latency)
      //  --Latency;

      ExitRoot->addDep(VDCtrlDep::CreateDep(VSU,  Latency));
    }
  }
}

void VPreRegAllocSched::buildExitRoot(VSchedGraph &CurState,
                                      MachineInstr *FirstTerminator) {
  SmallVector<MachineInstr*, 8> Exits;
  // We need wait all operation finish before the exit operation active, compute
  // the latency from operations need to wait to the exit operation.
  DetialLatencyInfo::DepLatInfoTy ExitDepInfo;
  VSUnit *ExitSU = 0;

  for (instr_it I = FirstTerminator, E = CurState->end(); I != E; ++I) {
    MachineInstr *MI = I;
    if (!I->isTerminator()) {
      assert(MI->getOpcode() == VTM::VOpMvPhi && "Bad MBB!");
      continue;
    }

    // Build the schedule unit for loop back operation.
    if (CurState.isLoopOp(I)) {
      CurState.addToLatInfo(MI);
      VSUnit *LoopOp = CurState.createVSUnit(MI);
      addSchedDepForSU(LoopOp, CurState);
      continue;
    }
  }
  
  for (instr_it I = FirstTerminator, E = CurState->end(); I != E; ++I) {
    MachineInstr *MI = I;
    if (!I->isTerminator() || CurState.isLoopOp(I)) continue;
    
    // Build a exit root or merge the terminators into the exit root.
    if (ExitSU == 0) {
      ExitSU = CurState.createVSUnit(MI);
      CurState.setExitRoot(ExitSU);
    } else {
      bool mapped = CurState.mapMI2SU(MI, ExitSU, 0);
      (void) mapped;
      assert(mapped && "Cannot merge terminators!");
    }
  }

  assert(ExitSU && "Terminator not found?");

  for (instr_it I = FirstTerminator, E = CurState->end(); I != E; ++I) {
    MachineInstr *MI = I;

    if (MI->getOpcode() == VTM::VOpMvPhi) {
      if (CurState.isLoopPHIMove(MI)) {
        // Forward the edges to exit root, so the dependences of PHI moves
        // can always finish in time.
        const DetialLatencyInfo::DepLatInfoTy *DepLat =
          CurState.getDepLatInfo(MI);
        assert(DepLat && "Operand latency information not available!");
        addSchedDepForMI<VDCtrlDep>(MI, 0/*Offset*/, ExitSU, CurState, *DepLat);
        // Add the dependence from PHISU to ExitSU, we will constraint the PHI
        // so it will schedule before the last stage of a pipeline BB.
        VSUnit *PHISU = CurState.lookupSUnit(MI);
        assert(PHISU->isPHI() && "Expect PHISU merged by PHI!");
        ExitSU->addDep(VDCtrlDep::CreateDep(PHISU, 0));
        continue;
      } else { // Also merge the PHI moves.
        bool mapped = CurState.mapMI2SU(MI, ExitSU, 0);
        (void) mapped;
        assert(mapped && "Cannot merge terminators!");
      }
    } else if (CurState.isLoopOp(I))
      continue;

    // Compute the dependence information.
    CurState.addToLatInfo(MI);
    // No need to wait the terminator.
    CurState.eraseFromExitSet(MI);
    // Build datapath latency information for the terminator.
    CurState.buildExitMIInfo(MI, ExitDepInfo);
  }

  // Add the dependence of exit root.
  addSchedDepForSU(ExitSU, CurState, true);

  // Add the control dependence edge edges to wait all operation finish.
  addSchedDepForMI<VDCtrlDep>(ExitSU->getRepresentativeInst(), 0/*Offset*/,
                              ExitSU, CurState, ExitDepInfo);

  // If we have a trivial schedule graph that only containing entry and exit
  // simply connect them together.
  VSUnit *Entry = CurState.getEntryRoot();
  if (Entry->use_empty())
    ExitSU->addDep(VDCtrlDep::CreateDep(Entry, 1));

  // Sort the schedule units after all units are built.
  CurState.prepareForCtrlSched();

  // If there is still schedule unit not connect to exit, connect it now, but
  // they are supposed to be connected in the previous stages, so hanging node
  // is not allow.
  buildExitDeps<false>(CurState);
}

void VPreRegAllocSched::buildControlPathGraph(VSchedGraph &State) {
  instr_it BI = State->begin();
  while(!BI->isTerminator() && BI->getOpcode() != VTM::VOpMvPhi) {
    MachineInstr *MI = BI;    
    State.addToLatInfo(MI);
    buildSUnit(MI, State);
    ++BI;
  }
  State.removeDeadSU();

  // Make sure every VSUnit have a dependence edge except EntryRoot.
  typedef VSchedGraph::iterator it;
  for (it I = State.begin() + 1, E = State.end(); I != E; ++I)
    if ((*I)->isControl()) addSchedDepForSU(*I, State);

  // Merge the loop PHI moves into the PHI Node, after the intra iteration
  // dependence edge added. If we merge the PHI moves into the PHI schedule
  // units before we adding intra iteration edges, the dependences of the PHI
  // moves are added to the PHI schedule unit as intra iteration dependences,
  // which is incorrect, all dependences of a PHI should be inter iteration
  // dependences.
  for (instr_it I = BI; !I->isTerminator(); ++I) {
    if (!State.isLoopPHIMove(I))
      continue;

    State.addToLatInfo(I);
    buildSUnit(I, State);
  }

  // Create the exit node, now BI points to the first terminator.
  buildExitRoot(State, BI);

  // Build loop edges if necessary.
  if (State.enablePipeLine())
    buildPipeLineDepEdges(State);

  // Build the memory edges.
  buildMemDepEdges(State);

  // Verify the schedule graph.
  State.verify();
}

void VPreRegAllocSched::buildDataPathGraph(VSchedGraph &State) {
  State.prepareForDatapathSched();
  for (su_it I = State.begin() + 1, E = State.end(); I != E; ++I) {
    VSUnit *U = *I;
    addValDep(State, U);
  }

  // Build control dependence for exitroot, hanging node is allowed because we
  // do not handle them explicitly.
  buildExitDeps<true>(State);

  // Verify the schedule graph.
  State.verify();
}

void VPreRegAllocSched::cleanUpSchedule() {
  for (unsigned i = 0, e = MRI->getNumVirtRegs(); i != e; ++i) {
     unsigned RegNum = TargetRegisterInfo::index2VirtReg(i);
    cleanUpRegisterClass(RegNum, VTM::DRRegisterClass);
  }
}

bool VPreRegAllocSched::cleanUpRegisterClass(unsigned RegNum,
                                             const TargetRegisterClass *RC) {
  // And Emit the wires defined in this module.
  if (MRI->getRegClass(RegNum) != RC) return false;

  MachineRegisterInfo::def_iterator DI = MRI->def_begin(RegNum);

  if (DI == MRI->def_end() || !MRI->use_empty(RegNum))
    return false;

  // Skip the bundle instruction.
  assert(llvm::next(MRI->def_begin(RegNum)) ==  MRI->def_end() &&"Not in SSA!");

  MachineInstr &DefMI = *DI;

  if (DefMI.isPHI()) {
    // The instruction is dead.
    DefMI.eraseFromParent();
    return true;
  }

  // Preserve the read fu information, and keep reading the source fu register
  if (DefMI.getOpcode() == VTM::VOpReadFU)    
    DI.getOperand().ChangeToRegister(0, false);
  else {
    // FIXME: Remove the PHI, and incoming copies (Bug 14).
    if (DefMI.getOpcode() == VTM::VOpDefPhi) return false;

    assert(0 && "Unexpected dead define!");
    DefMI.eraseFromParent();
  }

  return true;
}
