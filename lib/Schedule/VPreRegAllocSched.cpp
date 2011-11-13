//===------------ VSUnit.cpp - Translate LLVM IR to VSUnit  -----*- C++ -*-===//
//
//                            The Verilog Backend
//
// Copyright: 2010 by Hongbin Zheng. all rights reserved.
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
#include "vtm/VTM.h"

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
                        DetialLatencyInfo &LatInfo,
                        bool isExit = false);
  typedef const DetialLatencyInfo::DepLatInfoTy DepLatInfoTy;
  template<int IgnoreBackedge, typename CreateDepFuncTy>
  void addSchedDepForMI(MachineInstr *MI, VSUnit *A, VSchedGraph &CurState,
                        DepLatInfoTy &LatInfo, CreateDepFuncTy &CreateDepFunc);

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

  void buildPipeLineDepEdges(VSchedGraph &State, DetialLatencyInfo &LatInfo);

  typedef MachineBasicBlock::iterator instr_it;
  void buildExitRoot(VSchedGraph &CurState, MachineInstr *FirstTerminator,
                     DetialLatencyInfo &LatInfo);

  template<int AllowHanging>
  void buildExitDeps( VSchedGraph &CurState );

  void buildSUnit(MachineInstr *MI, VSchedGraph &CurState);

  bool mergeUnaryOp(MachineInstr *MI, unsigned OpIdx, VSchedGraph &CurState);

  bool mergeBitCat(MachineInstr *MI, VSchedGraph &CurState);
  bool canMergeBitCat(MachineInstr *SrcMI, VSUnit *SrcSU) const;

  /// @name FunctionPass interface
  //{
  static char ID;

  ~VPreRegAllocSched();
  bool runOnMachineFunction(MachineFunction &MF);

  // Remove redundant code after schedule emitted.
  void cleanUpSchedule();
  void cleanUpRegisterClass(const TargetRegisterClass *RC);
  void fixCmpFUPort();
  void fixSubModuleReturnPort();

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
  const TargetMachine &TM = MF.getTarget();
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
    VSchedGraph State(TM, MBB, couldBePipelined(MBB), getTotalCycle());
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
  const Type *SrcElTy = cast<PointerType>(SrcAddr->getType())->getElementType();
  if (SrcElTy->isSized()) SrcSize = AA->getTypeStoreSize(SrcElTy);

  uint64_t DstSize = AliasAnalysis::UnknownSize;
  const Type *DstElTy = cast<PointerType>(DstAddr->getType())->getElementType();
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

static inline bool mayAccessMemory(const TargetInstrDesc &TID) {
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

      DstMO = const_cast<Value*>((*DstMI->memoperands_begin())->getValue());
      const Type *DstElemTy
        = cast<SequentialType>(DstMO->getType())->getElementType();
      DstSize = TD->getTypeStoreSize(DstElemTy);
      assert(!isa<PseudoSourceValue>(DstMO) && "Unexpected frame stuffs!");
    }

    for (MemOpMapTy::iterator I = VisitedMemOps.begin(), E = VisitedMemOps.end();
         I != E; ++I) {
      Value *SrcMO = I->first;
      VSUnit *SrcU = I->second;

      MachineInstr *SrcMI = SrcU->getRepresentativeInst();

      // Handle unanalyzable memory access.
      if (DstMO == 0 || SrcMO == 0) {
        // Build the Src -> Dst dependence.
        unsigned Latency = VInstrInfo::getCtrlStepBetween(SrcMI, DstMI);
        DstU->addDep(getMemDepEdge(SrcU, Latency, 0));

        // Build the Dst -> Src (in next iteration) dependence.
        if (CurState.enablePipeLine()) {
          Latency = VInstrInfo::getCtrlStepBetween(SrcMI, DstMI);
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
          unsigned Latency = VInstrInfo::getCtrlStepBetween(SrcMI, DstMI);
          VDMemDep *MemDep = getMemDepEdge(SrcU, Latency, LD.getItDst());
          DstU->addDep(MemDep);
        }

        // We need to compute if Src depend on Dst even if Dst not depend on Src.
        // Because dependence depends on execute order.
        LD = analyzeLoopDep(DstMO, SrcMO, isDstLoad, isSrcLoad, *IRL, false);

        if (LD.hasDep()) {
          unsigned Latency = VInstrInfo::getCtrlStepBetween(SrcMI, DstMI);
          VDMemDep *MemDep = getMemDepEdge(DstU, Latency, LD.getItDst());
          SrcU->addDep(MemDep);
        }
      } else {
        const Type *SrcElemTy
          = cast<SequentialType>(SrcMO->getType())->getElementType();
        size_t SrcSize = TD->getTypeStoreSize(SrcElemTy);

        if (AA->isNoAlias(SrcMO, SrcSize, DstMO, DstSize)) continue;

        // Ignore the No-Alias pointers.
        unsigned Latency = VInstrInfo::getCtrlStepBetween(SrcMI, DstMI);
        VDMemDep *MemDep = getMemDepEdge(SrcU, Latency, 0);
        DstU->addDep(MemDep);
      }
    }

    // Add the schedule unit to visited map.
    VisitedMemOps.push_back(std::make_pair(DstMO, DstU));
  }
}

//===----------------------------------------------------------------------===//
template<int IgnoreBackedge, typename CreateDepFuncTy>
void VPreRegAllocSched::addSchedDepForMI(MachineInstr *MI, VSUnit *A,
                                         VSchedGraph &CurState,
                                         DepLatInfoTy &LatInfo,
                                         CreateDepFuncTy &CreateDepFunc) {
  assert(MI && "Unexpected entry root!");
  // FIXME: If several SrcMIs merged into a same SUnit, we may adding edges
  // from the same source.
  for (src_it I = LatInfo.begin(), E = LatInfo.end(); I != E; ++I) {
    MachineInstr *SrcMI = const_cast<MachineInstr*>(I->first);
    // Get the latency from SrcMI to MI.
    double DetailLatency = I->second;
    unsigned Latency = unsigned(ceil(DetailLatency));
    assert(SrcMI && "Unexpected null SrcMI!");
    // LatencyInfo use a special marker to mark the current MI have some latency
    // from entry of the MBB.
    if (SrcMI == DetialLatencyInfo::EntryMarker) {
      assert(IgnoreBackedge && "Unexpected datapath between PHI and entry node!");
      // Since there are some datapath between current schedule unit and the
      // entry node, we cannot schedule current schedule unit to the same slot
      // with the entry root.
      Latency = std::max(1u, Latency);
      A->addDep(CreateDepFunc(CurState.getEntryRoot(), Latency));
      continue;
    }

    VSUnit *SrcSU = CurState.lookupSUnit(SrcMI);
    assert(SrcSU && "Src SUnit not found!");
    assert(SrcSU->isControl() && "Datapath dependence should be forwarded!");
    // Avoid the back-edge or self-edge.
    if ((SrcSU->getIdx() > A->getIdx() && IgnoreBackedge) || SrcSU == A)
      continue;
    // Adjust the step between SrcMI and MI.
    Latency = std::max(Latency, VInstrInfo::getCtrlStepBetween(SrcMI, MI));
    // Call getLatencyTo to accumulate the intra-unit latency.
    Latency = SrcSU->getLatencyFrom(SrcMI, Latency);
    A->addDep(CreateDepFunc(SrcSU, Latency));
  }
}

void VPreRegAllocSched::addValDep(VSchedGraph &CurState, VSUnit *A) {
  typedef VSUnit::instr_iterator it;
  for (it I = A->instr_begin(), E = A->instr_end(); I != E; ++I) {
    MachineInstr *MI = *I;
    assert(MI && "Unexpected entry root!");
    for (unsigned i = 0, e = MI->getNumOperands(); i != e; ++i) {
      MachineInstr *DepSrc = 0;
      VSUnit *Dep = getDefSU(MI->getOperand(i), CurState, DepSrc);
      // Avoid self-edge
      if (Dep == 0 || Dep->getIdx() == A->getIdx()) continue;

      // Dirty Hack: Call get Detail latency.
      double DetailLatency = VInstrInfo::getChainingLatency(DepSrc, MI);
      unsigned Latency = floor(DetailLatency);
      Latency = Dep->getLatencyFrom(DepSrc, Latency);

      // Build the dependence edge.
      VDEdge *Edge = 0;
      // We got a back-edge, that should be a phi.
      if (Dep->getIdx() > A->getIdx()) {
        assert(A->getRepresentativeInst()->isPHI()
               && "Expected backedge for PHI!");
        // The iterate distance for backedge to PHI is always 1.
        Edge = VDMemDep::CreateMemDep<1>(Dep, Latency);
      } else
        Edge = VDValDep::CreateValDep(Dep, Latency);

      A->addDep(Edge);
    }
  }

  // If the atom depend on nothing and it must has some dependence edge,
  // make it depend on the entry node.
  if (A->dep_empty()) {
    unsigned Latency = VInstrInfo::getStepsFromEntry(A->getRepresentativeInst());
    A->addDep(VDCtrlDep::CreateCtrlDep(CurState.getEntryRoot(), Latency));
  }
}

void VPreRegAllocSched::addSchedDepForSU(VSUnit *A, VSchedGraph &CurState,
                                         DetialLatencyInfo &LatInfo,
                                         bool isExit) {
  // Build the dependence edge.
  typedef VSUnit::instr_iterator it;
  assert(A->isControl() && "Unexpected datapath schedule unit!");
  for (it I = A->instr_begin(), E = A->instr_end(); I != E; ++I) {
    MachineInstr *MI = *I;
    assert(MI && "Unexpected entry root!");
    const DetialLatencyInfo::DepLatInfoTy *DepLat =
      LatInfo.getDepLatInfo(MI);
    assert(DepLat && "Operand latency information not available!");
    addSchedDepForMI<true>(MI, A, CurState, *DepLat, VDValDep::CreateValDep);
  }

  // If the atom depend on nothing and it must has some dependence edge,
  // make it depend on the entry node.
  if (A->dep_empty() && !isExit) {
    unsigned Latency = VInstrInfo::getStepsFromEntry(A->getRepresentativeInst());
    A->addDep(VDCtrlDep::CreateCtrlDep(CurState.getEntryRoot(), Latency));
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

void VPreRegAllocSched::buildPipeLineDepEdges(VSchedGraph &State,
                                              DetialLatencyInfo &LatInfo) {
  MachineBasicBlock *CurBB = State.getMachineBasicBlock();
  VSUnit *LoopOp = State.getLoopOp();
  assert(LoopOp && "Not in loop?");
  assert(LoopOp != State.getExitRoot() && "Pipeline not enable!");

  for (instr_it I = CurBB->begin(), E = CurBB->end();I != E && I->isPHI(); ++I) {
    MachineInstr &PN = *I;
    const DetialLatencyInfo::DepLatInfoTy &DepLatInfo =
      LatInfo.buildPHIBELatInfo(&PN);

    VSUnit *PhiSU = State.lookupSUnit(&PN);
    assert(PhiSU && "Can not find SUnit for PHI!");

    // Add a anti-dependence edge from users of PHI to PHI because we must
    // have:
    // PHI -(RAW dep)-> PHI_user -(WAR dep)-> PHI_at_next_iteration.
    addSchedDepForMI<false>(&PN, PhiSU, State, DepLatInfo,
                            VDMemDep::CreateMemDep<1>);

    // Dirty Hack: We can emit the PHI while looping back to the loop entry.
    unsigned Latency = 0;
    //  LoopOp->getLatencyTo(LoopOp->getRepresentativeInst(), &PN);
    PhiSU->addDep(getMemDepEdge(LoopOp, Latency, 1));
  }
}

bool VPreRegAllocSched::mergeUnaryOp(MachineInstr *MI, unsigned OpIdx,
                                     VSchedGraph &CurState) {
  MachineInstr *SrcMI = 0;
  // Try to merge it into the VSUnit that defining its source operand.
  if (VSUnit *SrcSU = getDefSU(MI->getOperand(OpIdx), CurState, SrcMI))
    return CurState.mapMI2SU(MI, SrcSU, SrcSU->getLatencyTo(SrcMI, MI));

  // Try to merge it into the VSUnit that defining its predicate operand.
  if (const MachineOperand *Pred = VInstrInfo::getPredOperand(MI))
    if (VSUnit *SrcSU = getDefSU(*Pred, CurState, SrcMI))
      return CurState.mapMI2SU(MI, SrcSU, SrcSU->getLatencyTo(SrcMI, MI));

  // Merge it into the EntryRoot.
  return CurState.mapMI2SU(MI, CurState.getEntryRoot(),
                           VInstrInfo::getStepsFromEntry(MI));
}

bool VPreRegAllocSched::canMergeBitCat(MachineInstr *SrcMI, VSUnit *SrcSU)const{
  if (!SrcSU->isRepresentativeInst(SrcMI)) return false;

  if (SrcMI->getOpcode() != VTM::VOpBitCat) return false;

  // Becareful of such graph:
  //     bitcat
  //      |  \
  //      |   Op
  //      |  /
  //     bitcat
  //
  // In this case, the two bitcat cannot merge.
  if (!MRI->hasOneNonDBGUse(SrcMI->getOperand(0).getReg())) return false;

  return true;
}

bool VPreRegAllocSched::mergeBitCat(MachineInstr *MI, VSchedGraph &CurState) {
  MachineInstr *LHSMI = 0, *RHSMI = 0;
  VSUnit *LHSSU = getDefSU(MI->getOperand(1), CurState, LHSMI),
         *RHSSU = getDefSU(MI->getOperand(2), CurState, RHSMI);

  // Sources are merged?
  if (LHSSU == RHSSU) {
    // Concatting two symbol?
    if (LHSSU == 0) {
      LHSSU = RHSSU = CurState.getEntryRoot();
      LHSMI = RHSMI = 0;
    }

    unsigned Latency = std::max(LHSSU->getLatencyTo(LHSMI, MI),
                                RHSSU->getLatencyTo(RHSMI, MI));
    CurState.mapMI2SU(MI, LHSSU, Latency);
    return true;
  }

  // Only have 1 valid source?
  if (LHSSU == 0) {
    std::swap(LHSSU, RHSSU);
    std::swap(LHSMI, RHSMI);
  }

  if (RHSSU == 0) {
    CurState.mapMI2SU(MI, LHSSU, LHSSU->getLatencyTo(LHSMI, MI));
    return true;
  }

  bool LHSMerged = false;
  if (canMergeBitCat(LHSMI, LHSSU)) {
    CurState.mapMI2SU(MI, LHSSU, LHSSU->getLatencyTo(LHSMI, MI));
    LHSMerged = true;
  }

  if (canMergeBitCat(RHSMI, RHSSU)) {
    if (!LHSMerged) {
      CurState.mapMI2SU(MI, RHSSU, RHSSU->getLatencyTo(RHSMI, MI));
      return true;
    }

    CurState.mergeSU(RHSSU, LHSSU, 0);
    return true;
  }

  return LHSMerged;
}

void VPreRegAllocSched::buildSUnit(MachineInstr *MI,  VSchedGraph &CurState) {
  assert(!MI->getDesc().isTerminator() && "Unexpected terminator!");
  bool isCmdSeq = false;
  switch (MI->getOpcode()) {
  default: break;
  case VTM::VOpMove_ri:
  case VTM::VOpMove_rw:
  case VTM::VOpMove_rr:
    if (mergeUnaryOp(MI, 1, CurState))
      return;
    break;
  case VTM::VOpReadReturn:
    if (mergeUnaryOp(MI, 1, CurState))
      return;
    break;
  case VTM::VOpCmdSeq:
    isCmdSeq = true;
    // Merge the command sequence.
    if (!VInstrInfo::isCmdSeqBegin(MI)) {
      MachineInstr *PrevMI = LastCmdSeq->instr_back();
      if (VInstrInfo::isInSameCmdSeq(PrevMI, MI)) {
        VSUnit *U = CurState.lookupSUnit(PrevMI);
        CurState.mapMI2SU(MI, U, /*DirtyHack*/1);
        // Increase the latency
        U->setLatency(U->getLatency() + 1);
        return;
      }
    }
    break;
  }

  FuncUnitId Id = VInstrInfo::getPrebindFUId(MI);

  // TODO: Remember the register that live out this MBB.
  // and the instruction that only produce a chain.
  VSUnit *U = CurState.createVSUnit(MI, Id.getFUNum());
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
      assert(AllowHanging && "Unexpected handing node!");
      // Dirty Hack.
      unsigned Latency = VSU->getMaxLatencyTo(ExitMI);
      // We do not need to wait the trivial operation finish before exiting the
      // state, because the first control slot of next state will only contains
      // PHI copies, and the PHIElimination Hook will take care of the data
      // dependence and try to forward the wire value in last control slot
      // if possible, so they can take the time of the last control slot.
      //VIDesc VID(*Instr);
      //if (VID.hasTrivialFU() && !VID.hasDatapath() && Latency)
      //  --Latency;

      ExitRoot->addDep(VDCtrlDep::CreateCtrlDep(VSU,  Latency));
    }
  }
}

void VPreRegAllocSched::buildExitRoot(VSchedGraph &CurState,
                                      MachineInstr *FirstTerminator,
                                      DetialLatencyInfo &LatInfo) {
  SmallVector<MachineInstr*, 8> Exits;
  // We need wait all operation finish before the exit operation active, compute
  // the latency from operations need to wait to the exit operation.
  DetialLatencyInfo::DepLatInfoTy ExitDepInfo;

  for (instr_it I = FirstTerminator, E = CurState->end(); I !=E; ++I) {
    MachineInstr *MI = I;
    assert(MI->getDesc().isTerminator() && "Unexpected non-terminator!");
    // Build the dependence latency information for the terminator.
    LatInfo.addInstr(MI);
    // Try to build the schedule unit for the loop back operation.
    if (CurState.isLoopOp(I)) {
      VSUnit *LoopOp = CurState.createVSUnit(MI);
      addSchedDepForSU(LoopOp, CurState, LatInfo);
      continue;
    }

    // No need to wait the terminator.
    LatInfo.eraseFromExitSet(MI);
    Exits.push_back(MI);
  }

  assert(!Exits.empty() && "Not get any terminator exiting the block?");
  MachineInstr *FstExit = Exits.front();
  VSUnit *ExitSU = CurState.createVSUnit(FstExit);
  CurState.setExitRoot(ExitSU);
  // Build datapath latency information for the terminator.
  LatInfo.buildExitMIInfo(FstExit, ExitDepInfo);

  // Add others terminator to the exit node.
  while (Exits.size() != 1) {
    MachineInstr *ExitMI = Exits.pop_back_val();
    LatInfo.buildExitMIInfo(ExitMI, ExitDepInfo);
    bool mapped = CurState.mapMI2SU(ExitMI, ExitSU, 0);
    (void) mapped;
    assert(mapped && "Cannot merge terminators!");
  }

  // Add the dependence of exit root.
  addSchedDepForSU(ExitSU, CurState, LatInfo, true);

  // Add the control dependence edge edges to wait all operation finish.
  addSchedDepForMI<true>(FstExit, ExitSU, CurState, ExitDepInfo,
                         VDCtrlDep::CreateCtrlDep);

  // If we have a trivial schedule graph that only containing entry and exit
  // simply connect them together.
  VSUnit *Entry = CurState.getEntryRoot();
  if (Entry->use_empty())
    ExitSU->addDep(VDCtrlDep::CreateCtrlDep(Entry, 1));

  // If there is still schedule unit not connect to exit, connect it now, but
  // they are supposed to be connected in the previous stages, so hanging node
  // is not allow.
  buildExitDeps<false>(CurState);
}

void VPreRegAllocSched::buildControlPathGraph(VSchedGraph &State) {
  DetialLatencyInfo DetialLat(*MRI);
  instr_it BI = State->begin();
  while(!BI->getDesc().isTerminator()) {
    DetialLat.addInstr(BI);
    buildSUnit(&*BI, State);
    ++BI;
  }

  State.removeDeadSU();

  // Make sure every VSUnit have a dependence edge except EntryRoot.
  typedef VSchedGraph::iterator it;
  for (it I = State.begin() + 1, E = State.end(); I != E; ++I)
    if ((*I)->isControl()) addSchedDepForSU(*I, State, DetialLat);

  // Create the exit node, now BI points to the first terminator.
  buildExitRoot(State, BI, DetialLat);
  State.prepareForCtrlSched();

  // Build loop edges if necessary.
  if (State.enablePipeLine())
    buildPipeLineDepEdges(State, DetialLat);

  // Build the memory edges.
  buildMemDepEdges(State);
}

void VPreRegAllocSched::buildDataPathGraph(VSchedGraph &State) {
  State.prepareForDatapathSched();
  // Refresh the dependence edges and build data dependence.
  State.getEntryRoot()->cleanDeps();
  for (su_it I = State.begin() + 1, E = State.end(); I != E; ++I) {
    VSUnit *U = *I;
    if (U->isControl()) U->cleanDeps();
    addValDep(State, U);
  }

  // Build control dependence for exitroot, hanging node is allowed because we
  // do not handle them explicitly.
  buildExitDeps<true>(State);
}

void VPreRegAllocSched::cleanUpSchedule() {
  cleanUpRegisterClass(VTM::DRRegisterClass);
  fixCmpFUPort();
  fixSubModuleReturnPort();
}

void VPreRegAllocSched::cleanUpRegisterClass(const TargetRegisterClass *RC) {
  // And Emit the wires defined in this module.
  const std::vector<unsigned>& Wires = MRI->getRegClassVirtRegs(RC);

  for (std::vector<unsigned>::const_iterator I = Wires.begin(), E = Wires.end();
       I != E; ++I) {
    unsigned SrcReg = *I;
    MachineRegisterInfo::def_iterator DI = MRI->def_begin(SrcReg);

    if (DI == MRI->def_end() || !MRI->use_empty(SrcReg))
      continue;

    MachineInstr &DefMI = *DI;
    if (DefMI.isPHI()) {
      // The instruction is dead.
      DefMI.removeFromParent();
      continue;
    }

    assert(++MRI->def_begin(SrcReg) == MRI->def_end() && "Not in SSA From!");
    // Do not remove the operand, just change it to implicit define.
    ucOp Op = ucOp::getParent(DI);
    SrcReg = TargetRegisterInfo::virtReg2Index(SrcReg);

    // Preserve the read fu information, and keep reading the source fu register
    if (Op->getOpcode() == VTM::VOpReadFU) {
      MachineOperand &MO = DI.getOperand();
      MO.ChangeToImmediate(SrcReg);
      MO.setTargetFlags(64);
    } else {
      Op.changeOpcode(VTM::IMPLICIT_DEF, Op->getPredSlot());
      for (ucOp::op_iterator OI = Op.op_begin(), OE = Op.op_end();OI != OE;++OI){
        // Change the operand to some rubbish value.
        MachineOperand &MO = *OI;
        MO.ChangeToImmediate(SrcReg);
        MO.setTargetFlags(64);
      }
    }
  }
}

static void addSubRegIdx(unsigned Reg, unsigned SubReg,
                         MachineRegisterInfo *MRI) {
  typedef MachineRegisterInfo::use_iterator use_it;

  for (use_it I = MRI->use_begin(Reg), E = MRI->use_end(); I != E; ++I)
    I.getOperand().setSubReg(SubReg);
}

void VPreRegAllocSched::fixCmpFUPort() {
  // And Emit the wires defined in this module.
  const std::vector<unsigned>& Cmps =
    MRI->getRegClassVirtRegs(VTM::RUCMPRegisterClass);

  for (std::vector<unsigned>::const_iterator I = Cmps.begin(), E = Cmps.end();
       I != E; ++I) {
    unsigned SrcReg = *I;
    MachineRegisterInfo::def_iterator DI = MRI->def_begin(SrcReg);

    if (DI == MRI->def_end() || MRI->use_empty(SrcReg))
      continue;

    assert(!DI->isPHI() && "PHI with RUCMPRegister is not supported!");

    assert(++MRI->def_begin(SrcReg) == MRI->def_end() && "Not in SSA From!");
    // Do not remove the operand, just change it to implicit define.
    ucOp Op = ucOp::getParent(DI);
    if (Op->isOpcode(VTM::VOpICmp)) {
      unsigned SubRegIdx = VFUs::getICmpPort(Op.getOperand(3).getImm());
      addSubRegIdx(SrcReg, SubRegIdx, MRI);
      continue;
    }

    llvm_unreachable("Unsupported opcode!");
  }
}

static void addSubRegIdxForCalleeFN(unsigned Reg, MachineRegisterInfo *MRI) {
  typedef MachineRegisterInfo::use_iterator use_it;

  for (use_it I = MRI->use_begin(Reg), E = MRI->use_end(); I != E; ++I) {
    ucOp User = ucOp::getParent(I);
    if (User->isOpcode(VTM::VOpReadFU)) continue;

    assert(User->isOpcode(VTM::VOpReadReturn) && "Unexpected callee user!");

    unsigned ReturnPortIdx = User.getOperand(2).getImm();
    I.getOperand().setSubReg(ReturnPortIdx);
  }
}

void VPreRegAllocSched::fixSubModuleReturnPort() {
  // And Emit the wires defined in this module.
  const std::vector<unsigned>& Callees =
    MRI->getRegClassVirtRegs(VTM::RCFNRegisterClass);

  for (std::vector<unsigned>::const_iterator I = Callees.begin(),
       E = Callees.end(); I != E; ++I) {
      unsigned SrcReg = *I;
      MachineRegisterInfo::def_iterator DI = MRI->def_begin(SrcReg);

      if (DI == MRI->def_end() || MRI->use_empty(SrcReg))
        continue;

      assert(!DI->isPHI() && "PHI with RUCMPRegister is not supported!");

      assert(++MRI->def_begin(SrcReg) == MRI->def_end() && "Not in SSA From!");
      // Do not remove the operand, just change it to implicit define.
      ucOp Op = ucOp::getParent(DI);
      if (Op->isOpcode(VTM::VOpInternalCall)) {
        addSubRegIdxForCalleeFN(SrcReg, MRI);
        continue;
      }

      llvm_unreachable("Unsupported opcode!");
  }
}
