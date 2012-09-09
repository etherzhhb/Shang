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
#include "vtm/Utilities.h"
#include "vtm/Passes.h"
#include "vtm/VFInfo.h"
#include "vtm/VerilogBackendMCTargetDesc.h"

#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/Analysis/ScalarEvolution.h"
#include "llvm/Analysis/ScalarEvolutionExpressions.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/MachineLoopInfo.h"
#include "llvm/CodeGen/PseudoSourceValue.h"
#include "llvm/CodeGen/MachineBlockFrequencyInfo.h"
#include "llvm/CodeGen/MachineBranchProbabilityInfo.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/ADT/PostOrderIterator.h"
#include "llvm/ADT/DepthFirstIterator.h"
#include "llvm/Support/Allocator.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/GraphWriter.h"
#include "llvm/Support/MathExtras.h"
#define DEBUG_TYPE "vtm-sgraph"
#include "llvm/Support/Debug.h"
using namespace llvm;

STATISTIC(MutexPredNoAlias, "Number of no-alias because of mutex predicate");
//===----------------------------------------------------------------------===//
namespace {
/// @brief Schedule the operations.
///
struct VPreRegAllocSched : public MachineFunctionPass {
  const TargetInstrInfo *TII;
  // The loop Info
  MachineRegisterInfo *MRI;
  VFInfo *FInfo;

  MachineLoopInfo *MLI;
  LoopInfo *LI;
  AliasAnalysis *AA;
  ScalarEvolution *SE;
  // Also remember the operations that do not use by any others operations in
  // the same bb.
  std::set<const MachineInstr*> MIsToWait, MIsToRead;

  VPreRegAllocSched() : MachineFunctionPass(ID) {
    initializeVPreRegAllocSchedPass(*PassRegistry::getPassRegistry());
  }

  //===--------------------------------------------------------------------===//
  // Loop memory dependence information.
  int analyzeLoopDep(MachineMemOperand *SrcAddr, MachineMemOperand *DstAddr,
                     Loop &L, bool SrcBeforeDest);

  unsigned calculateLatencyFromEntry(MachineInstr *MI) const;
  unsigned calculateLatencyFromEntry(VSUnit *U) const;

  // We need to iterate over the operand latency table.
  typedef DetialLatencyInfo::DepLatInfoTy::const_iterator src_it;

  template<bool CrossBBOnly>
  void addControlPathDepForSU(VSUnit *A, VSchedGraph &G);

  typedef DetialLatencyInfo::DepLatInfoTy DepLatInfoTy;
  template<VDEdge::Types Type, bool CrossBBOnly>
  void addControlPathDepForMI(MachineInstr *MI, int MIOffset, VSUnit *A,
                        VSchedGraph &G, const DepLatInfoTy &LatInfo);
  // Add the dependence from the incoming value of PHI to PHI.
  void addIncomingDepForPHI(VSUnit *PN, VSchedGraph &G);

  void addDatapathDep(VSchedGraph &G, VSUnit *A);

  VSUnit *getDefSU(const MachineOperand &MO, VSchedGraph &G, MachineInstr *&Dep) {
    // Only care about the register dependences.
    if (!MO.isReg()) return 0;

    // The instruction do not depend the register defined by itself.
    if (MO.isDef()) return 0;

    unsigned Reg = MO.getReg();

    // It seems that sometimes the Register will be 0?
    if (!Reg) return 0;
    assert(TargetRegisterInfo::isVirtualRegister(Reg)
           && "Unexpected physics register!");

    Dep = MRI->getVRegDef(Reg);
    assert(Dep && "Register use without define?");
    /// Only add the dependence if DepSrc is in the same MBB with MI.
    return G.lookupSUnit(Dep);
  }

  void buildMemDepEdges(VSchedGraph &G, ArrayRef<VSUnit*> SUs);

  bool couldBePipelined(const MachineBasicBlock *MBB);

  typedef VSchedGraph::iterator iterator;
  void addDepsForBBEntry(VSchedGraph &G, VSUnit *EntrySU);

  /// Maintain the wait sets, which contain the MI that need to wait before
  /// leaving the BB.
  // Erase the instructions from exit set.
  void eraseFromWaitSet(const MachineInstr *MI) {
    MIsToRead.erase(MI);
    MIsToWait.erase(MI);
  }

  void updateWaitSets(MachineInstr *MI, VSchedGraph &G);

  void buildControlPathGraph(VSchedGraph &G, MachineBasicBlock *MBB,
                             std::vector<VSUnit*> &NewSUs);

  void buildDataPathGraph(VSchedGraph &G, ArrayRef<VSUnit*> NewSUs);

  void buildPipeLineDepEdges(VSchedGraph &G);

  typedef MachineBasicBlock::iterator instr_it;
  void buildExitRoot(VSchedGraph &G, MachineInstr *FirstTerminator,
                     std::vector<VSUnit*> &NewSUs);

  void buildTerminatorDeps(VSchedGraph &G, VSUnit *Terminator);

  VSUnit *buildSUnit(MachineInstr *MI, VSchedGraph &G);

  void mergeDstMux(VSUnit *U, VSchedGraph &G);

  bool mergeUnaryOp(MachineInstr *MI, unsigned OpIdx, VSchedGraph &G);

  /// @name FunctionPass interface
  //{
  static char ID;

  ~VPreRegAllocSched();

  bool runOnMachineFunction(MachineFunction &MF);

  void buildGlobalSchedulingGraph(VSchedGraph &G, MachineBasicBlock *Entry,
                                  MachineBasicBlock *VExit);
  bool pipelineBBLocally(VSchedGraph &G, MachineBasicBlock *MBB,
                         MachineBasicBlock *VExit);
  void schedule(VSchedGraph &G);

  // Remove redundant code after schedule emitted.
  void cleanUpSchedule();
  bool cleanUpRegisterClass(unsigned RegNum, const TargetRegisterClass *RC);

  void getAnalysisUsage(AnalysisUsage &AU) const;
  void print(raw_ostream &O, const Module *M) const;
  //}

  const char *getPassName() const {
    return "Schedule Hardware Operations for Verilog Backend";
  }
};
}

//===----------------------------------------------------------------------===//
char VPreRegAllocSched::ID = 0;

INITIALIZE_PASS_BEGIN(VPreRegAllocSched, "Verilog-pre-reg-allocet-sched",
                      "Verilog pre reg allocet sched", false, false)
  INITIALIZE_PASS_DEPENDENCY(MachineBasicBlockTopOrder)
  INITIALIZE_PASS_DEPENDENCY(LoopInfo)
  INITIALIZE_PASS_DEPENDENCY(ScalarEvolution)
  INITIALIZE_PASS_DEPENDENCY(MachineBlockFrequencyInfo)
  INITIALIZE_PASS_DEPENDENCY(MachineBranchProbabilityInfo)
  INITIALIZE_PASS_DEPENDENCY(MachineLoopInfo)
  INITIALIZE_PASS_DEPENDENCY(DetialLatencyInfo)
INITIALIZE_PASS_END(VPreRegAllocSched, "Verilog-pre-reg-allocet-sched",
                    "Verilog pre reg allocet sched", false, false)

Pass *llvm::createVPreRegAllocSchedPass() {
  return new VPreRegAllocSched();
}

void VPreRegAllocSched::getAnalysisUsage(AnalysisUsage &AU) const {
  MachineFunctionPass::getAnalysisUsage(AU);
  // Topological order of MachineBasicBlocks are not preserved, because we may
  // insert delay blocks between MBBs, but the newly inserted blocks are placed
  // at the end of the function.
  AU.addRequiredID(MachineBasicBlockTopOrderID);
  AU.addRequired<LoopInfo>();
  AU.addPreserved<LoopInfo>();
  AU.addRequired<ScalarEvolution>();
  AU.addPreserved<ScalarEvolution>();
  AU.addRequired<MachineBlockFrequencyInfo>();
  AU.addRequired<MachineBranchProbabilityInfo>();
  AU.addRequired<MachineLoopInfo>();
  AU.addRequired<AliasAnalysis>();
  AU.addPreserved<AliasAnalysis>();
  AU.addRequired<DetialLatencyInfo>();
}

bool VPreRegAllocSched::runOnMachineFunction(MachineFunction &MF) {
  TII = MF.getTarget().getInstrInfo();
  MRI = &MF.getRegInfo();
  FInfo = MF.getInfo<VFInfo>();
  AA = &getAnalysis<AliasAnalysis>();
  MLI = &getAnalysis<MachineLoopInfo>();
  LI = &getAnalysis<LoopInfo>();
  SE = &getAnalysis<ScalarEvolution>();

  // Create a place holder for the virtual exit for the scheduling graph.
  MachineBasicBlock *VirtualExit = MF.CreateMachineBasicBlock();
  MF.push_back(VirtualExit);
  
  VSchedGraph G(getAnalysis<DetialLatencyInfo>(), false, 1);

  buildGlobalSchedulingGraph(G, &MF.front(), VirtualExit);

  schedule(G);

  DEBUG(G.viewCPGraph());
  DEBUG(G.viewDPGraph());
  // Erase the virtual exit block.
  VirtualExit->eraseFromParent();
  MF.RenumberBlocks(&MF.back());

  unsigned TotalCycles = G.emitSchedule();
  FInfo->setTotalSlots(TotalCycles);

  cleanUpSchedule();

  return true;
}

void VPreRegAllocSched::print(raw_ostream &O, const Module *M) const {}

VPreRegAllocSched::~VPreRegAllocSched() {}

//===----------------------------------------------------------------------===//
int VPreRegAllocSched::analyzeLoopDep(MachineMemOperand *SrcAddr,
                                      MachineMemOperand *DstAddr,
                                      Loop &L, bool SrcBeforeDest) {
  uint64_t SrcSize = SrcAddr->getSize();
  uint64_t DstSize = DstAddr->getSize();
  Value *SrcAddrVal = const_cast<Value*>(SrcAddr->getValue()),
        *DstAddrVal = const_cast<Value*>(DstAddr->getValue());

  DEBUG(dbgs() << '\n'
        << *SrcAddr->getValue() << "+" << SrcAddr->getOffset() << " and "
        << *DstAddr->getValue() << "+" << DstAddr->getOffset() << ": ");

  if (!isMachineMemOperandAlias(SrcAddr, DstAddr, AA, SE))
    return -1;

  if (L.isLoopInvariant(SrcAddrVal) && L.isLoopInvariant(DstAddrVal)) {
    DEBUG(dbgs() << " Invariant");
    // FIXME: What about nested loops?
    // Loop Invariant, let AA decide.
    return getLoopDepDist(SrcBeforeDest);
  }

  // We can only handle two access have the same element size.
  if (SrcSize == DstSize) {
    const SCEV *SSAddr =
      SE->getSCEVAtScope(getMachineMemOperandSCEV(SrcAddr, SE), &L);
    const SCEV *SDAddr =
      SE->getSCEVAtScope(getMachineMemOperandSCEV(DstAddr, SE), &L);
    return getLoopDepDist(SSAddr, SDAddr, SrcBeforeDest, SrcSize, SE);
  }

  // Cannot handle, simply assume dependence occur.
  return getLoopDepDist(SrcBeforeDest);
}

static inline bool mayAccessMemory(const MCInstrDesc &TID) {
  return TID.mayLoad() || TID.mayStore() || TID.isCall();
}

void VPreRegAllocSched::buildMemDepEdges(VSchedGraph &G, ArrayRef<VSUnit*> SUs){
  // The schedule unit and the corresponding memory operand.
  typedef std::vector<std::pair<MachineMemOperand*, VSUnit*> > MemOpMapTy;
  MemOpMapTy VisitedOps;
  Loop *IRL = LI->getLoopFor(G.getEntryBB()->getBasicBlock());

  typedef ArrayRef<VSUnit*>::iterator it;
  for (it I = SUs.begin(), E = SUs.end(); I != E; ++I) {
    VSUnit *DstU = *I;
    MachineInstr *DstMI = DstU->getRepresentativePtr();
    // Skip the non-memory operation and non-call operation.
    if (!mayAccessMemory(DstMI->getDesc())) continue;

    bool isDstWrite = VInstrInfo::mayStore(DstMI);

    // Dirty Hack: Is the const_cast safe?
    MachineMemOperand *DstMO = 0;
    // TODO: Also try to get the address information for call instruction.
    if (!DstMI->memoperands_empty() && !DstMI->hasVolatileMemoryRef()) {
      assert(DstMI->hasOneMemOperand() && "Can not handle multiple mem ops!");
      assert(!DstMI->hasVolatileMemoryRef() && "Can not handle volatile op!");
      
      // FIXME: DstMO maybe null in a VOpCmdSeq
      if ((DstMO = /*ASSIGNMENT*/ *DstMI->memoperands_begin())){
        assert(!isa<PseudoSourceValue>(DstMO->getValue())
               && "Unexpected frame stuffs!");
      }
    }

    typedef MemOpMapTy::iterator visited_it;
    for (visited_it I = VisitedOps.begin(), E = VisitedOps.end(); I != E; ++I) {
      MachineMemOperand *SrcMO = I->first;
      VSUnit *SrcU = I->second;

      MachineInstr *SrcMI = SrcU->getRepresentativePtr();

      bool MayBothActive = !VInstrInfo::isPredicateMutex(SrcMI, DstMI);
      if (!MayBothActive) ++MutexPredNoAlias;

      // Handle unanalyzable memory access.
      if (DstMO == 0 || SrcMO == 0) {
        // Build the Src -> Dst dependence.
        unsigned Latency = G.getStepsToFinish(SrcMI);
        //if (MayBothActive || SrcMO != DstMO)
        DstU->addDep<true>(SrcU, VDEdge::CreateMemDep(Latency, 0));

        // Build the Dst -> Src (in next iteration) dependence, the dependence
        // occur even if SrcMI and DstMI are mutual exclusive.
        if (G.enablePipeLine()) {
          Latency = G.getStepsToFinish(SrcMI);
          SrcU->addDep<true>(DstU, VDEdge::CreateMemDep(Latency, 1));
        }
        // Go on handle next visited SUnit.
        continue;
      }

      bool isSrcWrite = VInstrInfo::mayStore(SrcMI);

      // Ignore RAR dependence.
      if (!isDstWrite && !isSrcWrite) continue;

      if (!isMachineMemOperandAlias(SrcMO, DstMO, AA, SE))
        continue;

      if (G.enablePipeLine()) {
        assert(IRL && "Can not handle machine loop without IR loop!");
        DEBUG(SrcMI->dump();  dbgs() << "vs\n"; DstMI->dump(); dbgs() << '\n');

        // Dst not depend on Src if they are mutual exclusive.
        if (MayBothActive) {
          // Compute the iterate distance.
          int DepDst = analyzeLoopDep(SrcMO, DstMO, *IRL, true);

          if (DepDst >= 0) {
            unsigned Latency = G.getStepsToFinish(SrcMI);
            DstU->addDep<true>(SrcU, VDEdge::CreateMemDep(Latency, DepDst));
          }
        }

        // We need to compute if Src depend on Dst even if Dst not depend on Src.
        // Because dependence depends on execute order, if SrcMI and DstMI are
        // mutual exclusive.
        int DepDst = analyzeLoopDep(DstMO, SrcMO, *IRL, false);

        if (DepDst >=0 ) {
          unsigned Latency = G.getStepsToFinish(SrcMI);
          SrcU->addDep<true>(DstU, VDEdge::CreateMemDep(Latency, DepDst));
        }
      } else if (MayBothActive) {
        unsigned Latency = G.getStepsToFinish(SrcMI);
        DstU->addDep<true>(SrcU, VDEdge::CreateMemDep(Latency, 0));
      }
    }

    // Add the schedule unit to visited map.
    VisitedOps.push_back(std::make_pair(DstMO, DstU));
  }
}

unsigned VPreRegAllocSched::calculateLatencyFromEntry(MachineInstr *MI) const {
  MachineBasicBlock *MBB = MI->getParent();

  // PHIs can be scheduled to the first slot.
  if (MI->isPHI()) return 0;

  // Schedule data-path operation right after the first control slot.
  if (VInstrInfo::isDatapath(MI->getOpcode())) return 0;

  // Reading the result of operation in the same MBB?
  for (unsigned i = 0, e = MI->getNumOperands(); i < e; ++i) {
    const MachineOperand &MO = MI->getOperand(i);

    if (!MO.isReg() || !MO.getReg() || MO.isDef())
      continue;

    unsigned UseReg = MO.getReg();
    MachineInstr *DefMI = MRI->getVRegDef(UseReg);
    assert(DefMI && "DefMI not found!");
    // Cannot schedule to the first slot if MI is using the result of the
    // operations in the same BB.
    if (DefMI->getParent() == MBB) return 1;

    // No need to check the data-path definition.
    if (VInstrInfo::isDatapath(DefMI->getOpcode())) continue;
  }

  return 0;
}
unsigned VPreRegAllocSched::calculateLatencyFromEntry(VSUnit *U) const {
  int Latency = 0;

  for (unsigned i = 0, e = U->num_instrs(); i < e; ++i) {
    // Do not consider positive intra schedule unit latency at the moment.
    int IntraLatency = std::min(int(U->getLatencyAt(i)), 0);
    int InstLatency = calculateLatencyFromEntry(U->getPtrAt(i));
    Latency = std::max(Latency, InstLatency - IntraLatency);
  }

  return Latency;
}

//===----------------------------------------------------------------------===//
template<VDEdge::Types Type, bool CrossBBOnly>
void VPreRegAllocSched::addControlPathDepForMI(MachineInstr *MI, int MIOffset,
                                         VSUnit *A, VSchedGraph &G,
                                         const DepLatInfoTy &LatInfo) {
  assert(MI && "Unexpected entry root!");
  // DIRTY HACK: Positive offset is not supported now.
  MIOffset = std::min(MIOffset, 0);
  MachineBasicBlock *CurMBB = MI->getParent();
  assert(A->isControl() && "Unexpected data-path operation!");

  // FIXME: If several SrcMIs merged into a same SUnit, we may adding edges
  // from the same source.
  for (src_it I = LatInfo.begin(), E = LatInfo.end(); I != E; ++I) {
    InstPtrTy Src = I->first;
    // Get the latency from SrcMI to MI.
    float DetailLatency = DetialLatencyInfo::getMaxLatency(*I);
    int Latency = int(ceil(DetailLatency));

    // LatencyInfo use a special marker to mark the current MI have some latency
    // from entry of the MBB.
    if (MachineBasicBlock *SrcBB = Src) {
      VSUnit *SrcSU = G.lookupSUnit(SrcBB);
      // Ignore the dependency from other BB in local scheduling mode.
      if (SrcSU == 0) continue;

      if (SrcBB != CurMBB || !CrossBBOnly)
        A->addDep<true>(SrcSU, VDEdge::CreateDep<Type>(Latency - MIOffset));

      // If we are only adding cross basic block dependencies, do not add the
      // control dependencies from the entry of the same BB.
      if (SrcBB != CurMBB && !CrossBBOnly) {
        // Add the cross dependent edge from the BBEntry if SrcMI is from others
        // BB, because we may need to adjust its latency during scheduling.
        VSUnit *BBEntry = G.lookupSUnit(CurMBB);
        unsigned LatencyFromBBEntry = calculateLatencyFromEntry(MI);
        LatencyFromBBEntry -= MIOffset;
        A->addDep<true>(BBEntry, VDEdge::CreateCtrlDep(LatencyFromBBEntry));
      }

      continue;
    }

    MachineInstr *SrcMI = Src.get_mi();
    if (VInstrInfo::isDatapath(SrcMI->getOpcode())) continue;

    VSUnit *SrcSU = G.lookupSUnit(SrcMI);

    if (SrcMI->getParent() != CurMBB) {
      // If we are only adding cross basic block dependencies, do not add the
      // control dependencies from the entry of the same BB.
      if (!CrossBBOnly) {
        VSUnit *BBEntry = G.lookupSUnit(CurMBB);
        // Add the cross dependent edge from the BBEntry if SrcMI is from others
        // BB, because we may need to adjust its latency during scheduling.
        unsigned LatencyFromBBEntry = calculateLatencyFromEntry(MI);
        LatencyFromBBEntry -= MIOffset;
        // FIXME: The latency of this constraint will be changed during
        // scheduling.
        A->addDep<true>(BBEntry, VDEdge::CreateCtrlDep(LatencyFromBBEntry));
      }

      // Note that the dangling node are not chained with its depending control
      // operations, so for the scheduled instruction that has nozero latency,
      // the result is written to register, so the result will available 1 slot
      // later than it is expected when we are computing the original latency.
      if (!VInstrInfo::isCopyLike(SrcMI->getOpcode())) Latency += 1;

      // We are in local scheduling mode.
      if (SrcSU == 0) continue;
    }

    // If we are adding cross basic block dependencies only, do not add the
    // intra-basic-block dependencies.
    if (CrossBBOnly && SrcMI->getParent() == CurMBB)
      continue;

    // Get the minimal steps between MI and its dependency.
    unsigned MinCtrlDistance = G.getCtrlStepBetween<Type>(SrcMI, MI);
    // The the latency must bigger than the minimal latency between two control
    // operations.
    Latency = std::max(int(MinCtrlDistance), Latency);

    assert(SrcSU && "Src SUnit not found!");
    assert(SrcSU->isControl() && "Datapath dependence should be forwarded!");
    // Avoid the back-edge or self-edge.
    if (SrcSU->getIdx() >= A->getIdx()) continue;
    // Call getLatencyTo to accumulate the intra-unit latency.
    Latency = SrcSU->getLatencyFrom(SrcMI, Latency);
    Latency -= MIOffset;
    A->addDep<true>(SrcSU, VDEdge::CreateDep<Type>(Latency));
  }

  // Make sure the current MI start after the entry of the CurMBB if there is
  // no dependencies edge for current MI is added.
  if (!CrossBBOnly && LatInfo.empty() && -MIOffset > 0)
    A->addDep<true>(G.lookupSUnit(CurMBB), VDEdge::CreateCtrlDep(-MIOffset));
}

void VPreRegAllocSched::addIncomingDepForPHI(VSUnit *PHIMove, VSchedGraph &G){
  assert(G.enablePipeLine() && "Unexpected PHIMove!");

  // Find the incoming copy.
  MachineInstr *IncomingCopy = PHIMove->getRepresentativePtr();
  assert(IncomingCopy->getOpcode() == VTM::VOpMvPhi && "Expect PHI move!");

  unsigned Reg = IncomingCopy->getOperand(0).getReg();
  assert(MRI->hasOneUse(Reg) && "Incoming copy has more than one use?");
  MachineInstr *PN = &*MRI->use_begin(Reg);
  assert(PN && PN->isPHI() && "Bad user of incoming copy!");
  VSUnit *PHISU = G.lookupSUnit(PN);
  assert(PHISU && "Schedule unit for PHI node not found!");
  // Add the anti-dependency edge from the Incoming value copy to the PHI.
  PHISU->addDep<true>(PHIMove, VDEdge::CreateMemDep(0, 1));
  // Also add a dependencies from PHISU to the incoming copy SU, so the PHIMove
  // SU will be schedule II slots after the PHISU.
  PHIMove->addDep<true>(PHISU, VDEdge::CreateMemDep(0, -1));
}

void VPreRegAllocSched::addDatapathDep(VSchedGraph &G, VSUnit *A) {
  // Ignore the basic block entry.
  if (A->getRepresentativePtr().isMBB()) return;

  typedef VSUnit::instr_iterator it;
  bool IsCtrl = A->isControl();
  unsigned NumValDep = 0;
  MachineBasicBlock *ParentBB = A->getParentBB();

  // Simply add dependencies between data-path operations so that the schedule
  // preserve the dependencies, the latencies between data-path operations are
  // not capture here.
  for (unsigned I = 0, E = A->num_instrs(); I < E; ++I) {
    MachineInstr *MI = A->getPtrAt(I);
    assert(MI && "Unexpected entry root!");
    int IntraSULatency = A->getLatencyAt(I);
    // Prevent the data-path dependency from scheduling to the same slot with
    // the MI with the Control SU.
    if (IntraSULatency < 0 && IsCtrl)
      IntraSULatency -= DetialLatencyInfo::DeltaLatency;

    const DepLatInfoTy *DepLats = G.getDepLatInfo(MI);
    for (src_it I = DepLats->begin(), E = DepLats->end(); I != E; ++I) {
      VSUnit *SrcSU = G.lookupSUnit(I->first);
      assert(SrcSU && "Bad source scheduling unit!");
      bool IsSrcCtrl = SrcSU->isControl();
      // No need to add control-path to control-path dependencies into the
      // data-path dependencies graph.
      if (IsSrcCtrl && IsCtrl) continue;

      // Initialize the Latency to data-path to data-path dependencies. Setting
      // it to 0 means their can chain together.
      int Latency = 0;

      // Get the minimal latency from the control-path dependencies to current
      // MI, when bit-level chaining is enabled, they are in fact the latencies
      // from the control-path dependencies to the first started bit of current
      // MI.
      if (IsSrcCtrl)
        Latency = floor(DetialLatencyInfo::getMinLatency(*I));
      // Get the maximal latency from the data-path dependencies to control-path
      // operations, because the control-path operations read their operand
      // value right before it start, hence we need to get the maximal latency
      // to ensure the data-path operation had already finished when its result
      // is read.
      else if (IsCtrl)
        Latency = std::max<int>(ceil(DetialLatencyInfo::getMaxLatency(*I)), 1);
      else {//if(!IsCtrl && !IsSrcCtrl)
        MachineInstr *SrcMI = I->first;
        // Only create the edge if MI is actually reading the result of SrcMI.
        if (!SrcMI || !MI->readsVirtualRegister(SrcMI->getOperand(0).getReg()))
          continue;
      }

      Latency -= IntraSULatency;
      Latency = SrcSU->getLatencyFrom(I->first, Latency);
      // There is non back-edge in data-path dependency graph.
      assert(SrcSU->getIdx() < A->getIdx()
             &&"Unexpected back-edge or loop in data-path!");

      A->addDep<false>(SrcSU, VDEdge::CreateDep<VDEdge::ChainSupporting>(Latency));

      if (SrcSU->getParentBB() == ParentBB) ++NumValDep;
    }
  }

  assert((NumValDep || IsCtrl) && "Expect at least 1 dependency in the same MBB!");
  (void) NumValDep;
}

template<bool CrossBBOnly>
void VPreRegAllocSched::addControlPathDepForSU(VSUnit *A, VSchedGraph &G) {
  assert(A->isControl() && "Unexpected data-path operations!");

  for (unsigned I = 0, E = A->num_instrs(); I != E; ++I) {
    MachineInstr *MI = A->getPtrAt(I);    
    assert(MI && "Unexpected entry root!");
    const DetialLatencyInfo::DepLatInfoTy *Deps = G.getDepLatInfo(MI);
    assert(Deps && "Operand latency information not available!");
    int MIOffset = A->getLatencyAt(I);
    addControlPathDepForMI<VDEdge::ValDep, CrossBBOnly>(MI, MIOffset, A, G, *Deps);
  }

  if (!cp_empty(A)) return;

  // Restrict the control-path operations within the BB boundaries.
  VSUnit *Entry = CrossBBOnly?G.getEntryRoot():G.lookupSUnit(A->getParentBB());
  unsigned LatencyFromBBEntry = CrossBBOnly ? 0 : calculateLatencyFromEntry(A);
  A->addDep<true>(Entry, VDEdge::CreateCtrlDep(LatencyFromBBEntry));
}

bool VPreRegAllocSched::couldBePipelined(const MachineBasicBlock *MBB) {
  MachineLoop *L = MLI->getLoopFor(MBB);
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

void VPreRegAllocSched::buildPipeLineDepEdges(VSchedGraph &G) {
  VSUnit *LoopOp = G.getLoopOp();
  assert(LoopOp && "Not in loop?");
  MachineBasicBlock *CurBB = G.getEntryBB();
  VSUnit *CurTerminator = G.lookUpTerminator(CurBB);
  assert(CurTerminator && "Cannot get terminator!");
  assert(LoopOp != CurTerminator && "Pipeline not enable!");

  for (iterator I = cp_begin(&G) + 1, E = cp_end(&G);
       I != E && (*I)->isPHI(); ++I) {
    VSUnit *PHISU = *I;

    // Add a anti-dependence edge from users of PHI to PHI because we must
    // have:
    // PHI -(RAW dep)-> PHI_user -(WAR dep)-> PHI_at_next_iteration.
    typedef VSUnit::use_iterator use_it;
    for (use_it UI = cuse_begin(PHISU), UE = cuse_end(PHISU); UI != UE; ++UI){
      VSUnit *PHIUser = *UI;
      if (PHIUser != CurTerminator)
        PHISU->addDep<true>(PHIUser, VDEdge::CreateMemDep(0, 1));
    }

    // Add the dependence edge PHI -> Loop back -> PHI_at_iteration.
    PHISU->addDep<true>(LoopOp, VDEdge::CreateMemDep(0, 1));
    //LoopOp->addDep(VDValDep::CreateValDep(PHISU, 0));
  }
}

bool VPreRegAllocSched::mergeUnaryOp(MachineInstr *MI, unsigned OpIdx,
                                     VSchedGraph &G) {
  MachineInstr *SrcMI = 0;
  // Try to merge it into the VSUnit that defining its source operand.
  if (VSUnit *SrcSU = getDefSU(MI->getOperand(OpIdx), G, SrcMI))
    return G.mapMI2SU(MI, SrcSU, SrcSU->getValLatencyTo(SrcMI, MI, G));

  // Try to merge it into the VSUnit that defining its predicate operand.
  if (const MachineOperand *Pred = VInstrInfo::getPredOperand(MI))
    if (VSUnit *SrcSU = getDefSU(*Pred, G, SrcMI))
      return G.mapMI2SU(MI, SrcSU, SrcSU->getValLatencyTo(SrcMI, MI, G));

  // Merge it into the EntryRoot.
  return G.mapMI2SU(MI, G.lookupSUnit(MI->getParent()), G.getStepsFromEntry(MI));
}

void VPreRegAllocSched::mergeDstMux(VSUnit * U, VSchedGraph &G) {
  // Look for the source value form distributed multiplexers.
  SmallVector<std::pair<MachineInstr*, unsigned>, 4> WorkStack;
  WorkStack.push_back(std::make_pair(U->getRepresentativePtr(), 0));

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
    G.mapMI2SU(ChildMI, U, -Level);

    // The operand 0 is a define, simply ignore it.
    WorkStack.push_back(std::make_pair(ChildMI, 1));
  }
}

VSUnit *VPreRegAllocSched::buildSUnit(MachineInstr *MI,  VSchedGraph &G) {
  assert(!MI->getDesc().isTerminator() && "Unexpected terminator!");

  switch (MI->getOpcode()) {
  default: break;
  case VTM::VOpMove:
    if (mergeUnaryOp(MI, 1, G)) return 0;
    break;
  case VTM::VOpDisableFU: {
    MachineInstr *SrcMI = 0;
    // Try to merge it into the VSUnit that defining its source operand.
    VSUnit *SrcSU = getDefSU(MI->getOperand(0), G, SrcMI);
    assert(SrcSU && "Expected source schedule unit!");
    // Disable the FU at next state.
    bool merged = G.mapMI2SU(MI, SrcSU, 1);
    assert(merged && "DisableFU not merged?");
    (void) merged;
    return 0;
  }
  case VTM::PHI:
    // Merge the the PHI into entry root if the BB is not pipelined.
    if (!G.enablePipeLine()) {
      G.mapMI2SU(MI, G.lookupSUnit(MI->getParent()), G.getStepsFromEntry(MI));
      return 0;
    }
    break;
  case VTM::VOpReadReturn:
    if (mergeUnaryOp(MI, 1, G))
      return 0;
    break;
  case VTM::VOpMoveArg:
    G.mapMI2SU(MI, G.getEntryRoot(), 0);
    return 0;
  // The VOpDstMux should be merged to its user.
  case VTM::VOpDstMux: return 0;
  case VTM::VOpMvPhi:
    assert(G.isLoopPHIMove(MI) && "Unexpected PHIMove!");
    break;
  }

  // TODO: Remember the register that live out this MBB.
  // and the instruction that only produce a chain.
  FuncUnitId Id = VInstrInfo::getPreboundFUId(MI);
  VSUnit *U = G.createVSUnit(MI, Id.getFUNum());
  if (Id.isBound()) mergeDstMux(U, G);
  return U;
}

void VPreRegAllocSched::buildTerminatorDeps(VSchedGraph &G, VSUnit *Terminator) {
  typedef MachineBasicBlock::instr_iterator it;
  MachineInstr *ExitMI = Terminator->getRepresentativePtr();
  assert(ExitMI && ExitMI->isTerminator() && "Unexpected instruction type!");
  MachineBasicBlock *MBB = Terminator->getParentBB();

  for (it I = MBB->instr_begin(), E = MBB->instr_end(); I != E; ++I) {
    MachineInstr *MI = I;
    VSUnit *VSU = G.lookupSUnit(MI);

    if (VSU->isScheduled()) continue;

    // Since the exit root already added to state sunit list, skip the
    // exit itself.
    if (cuse_empty(VSU) && VSU != Terminator) {
      if (VSU->isDatapath()) continue;

      if (!G.isLoopOp(VSU->getRepresentativePtr()))
        llvm_unreachable("Unexpected handing node!");

      // A PHIMove can be scheduled to the same slot with the exit root.
      unsigned Latency = VSU->getMaxLatencyTo<VDEdge::CtrlDep>(ExitMI, G);
      // We do not need to wait the trivial operation finish before exiting the
      // state, because the first control slot of next state will only contains
      // PHI copies, and the PHIElimination Hook will take care of the data
      // dependence and try to forward the wire value in last control slot
      // if possible, so they can take the time of the last control slot.
      //VIDesc VID(*Instr);
      //if (VID.hasTrivialFU() && !VID.hasDatapath() && Latency)
      //  --Latency;

      Terminator->addDep<true>(VSU, VDEdge::CreateCtrlDep(Latency));
    }
  }
}

void VPreRegAllocSched::addDepsForBBEntry(VSchedGraph &G, VSUnit *EntrySU) {
  MachineBasicBlock *MBB = EntrySU->getRepresentativePtr();
  assert(MBB && "Bad entry node type!");

  typedef MachineBasicBlock::pred_iterator pred_iterator;
  for (pred_iterator I = MBB->pred_begin(), E = MBB->pred_end(); I != E; ++I) {
    MachineBasicBlock *PredBB = *I;

    // Avoid self-loop.
    if (PredBB == MBB) continue;

    // Ignore the terminator that is not yet created, which mean we are ignoring
    // the loop back edges.
    if (VSUnit *PredTerminator = G.lookUpTerminator(PredBB))
      EntrySU->addDep<true>(PredTerminator, VDEdge::CreateCtrlDep(0));
  }
}

void VPreRegAllocSched::buildExitRoot(VSchedGraph &G,
                                      MachineInstr *FirstTerminator,
                                      std::vector<VSUnit*> &NewSUs) {
  SmallVector<MachineInstr*, 8> Exits;
  // We need wait all operation finish before the exit operation active, compute
  // the latency from operations need to wait to the exit operation.
  DetialLatencyInfo::DepLatInfoTy ExitDeps;
  MachineBasicBlock *MBB = FirstTerminator->getParent();

  for (instr_it I = FirstTerminator, E = MBB->end(); I != E; ++I) {
    MachineInstr *MI = I;
    if (!I->isTerminator()) {
      assert(MI->getOpcode() == VTM::VOpMvPhi && "Bad MBB!");
      continue;
    }

    updateWaitSets(MI, G);

    // Build the schedule unit for loop back operation.
    if (G.isLoopOp(I)) {
      VSUnit *LoopOp = G.createVSUnit(MI);
      addControlPathDepForSU<false>(LoopOp, G);
      VSUnit *BBEntry = G.lookupSUnit(MBB);
      // Schedule the LoopOp II slots after the entry SU.
      LoopOp->addDep<true>(BBEntry, VDEdge::CreateMemDep(0, -1));
      NewSUs.push_back(LoopOp);
      continue;
    }
  }
  
  VSUnit *ExitSU = G.createTerminator(MBB);
  NewSUs.push_back(ExitSU);
  for (instr_it I = FirstTerminator, E = MBB->end(); I != E; ++I) {
    MachineInstr *MI = I;
    // Ignore the VOpMvPhis, which are handled, and also ignore the looping-back
    // terminator, which had been handled in the previous loop.
    if (!I->isTerminator() || G.isLoopOp(I)) continue;

    // No need to wait the terminator.
    eraseFromWaitSet(MI);

    // Build a exit root or merge the terminators into the exit root.
    bool mapped = G.mapMI2SU(MI, ExitSU, 0);
    (void) mapped;
    assert(mapped && "Cannot merge terminators!");
  }

  assert(ExitSU && "Terminator not found?");

  for (instr_it I = FirstTerminator, E = MBB->end(); I != E; ++I) {
    MachineInstr *MI = I;

    if (MI->getOpcode() == VTM::VOpMvPhi) {
      if (!G.isLoopPHIMove(MI)) { // Also merge the PHI moves.
        bool mapped = G.mapMI2SU(MI, ExitSU, 0);
        (void) mapped;
        assert(mapped && "Cannot merge terminators!");
      } else
        // Do not erase the PHIMove from wait set, so that the ExitSU will
        // wait until the PHIMove is finished.
        continue;

      // No need to wait VOpMvPhi, because we are going to merge it into the
      // exit root.
      eraseFromWaitSet(MI);
    } else if (G.isLoopOp(I))
      continue;

    // Build datapath latency information for the terminator.
    G.buildExitMIInfo(MI, ExitDeps, MIsToWait, MIsToRead);
  }

  // Add the control dependence edge edges to wait all operation finish.
  addControlPathDepForMI<VDEdge::CtrlDep, false>(ExitSU->getRepresentativePtr(),
                                           0/*Offset*/, ExitSU, G, ExitDeps);
  // Add the dependence of exit root.
  addControlPathDepForSU<false>(ExitSU, G);

  // If there is still schedule unit not connect to exit, connect it now, but
  // they are supposed to be connected in the previous stages, so dangling node
  // is not allow.
  buildTerminatorDeps(G, ExitSU);
}

void VPreRegAllocSched::updateWaitSets(MachineInstr *MI, VSchedGraph &G) {
  const DepLatInfoTy &Deps = *G.getDepLatInfo(MI);
  MachineBasicBlock *MBB = MI->getParent();

  // Compute the instruction which are need to wait before leaving the BB.
  bool IsControl = VInstrInfo::isControl(MI->getOpcode());

  // Iterate from use to define.
  for (unsigned i = 0, e = MI->getNumOperands(); i != e; ++i) {
    const MachineOperand &MO = MI->getOperand(i);

    // Only care about a use register.
    if (!MO.isReg() || MO.isDef() || MO.getReg() == 0)
      continue;

    unsigned SrcReg = MO.getReg();
    MachineInstr *SrcMI = MRI->getVRegDef(SrcReg);
    assert(SrcMI && "Virtual register use without define!");

    if (MBB != SrcMI->getParent()) continue;

    // Now MI is actually depends on SrcMI in this MBB, no need to wait it
    // explicitly.
    MIsToWait.erase(SrcMI);

    // SrcMI is read by a data-path operation, we need to wait its result before
    // exiting the BB if there is no other control operation read it.
    if (VInstrInfo::isControl(SrcMI->getOpcode()) && !IsControl)
      MIsToRead.insert(SrcMI);
  }

  if (IsControl || false /*Disable Dangling*/) MIsToWait.insert(MI);

  if (!IsControl) return;

  typedef DepLatInfoTy::const_iterator it;
  for (it I = Deps.begin(), E = Deps.end(); I != E; ++I) {
    const MachineInstr *SrcMI = I->first;
    if (SrcMI == 0 || MBB != SrcMI->getParent())
      continue;

    // SrcMI (or its user) is read by MI, no need to wait it explicitly.
    MIsToRead.erase(SrcMI);
  }
}

void VPreRegAllocSched::buildControlPathGraph(VSchedGraph &G, 
                                              MachineBasicBlock *MBB,
                                              std::vector<VSUnit*> &NewSUs) {
  // Clean the context
  MIsToWait.clear();
  MIsToRead.clear();

  instr_it BI = MBB->begin();
  while(!BI->isTerminator() && BI->getOpcode() != VTM::VOpMvPhi) {
    MachineInstr *MI = BI++;

    updateWaitSets(MI, G);

    if (VSUnit *U = buildSUnit(MI, G)) {
      NewSUs.push_back(U);
      if (U->isControl()) addControlPathDepForSU<false>(U, G);
    }
  }

  for (instr_it I = BI; !I->isTerminator(); ++I) {
    updateWaitSets(I, G);

    if (!G.isLoopPHIMove(I)) continue;
    
    VSUnit *PHIMove = buildSUnit(I, G);
    NewSUs.push_back(PHIMove);
    addIncomingDepForPHI(PHIMove, G);
    addControlPathDepForSU<false>(PHIMove, G);
  }

  // Create the exit node, now BI points to the first terminator.
  buildExitRoot(G, BI, NewSUs);

  // Build loop edges if necessary.
  if (G.enablePipeLine()) buildPipeLineDepEdges(G);

  // Build the memory edges.
  buildMemDepEdges(G, NewSUs);
}

void VPreRegAllocSched::buildDataPathGraph(VSchedGraph &G,
                                           ArrayRef<VSUnit*> NewSUs) {
  typedef ArrayRef<VSUnit*>::iterator iterator;
  for (iterator I = NewSUs.begin(), E = NewSUs.end(); I != E; ++I)
    addDatapathDep(G, *I);
}

bool VPreRegAllocSched::pipelineBBLocally(VSchedGraph &G, MachineBasicBlock *MBB,
                                          MachineBasicBlock *VExit) {
  VSchedGraph LocalG(G.DLInfo, true, 1);
  std::vector<VSUnit*> NewSUs;
  VSUnit *CurEntry = LocalG.createVSUnit(MBB);
  buildControlPathGraph(LocalG, MBB, NewSUs);
  // Todo: Simply set the terminator SU as the exit root?
  LocalG.createExitRoot(VExit);
  LocalG.verify();
  // Do not merge the local graph into the global graph if we fail to pipeline
  // the block.
  if (!LocalG.scheduleLoop()) return false;

  typedef VSchedGraph::iterator it;
  for (it I = G.mergeSUsInSubGraph(LocalG), E = cp_end(&G); I != E; ++I)
    addControlPathDepForSU<true>(*I, G);

  buildDataPathGraph(G, NewSUs);
  addDepsForBBEntry(G, CurEntry);

  // The BB is pipelined successfully.
  return true;
}

void VPreRegAllocSched::buildGlobalSchedulingGraph(VSchedGraph &G,
                                                   MachineBasicBlock *Entry,
                                                   MachineBasicBlock *VExit) {
  std::vector<VSUnit*> NewSUs;

  for (MachineFunction::iterator I = Entry, E = VExit; I != E; ++I) {
    MachineBasicBlock *MBB = I;

    // Perform software pipelining with local scheduling algorithm.
    // FIXME: Reuse the local scheduling graph which is built for software
    // pipelining.
    if (couldBePipelined(MBB) && pipelineBBLocally(G, MBB, VExit))
      continue;

    VSUnit *CurEntry = G.createVSUnit(MBB);
    addDepsForBBEntry(G, CurEntry);
    NewSUs.clear();
    buildControlPathGraph(G, MBB, NewSUs);
    buildDataPathGraph(G, NewSUs);
  }

  G.createExitRoot(VExit);
  // Verify the schedule graph.
  G.verify();
}

void VPreRegAllocSched::schedule(VSchedGraph &G) {
  MachineBlockFrequencyInfo &MBFI = getAnalysis<MachineBlockFrequencyInfo>();
  MachineBranchProbabilityInfo &MBPI
    = getAnalysis<MachineBranchProbabilityInfo>();
  double FreqSum = 0.0;
  MachineBasicBlock *EntryBB = G.getEntryBB(), *ExitBB = G.getExitBB();
  typedef MachineFunction::iterator iterator;

  for (iterator I = G.getEntryBB(), E = G.getExitBB(); I != E; ++I)
    FreqSum += MBFI.getBlockFreq(I).getFrequency();

  SDCScheduler<true> Scheduler(G);

  Scheduler.buildTimeFrameAndResetSchedule(true);
  BasicLinearOrderGenerator::addLinOrdEdge(Scheduler);
  // Build the step variables, and no need to schedule at all if all SUs have
  // been scheduled.
  if (Scheduler.createLPAndVariables()) {
    for (iterator I = EntryBB, E = ExitBB; I != E; ++I) {
      MachineBasicBlock *MBB = I;
      BlockFrequency BlockFreq = MBFI.getBlockFreq(MBB);
      double BBFreq = double(BlockFreq.getFrequency()) / FreqSum;
      // Avoid setting zero coefficient.
      BBFreq = std::max(0.001, BBFreq);

      DEBUG(dbgs() << "MBB#" << MBB->getNumber() << ' ' << BBFreq << '\n');
      // Minimize the latency of the BB.
      // Min (BBEnd - BBStart) * BBFreq;
      // => Max BBStart * BBFreq - BBEnd * BBFreq.
      Scheduler.addObjectCoeff(G.lookupSUnit(MBB), BBFreq);
      Scheduler.addObjectCoeff(G.lookUpTerminator(MBB), -BBFreq);

      typedef MachineBasicBlock::const_succ_iterator succ_iterator;
      for (succ_iterator SI = MBB->succ_begin(), SE = MBB->succ_begin();
           SI != SE; ++SI) {
        MachineBasicBlock *SuccBB = *SI;
        // Ignore the back-edges and edge to the virtual ExitBB.
        if (SuccBB->getNumber() <= MBB->getNumber() || SuccBB == ExitBB)
          continue;

        BlockFrequency EdgeProb
          = BlockFreq * MBPI.getEdgeProbability(MBB, SuccBB);
        double EdgeFreq
          = double(EdgeProb.getFrequency()) / double(BlockFreq.getFrequency());

        EdgeFreq *= BBFreq;
        // Avoid setting zero coefficient.
        EdgeFreq = std::max(0.001, EdgeFreq);

        // Minimize the latency of edge.
        // Min (SuccBBStart - MBBEnd) * EdgeFreq;
        // => Max MBBEnd * EdgeFreq - SuccBBStart * EdgeFreq
        Scheduler.addObjectCoeff(G.lookupSUnit(SuccBB), -EdgeFreq);
        // Do not set the coefficient of the terminator, otherwise the
        // coefficient to minimize the latency of the BB will be neutralized.
        //Scheduler.addObjectCoeff(G.lookUpTerminator(MBB), EdgeFreq);
      }
    }

    bool success = Scheduler.schedule();
    assert(success && "SDCScheduler fail!");
    (void) success;
  }

  G.scheduleDatapath();
}

void VPreRegAllocSched::cleanUpSchedule() {
  for (unsigned i = 0, e = MRI->getNumVirtRegs(); i != e; ++i) {
     unsigned RegNum = TargetRegisterInfo::index2VirtReg(i);
    cleanUpRegisterClass(RegNum, &VTM::DRRegClass);
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
  if (DefMI.getOpcode() == VTM::VOpReadFU ||
      DefMI.getOpcode() == VTM::VOpPipelineStage)    
    DI.getOperand().ChangeToRegister(0, true);
  else {
    // FIXME: Remove the PHI, and incoming copies (Bug 14).
    if (DefMI.getOpcode() == VTM::VOpDefPhi) return false;

    DI.getOperand().setIsDead(true);
    return false;
    //assert(0 && "Unexpected dead define!");
    //DefMI.eraseFromParent();
  }

  return true;
}
