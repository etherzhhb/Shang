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
#include "llvm/ADT/PostOrderIterator.h"
#include "llvm/Support/Allocator.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/GraphWriter.h"
#include "llvm/Support/MathExtras.h"
#define DEBUG_TYPE "vtm-sgraph"
#include "llvm/Support/Debug.h"
using namespace llvm;

STATISTIC(MutexPredNoAlias, "Number of no-alias because of mutex predicate");
STATISTIC(DanglingDatapath, "Number of dangling data-path operations");
//===----------------------------------------------------------------------===//
namespace {
/// @brief Schedule the operations.
///
struct VPreRegAllocSched : public MachineFunctionPass {
  const TargetInstrInfo *TII;
  // The loop Info
  MachineRegisterInfo *MRI;
  VFInfo *FInfo;
  IndexedMap<unsigned> CyclesFromEntry;

  TargetData *TD;

  MachineLoopInfo *LI;
  LoopInfo *IRLI;
  AliasAnalysis *AA;
  ScalarEvolution *SE;

  // Total states
  // Cycle is start from 1 because  cycle 0 is reserve for idle state.
  unsigned short totalCycle;

  VPreRegAllocSched() : MachineFunctionPass(ID), CyclesFromEntry(UINT32_MAX),
    totalCycle(1) {}

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

  LoopDep analyzeLoopDep(MachineMemOperand *SrcAddr, MachineMemOperand *DstAddr,
                         bool SrcLoad, bool DstLoad, Loop &L, bool SrcBeforeDest);


  LoopDep advancedLoopDepsAnalysis(MachineMemOperand *SrcAddr,
                                   MachineMemOperand *DstAddr,
                                   bool SrcLoad, bool DstLoad, Loop &L,
                                   bool SrcBeforeDest, unsigned ElSizeInByte);

  LoopDep createLoopDep(bool SrcLoad, bool DstLoad, bool SrcBeforeDest,
                        int Diff = 0);

  VDMemDep *getMemDepEdge(VSUnit *Src, unsigned Latency, unsigned Diff) {
    return new VDMemDep(Src, Latency, Diff);
  }

  unsigned getCyclesFromEntry(const MachineBasicBlock *MBB) const {
    return CyclesFromEntry[MBB->getNumber()];
  }

  unsigned &getCyclesFromEntry(const MachineBasicBlock *MBB) {
    return CyclesFromEntry[MBB->getNumber()];
  }

  unsigned getCyclesToBB(const MachineInstr *SrcMI,
                         const MachineBasicBlock *DstBB) const;
  unsigned getCyclesToBB(const MachineBasicBlock *SrcBB,
                         const MachineBasicBlock *DstBB) const;
  unsigned calculateLatencyFromEntry(MachineInstr *MI) const;
  bool hasFUConflictAtLastSlot(FuncUnitId Id, MachineBasicBlock *MBB) const;
  unsigned calculateLatencyFromEntry(VSUnit *U) const;

  // We need to iterate over the operand latency table.
  typedef DetialLatencyInfo::DepLatInfoTy::const_iterator src_it;

  void addSchedDepForSU(VSUnit *A, VSchedGraph &G,
                        bool isExit = false);
  typedef const DetialLatencyInfo::DepLatInfoTy DepLatInfoTy;
  template<typename DepEdgeTy>
  void addSchedDepForMI(MachineInstr *MI, int MIOffset, VSUnit *A,
                        VSchedGraph &G, DepLatInfoTy &LatInfo);
  // Add the dependence from the incoming value of PHI to PHI.
  void addIncomingDepForPHI(VSUnit *PN, VSchedGraph &G);

  void addValDep(VSchedGraph &G, VSUnit *A);

  VSUnit *getDefSU(const MachineOperand &MO, VSchedGraph &G, MachineInstr *&Dep) {
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

    Dep = MRI->getVRegDef(Reg);
    assert(Dep && "Register use without define?");
    /// Only add the dependence if DepSrc is in the same MBB with MI.
    return G.lookupSUnit(Dep);
  }

  void clear();

  void buildMemDepEdges(VSchedGraph &G, MachineBasicBlock *MBB);

  bool couldBePipelined(const MachineBasicBlock *MBB);

  typedef VSchedGraph::iterator su_it;
  void addDepsForBBEntry(VSchedGraph &G, VSUnit *EntrySU);
  void buildControlPathGraph(VSchedGraph &G, MachineBasicBlock *MBB);
  void buildDataPathGraph(VSchedGraph &G);

  void buildPipeLineDepEdges(VSchedGraph &G, MachineBasicBlock *MBB);

  typedef MachineBasicBlock::iterator instr_it;
  void buildExitRoot(VSchedGraph &G, MachineInstr *FirstTerminator);

  void scheduleDanglingDatapathOps(VSchedGraph &G);
  void clearDanglingFlagForTree(VSUnit *Root);

  template<int AllowDangling>
  void buildTerminatorDeps(VSchedGraph &G, VSUnit *Terminator);

  void buildSUnit(MachineInstr *MI, VSchedGraph &G);

  void mergeDstMux(VSUnit *U, VSchedGraph &G);

  bool mergeUnaryOp(MachineInstr *MI, unsigned OpIdx, VSchedGraph &G);

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

  MachineBasicBlock *Entry = MF.begin();
  ReversePostOrderTraversal<MachineBasicBlock*> RPOT(Entry);
  typedef ReversePostOrderTraversal<MachineBasicBlock*>::rpo_iterator rpo_it;

  DetialLatencyInfo DLInfo(*MRI, false);
  CyclesFromEntry.resize(MF.getNumBlockIDs());
  getCyclesFromEntry(Entry) = 0;

  for (rpo_it I = RPOT.begin(), E = RPOT.end(); I != E; ++I) {
    MachineBasicBlock *MBB = *I;
    DLInfo.resetExitSet();

    unsigned &CurCyclesFromEntry = getCyclesFromEntry(MBB);
    typedef MachineBasicBlock::pred_iterator pred_it;
    for (pred_it PI = MBB->pred_begin(), PE = MBB->pred_end(); PI != PE; ++PI)
      CurCyclesFromEntry = std::min(CurCyclesFromEntry, getCyclesFromEntry(*PI));

    VSchedGraph G(DLInfo, MBB, couldBePipelined(MBB), getTotalCycle());
    VSUnit *CurEntry = G.createVSUnit(MBB);
    addDepsForBBEntry(G, CurEntry);

    buildControlPathGraph(G, MBB);

    G.createExitRoot();
    // Sort the schedule units after all units are built.
    G.prepareForCtrlSched();
    // Verify the schedule graph.
    G.verify();

    DEBUG(G.viewGraph());
    G.scheduleCtrl();
    buildDataPathGraph(G);
    DEBUG(G.viewGraph());
    G.scheduleDatapath();
    setTotalCycle(G.getEndSlot() + 1);
    DEBUG(G.viewGraph());
    G.emitSchedule();
    // Compute the shortest distance from the entry to the end of this BB, note
    // that the last slot is in fact alias with the first slot of its successors,
    // so do not include the last slot when computing distance.
    unsigned TotalSlots = G.getTotalSlot();
    if (TotalSlots) TotalSlots -= 1;    
    CurCyclesFromEntry += TotalSlots;
  }

  FInfo->setTotalSlots(totalCycle);
  cleanUpSchedule();

  return true;
}

void VPreRegAllocSched::clear() {
  // Reset total Cycle
  totalCycle = 1;
  CyclesFromEntry.clear();
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
VPreRegAllocSched::analyzeLoopDep(MachineMemOperand *SrcAddr,
                                  MachineMemOperand *DstAddr,
                                  bool SrcLoad, bool DstLoad,
                                  Loop &L, bool SrcBeforeDest) {
  uint64_t SrcSize = SrcAddr->getSize();
  uint64_t DstSize = DstAddr->getSize();
  Value *SrcAddrVal = const_cast<Value*>(SrcAddr->getValue()),
        *DstAddrVal = const_cast<Value*>(DstAddr->getValue());

  DEBUG(dbgs() << '\n'
        << *SrcAddr->getValue() << "+" << SrcAddr->getOffset() << " and "
        << *DstAddr->getValue() << "+" << DstAddr->getOffset() << ": ");

  if (L.isLoopInvariant(SrcAddrVal) && L.isLoopInvariant(DstAddrVal)) {
    DEBUG(dbgs() << " Invariant");
    // FIXME: What about nested loops?
    // Loop Invariant, let AA decide.
    if (isMachineMemOperandAlias(SrcAddr, DstAddr, AA, SE))
      return createLoopDep(SrcLoad, DstLoad, SrcBeforeDest);
    else
      return LoopDep();
  }

  if (!isMachineMemOperandAlias(SrcAddr, DstAddr, AA, SE))
    return LoopDep();

  // We can only handle two access have the same element size.
  if (SrcSize == DstSize)
    return advancedLoopDepsAnalysis(SrcAddr, DstAddr, SrcLoad, DstLoad,
                                    L, SrcBeforeDest, SrcSize);

  // Cannot handle, simply assume dependence occur.
  return createLoopDep(SrcLoad, DstLoad, SrcBeforeDest);
}

VPreRegAllocSched::LoopDep
VPreRegAllocSched::advancedLoopDepsAnalysis(MachineMemOperand *SrcAddr,
                                            MachineMemOperand *DstAddr,
                                            bool SrcLoad, bool DstLoad,
                                            Loop &L, bool SrcBeforeDest,
                                            unsigned ElSizeInByte) {
  const SCEV *SSAddr =
    SE->getSCEVAtScope(getMachineMemOperandSCEV(SrcAddr, SE), &L);
  const SCEV *SDAddr =
    SE->getSCEVAtScope(getMachineMemOperandSCEV(DstAddr, SE), &L);
  DEBUG(dbgs() << *SSAddr << " and " << *SDAddr << ": ");
  // Use SCEV to compute the dependencies distance.
  const SCEV *Distance = SE->getMinusSCEV(SSAddr, SDAddr);
  DEBUG(Distance->dump());
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
   if (!SrcBeforeDest && (Diff == 0)) Diff = 1;

   assert(Diff >= 0 && "Do not create a dependence with diff small than 0!");
   assert(!(SrcLoad && DstLoad) && "Do not create a RAR dep!");

   // WAW
   if (!SrcLoad && !DstLoad ) {
     DEBUG(dbgs() << " Out " << Diff << '\n');
     return LoopDep(LoopDep::OutputDep, Diff);
   }

   if (!SrcLoad && DstLoad)
     SrcBeforeDest = !SrcBeforeDest;

   DEBUG(dbgs() << " Anti/True " << Diff << '\n');
   return LoopDep(SrcBeforeDest ? LoopDep::AntiDep : LoopDep::TrueDep, Diff);
}

static inline bool mayAccessMemory(const MCInstrDesc &TID) {
  return TID.mayLoad() || TID.mayStore() || TID.isCall();
}

void VPreRegAllocSched::buildMemDepEdges(VSchedGraph &G,
                                         MachineBasicBlock *MBB) {
  // The schedule unit and the corresponding memory operand.
  typedef std::vector<std::pair<MachineMemOperand*, VSUnit*> > MemOpMapTy;
  MemOpMapTy VisitedMemOps;
  Loop *IRL = IRLI->getLoopFor(G.getEntryBB()->getBasicBlock());

  typedef MachineBasicBlock::instr_iterator it;
  for (it I = MBB->instr_begin(), E = MBB->instr_end(); I != E; ++I) {
    MachineInstr *DstMI = I;
    // Skip the non-memory operation and non-call operation.
    if (!mayAccessMemory(DstMI->getDesc())) continue;

    bool isDstLoad = VInstrInfo::mayLoad(DstMI);

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

    VSUnit *DstU = G.lookupSUnit(DstMI);
    assert(DstU && "VSUnit for memory access not found!");

    for (MemOpMapTy::iterator I = VisitedMemOps.begin(), E = VisitedMemOps.end();
         I != E; ++I) {
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
        DstU->addDep(getMemDepEdge(SrcU, Latency, 0));

        // Build the Dst -> Src (in next iteration) dependence, the dependence
        // occur even if SrcMI and DstMI are mutual exclusive.
        if (G.enablePipeLine()) {
          Latency = G.getStepsToFinish(SrcMI);
          SrcU->addDep(getMemDepEdge(DstU, Latency, 1));
        }
        // Go on handle next visited SUnit.
        continue;
      }

      bool isSrcLoad = VInstrInfo::mayLoad(SrcMI);

      // Ignore RAR dependence.
      if (isDstLoad && isSrcLoad) continue;

      if (!isMachineMemOperandAlias(SrcMO, DstMO, AA, SE))
        continue;

      if (G.enablePipeLine()) {
        assert(IRL && "Can not handle machine loop without IR loop!");
        DEBUG(SrcMI->dump();  dbgs() << "vs\n"; DstMI->dump(); dbgs() << '\n');

        // Dst not depend on Src if they are mutual exclusive.
        if (MayBothActive) {
          // Compute the iterate distance.
          LoopDep LD = analyzeLoopDep(SrcMO, DstMO, isSrcLoad, isDstLoad, *IRL,
                                      true);

          if (LD.hasDep()) {
            unsigned Latency = G.getStepsToFinish(SrcMI);
            VDMemDep *MemDep = getMemDepEdge(SrcU, Latency, LD.getItDst());
            DstU->addDep(MemDep);
          }
        }

        // We need to compute if Src depend on Dst even if Dst not depend on Src.
        // Because dependence depends on execute order, if SrcMI and DstMI are
        // mutual exclusive.
        LoopDep LD = analyzeLoopDep(DstMO, SrcMO, isDstLoad, isSrcLoad, *IRL,
                                    false);

        if (LD.hasDep()) {
          unsigned Latency = G.getStepsToFinish(SrcMI);
          VDMemDep *MemDep = getMemDepEdge(DstU, Latency, LD.getItDst());
          SrcU->addDep(MemDep);
        }
      } else if (MayBothActive) {
        unsigned Latency = G.getStepsToFinish(SrcMI);
        VDMemDep *MemDep = getMemDepEdge(SrcU, Latency, 0);
        DstU->addDep(MemDep);
      }
    }

    // Add the schedule unit to visited map.
    VisitedMemOps.push_back(std::make_pair(DstMO, DstU));
  }
}

unsigned VPreRegAllocSched::getCyclesToBB(const MachineInstr *SrcMI,
                                          const MachineBasicBlock *DstBB) const{
  if (SrcMI->getOpcode() == VTM::VOpMvPhi
      && SrcMI->getOperand(2).getMBB() == DstBB)
    return 0;

  assert(VInstrInfo::isControl(SrcMI->getOpcode())
         && "Expect control instruction!");

  const MachineBasicBlock *SrcBB = SrcMI->getParent();
  unsigned SrcMISlot = 0;
  if (const MachineOperand *SlotMO = VInstrInfo::getTraceOperand(SrcMI)) {
    assert(SlotMO->getTargetFlags() == 0xff && "Instruction not scheduled!");
    SrcMISlot = SlotMO->getImm();
    // Note that the dangling node are not chained with its depending control
    // operations, so for the scheduled instruction that has nozero latency,
    // the result is written to register, so the result will available 1 slot
    // later than it is expected when we are computing the original latency.
    unsigned Opcode = SrcMI->getOpcode();
    if (!VInstrInfo::isCopyLike(Opcode)) ++SrcMISlot;
  } else {
    assert(SrcMI->isPHI() && "Unexpected pseudo instruction type!");
    SrcMISlot = FInfo->getStartSlotFor(SrcBB);
  }

  unsigned SrcMBBEndSlot = FInfo->getEndSlotFor(SrcBB);
  int CyclesToBB = SrcMBBEndSlot - SrcMISlot;
  assert(CyclesToBB >=0 && "Bad schedule!");
  CyclesToBB += getCyclesFromEntry(DstBB) - getCyclesFromEntry(SrcBB);
  assert(CyclesToBB >=0 && "Bad cross BB distance!");
  return CyclesToBB;
}

unsigned VPreRegAllocSched::getCyclesToBB(const MachineBasicBlock *SrcBB,
                                          const MachineBasicBlock *DstBB) const{
  if (SrcBB == DstBB) return 0;
  
  int CyclesToBB = FInfo->getTotalSlotFor(SrcBB);
  CyclesToBB += getCyclesFromEntry(DstBB) - getCyclesFromEntry(SrcBB);
  assert(CyclesToBB >=0 && "Bad cross BB distance!");
  return CyclesToBB;
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

    assert(DefMI->getParent()->back().getOpcode() == VTM::EndState
           && "Unexpected unscheduled MBB!");
    // DefMI write its result at the first slot of MBB, cannot schedule its
    // user to the first slot.
    if (getCyclesToBB(DefMI, MBB) == 0) return 1;
  }

  FuncUnitId Id = VInstrInfo::getPreboundFUId(MI);

  // The not-yet-bound MI will never conflict.
  if (!Id.isBound()) return 0;

  // Check if there is any FU conflict or register conflict.
  typedef MachineBasicBlock::pred_iterator pred_iterator;
  SmallVector<std::pair<MachineBasicBlock*, pred_iterator>, 4> WorkStack;
  SmallPtrSet<MachineBasicBlock*, 4> Visited;
  // No need to check current BB, the scheduler will handle the FU conflicts.
  Visited.insert(MBB);

  WorkStack.push_back(std::make_pair(MBB, MBB->pred_begin()));
  unsigned CurStartSlot = getCyclesFromEntry(MBB);

  while (!WorkStack.empty()) {
    MachineBasicBlock *CurMBB = WorkStack.back().first;
    pred_iterator ChildIt = WorkStack.back().second;

    if (ChildIt == CurMBB->pred_end()) {
      WorkStack.pop_back();
      continue;
    }

    MachineBasicBlock *ChildBB = *ChildIt;
    ++WorkStack.back().second;

    // ChildBB had already been visited.
    if (!Visited.insert(ChildBB)) continue;

    // We assume there is always a conflict in the unscheduled blocks.
    if (ChildBB->back().getOpcode() != VTM::EndState) return 1;

    // Try to detect conflict.
    if (hasFUConflictAtLastSlot(Id, ChildBB)) return 1;

    // Only check the BB whose last slot overlap with the first slot of current
    // MBB.
    if (getCyclesFromEntry(ChildBB) != CurStartSlot) continue;

    WorkStack.push_back(std::make_pair(ChildBB, ChildBB->pred_begin()));
  }

  return 0;
}

bool VPreRegAllocSched::hasFUConflictAtLastSlot(FuncUnitId Id,
                                                MachineBasicBlock *MBB) const {
  // We assume there is always a conflict in the unscheduled blocks.
  if (MBB->back().getOpcode() != VTM::EndState) return true;

  unsigned StartSlot = FInfo->getStartSlotFor(MBB),
           EndSlot = FInfo->getEndSlotFor(MBB),
           II = FInfo->getIIFor(MBB);

  // The FU will never be enabled at the last slot, unless the MBB is pipelined,
  // in which the FU may be enabled in the alias slots of the last slot.
  if (II >= FInfo->getTotalSlotFor(MBB))
    return false;

  EndSlot = StartSlot + (EndSlot - StartSlot) % II;
  unsigned CurSlot = FInfo->getIISlotFor(MBB);

  // Scan the instructions in the MBB to detect the conflict.
  // FIXME: We had better to setup a global FU usage table.
  typedef MachineBasicBlock::reverse_instr_iterator rev_it;
  for (rev_it I = MBB->instr_rbegin(), E = MBB->instr_rend(); I != E; ++I) {
    MachineInstr &MI = *I;
    unsigned Opcode = MI.getOpcode();

    if (MI.isPseudo() || VInstrInfo::isDatapath(Opcode)
        || Opcode == VTM::VOpDisableFU || Opcode == VTM::VOpReadFU)
      continue;

    // Update CurSlot if we are entering a new control bundle.
    if (Opcode == VTM::CtrlEnd) CurSlot = MI.getOperand(0).getImm();

    // Only check the FU conflict at the modulo endslot.
    if (CurSlot < EndSlot) break;
    if (CurSlot > EndSlot) continue;

    // The instruction in the last slot accessing the same FU?
    if (VInstrInfo::getPreboundFUId(&MI) == Id) return true;
  }

  return false;
}

unsigned VPreRegAllocSched::calculateLatencyFromEntry(VSUnit *U) const {
  int Latency = 0;

  for (unsigned i = 0, e = U->num_instrs(); i < e; ++i) {
    // Do not consider positive intra schedule unit latency at the moment.
    int IntraLatency = i ? std::min(int(U->getLatencyAt(i)), 0) : 0;
    int InstLatency = calculateLatencyFromEntry(U->getPtrAt(i));
    Latency = std::max(Latency, InstLatency - IntraLatency);
  }

  return Latency;
}

//===----------------------------------------------------------------------===//
template<typename DepEdgeTy>
void VPreRegAllocSched::addSchedDepForMI(MachineInstr *MI, int MIOffset,
                                         VSUnit *A, VSchedGraph &G,
                                         DepLatInfoTy &LatInfo) {
  assert(MI && "Unexpected entry root!");
  // Dirt
  MIOffset = std::min(MIOffset, 0);
  MachineBasicBlock *CurMBB = MI->getParent();
  // FIXME: If several SrcMIs merged into a same SUnit, we may adding edges
  // from the same source.
  for (src_it I = LatInfo.begin(), E = LatInfo.end(); I != E; ++I) {
    InstPtrTy Src = I->first;
    // Get the latency from SrcMI to MI.
    float DetailLatency = DetialLatencyInfo::getLatency(*I);
    int Latency = int(ceil(DetailLatency));

    // LatencyInfo use a special marker to mark the current MI have some latency
    // from entry of the MBB.
    if (MachineBasicBlock *SrcBB = Src) {
      VSUnit *SrcSU = G.lookupSUnit(SrcBB);
      // Ignore the dependency from other BB in local scheduling mode.
      if (SrcSU == 0) continue;

      if (SrcSU == G.getEntryRoot()) {
        // Since there are some datapath between current schedule unit and the
        // entry node, we cannot schedule current schedule unit to the same slot
        // with the entry root.
        Latency -= getCyclesToBB(SrcBB, CurMBB);
        // Adjust the latency to avoid register/FU conflict.
        Latency = std::max(int(calculateLatencyFromEntry(MI)), Latency);
      }

      Latency -= MIOffset;
      A->addDep(DepEdgeTy::CreateDep(SrcSU, Latency));
      continue;
    }

    MachineInstr *SrcMI = Src.get_mi();
    // Step between MI and its dependent.
    unsigned MinCtrlDistance
      = G.getCtrlStepBetween<DepEdgeTy::IsValDep>(SrcMI, MI);
    // The the latency must bigger than the minimal latency between two control
    // operations.
    Latency = std::max(int(MinCtrlDistance), Latency);
    VSUnit *SrcSU = G.lookupSUnit(SrcMI);

    if (SrcSU == 0) {
      assert(SrcMI->getParent() != CurMBB && "SU for SrcMI not found!");
      // Now SrcMI is from other BasicBlock, try to make use of the steps from
      // SrcMI's schedule to the entry of current BB.
      Latency -= getCyclesToBB(SrcMI, CurMBB);
      // Adjust the latency to avoid register/FU conflict.
      Latency = std::max(int(calculateLatencyFromEntry(MI)), Latency);
      Latency -= MIOffset;
      A->addDep(DepEdgeTy::CreateDep(G.lookupSUnit(CurMBB), Latency));
      continue;
    }

    assert(SrcSU && "Src SUnit not found!");
    assert(SrcSU->isControl() && "Datapath dependence should be forwarded!");
    // Avoid the back-edge or self-edge.
    if (SrcSU->getIdx() >= A->getIdx()) continue;
    // Call getLatencyTo to accumulate the intra-unit latency.
    Latency = SrcSU->getLatencyFrom(SrcMI, Latency);
    Latency -= MIOffset;
    A->addDep(DepEdgeTy::CreateDep(SrcSU, Latency));
  }
}

void VPreRegAllocSched::addIncomingDepForPHI(VSUnit *PHISU, VSchedGraph &G){
  assert(PHISU->isPHI() && "Expect PHI in addIncomingDepForPHI!");
  MachineBasicBlock *CurMBB = G.getEntryBB();

  // Find the incoming copy.
  MachineInstr *IncomingCopy = PHISU->getPtrAt(1);
  assert(IncomingCopy->getOpcode() == VTM::VOpMvPhi && "Expect PHI move!");
  DepLatInfoTy *LatInfo = G.getDepLatInfo(IncomingCopy);
  assert(LatInfo && "Latency information for incoming copy not avaiable!");

  for (src_it I = LatInfo->begin(), E = LatInfo->end(); I != E; ++I) {    
    MachineInstr *SrcMI = const_cast<MachineInstr*>(I->first.dyn_cast_mi());
    // Get the latency from SrcMI to MI.
    float DetailLatency = DetialLatencyInfo::getLatency(*I);
    int Latency = int(ceil(DetailLatency));

    // Simply ignore the edge from entry, it is not an anti-dependence.
    if (SrcMI == 0 || SrcMI->getParent() != CurMBB)
      continue;
    
    VSUnit *SrcSU = G.lookupSUnit(SrcMI);
    assert(SrcSU && "Src SUnit not found!");
    assert(SrcSU->isControl() && "Datapath dependence should be forwarded!");

    // Avoid self-edge.
    if (SrcSU == PHISU) continue;
    
    //assert(SrcSU->getIdx() > PHISU->getIdx() && "Expect back-edge!");

    // Adjust the step between SrcMI and MI.
    Latency = std::max(int(G.getStepsToFinish(SrcMI)), Latency);
    // Call getLatencyTo to accumulate the intra-unit latency.
    Latency = SrcSU->getLatencyFrom(SrcMI, Latency);
    PHISU->addDep(VDMemDep::CreateDep<1>(SrcSU, Latency));
  }
}

void VPreRegAllocSched::addValDep(VSchedGraph &G, VSUnit *A) {
  // Ignore the virtual exit root.
  if (A == G.getExitRoot()) return;

  typedef VSUnit::instr_iterator it;
  bool isCtrl = A->isControl();
  unsigned NumValDep = 0;

  for (unsigned I = 0, E = A->num_instrs(); I < E; ++I) {
    MachineInstr *MI = A->getPtrAt(I);
    int IntraSULatency = I ? A->getLatencyAt(I) : 0;

    assert(MI && "Unexpected entry root!");
    for (unsigned i = 0, e = MI->getNumOperands(); i != e; ++i) {
      MachineInstr *DepSrc = 0;
      const MachineOperand &MO = MI->getOperand(i);
      VSUnit *Dep = getDefSU(MO, G, DepSrc);
      // Avoid self-edge
      if (Dep == 0 || Dep->getIdx() == A->getIdx()) continue;

      // Dirty Hack: Get the detail latency.
      float DetailLatency = G.getChainingLatency(DepSrc, MI);
      DetailLatency += VInstrInfo::getOperandLatency(MI, i);
      // Compute the latency from DepSrc to the repinst of the SU.
      DetailLatency -= std::min(0.0f,
                                IntraSULatency - DetialLatencyInfo::DeltaLatency);
      // All control operations are read at emit, wait until the datapath
      // operations finish if destination is control operation.
      int Latency = isCtrl ? ceil(DetailLatency) : floor(DetailLatency);
      Latency = Dep->getLatencyFrom(DepSrc, Latency);

      // If we got a back-edge, that should be a phinode.
      if (Dep->getIdx() > A->getIdx()) {
        assert(A->getRepresentativePtr().get_mi()->isPHI()
               && "Expected backedge for PHI!");
        // Cross iteration dependences do not make sense in normal loops.
        if (G.isPipelined())
          // The iterate distance for back-edge to PHI is always 1.
          A->addDep(VDMemDep::CreateDep<1>(Dep, Latency));
        else {
          // Else connect the schedule unit to exit root, since it is not
          // dangling.
          VSUnit *CurTerminator = G.lookUpTerminator(A->getParentBB());
          CurTerminator->addDep(VDCtrlDep::CreateDep(Dep, Latency));
        }
      } else {
        A->addDep(VDValDep::CreateDep(Dep, Latency));
        ++NumValDep;
      }
    }
  }

  // If the atom depend on nothing and it must has some dependence edge,
  // make it depend on the entry node.
  if (NumValDep == 0) {
    A->addDep(VDCtrlDep::CreateDep(G.lookupSUnit(A->getParentBB()), 0));
    return;
  }

  // For pipelined loop, take care of the Anti-dependence from PHI.
  if (G.isPipelined() && !isCtrl) {
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

void VPreRegAllocSched::addSchedDepForSU(VSUnit *A, VSchedGraph &G,
                                         bool isExit) {
  // Build the dependence edge.
  typedef VSUnit::instr_iterator it;
  assert(A->isControl() && "Unexpected data-path schedule unit!");
  for (unsigned I = 0, E = A->num_instrs(); I != E; ++I) {
    MachineInstr *MI = A->getPtrAt(I);    
    assert(MI && "Unexpected entry root!");
    const DetialLatencyInfo::DepLatInfoTy *DepLat = G.getDepLatInfo(MI);
    assert(DepLat && "Operand latency information not available!");
    addSchedDepForMI<VDValDep>(MI, I ? A->getLatencyAt(I) : 0, A, G,
                               *DepLat);
  }

  // If the atom depend on nothing and it must has some dependence edge,
  // make it depend on the entry node.
  if (A->dep_empty() && !isExit) {
    VSUnit *BBEntry = G.lookupSUnit(A->getParentBB());
    unsigned Latency = 0;
    if (BBEntry == G.getEntryRoot())
      Latency = calculateLatencyFromEntry(A);

    A->addDep(VDCtrlDep::CreateDep(BBEntry, Latency));
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

void VPreRegAllocSched::buildPipeLineDepEdges(VSchedGraph &G,
                                              MachineBasicBlock *CurBB) {
  VSUnit *LoopOp = G.getLoopOp();
  assert(LoopOp && "Not in loop?");
  VSUnit *CurTerminator = G.lookUpTerminator(CurBB);
  assert(LoopOp != CurTerminator && "Pipeline not enable!");

  for (instr_it I = CurBB->begin(), E = CurBB->end();I != E && I->isPHI(); ++I) {
    MachineInstr &PN = *I;
    VSUnit *PHISU = G.lookupSUnit(&PN);
    assert(PHISU && "Can not find SUnit for PHI!");

    // Add dependence from PHI incoming value:
    // PHI_incoming -(RAW dep)-> PHI_at_next_iteration.
    addIncomingDepForPHI(PHISU, G);
    // Add a anti-dependence edge from users of PHI to PHI because we must
    // have:
    // PHI -(RAW dep)-> PHI_user -(WAR dep)-> PHI_at_next_iteration.
    typedef VSUnit::use_iterator use_it;
    for (use_it UI = PHISU->use_begin(), UE = PHISU->use_end(); UI != UE; ++UI){
      VSUnit *PHIUser = *UI;
      if (PHIUser != CurTerminator)
        PHISU->addDep(getMemDepEdge(PHIUser, 0, 1));
    }

    // Add the dependence edge PHI -> Loop back -> PHI_at_iteration.
    PHISU->addDep(getMemDepEdge(LoopOp, 0, 1));
    //LoopOp->addDep(VDValDep::CreateValDep(PHISU, 0));
  }
}

bool VPreRegAllocSched::mergeUnaryOp(MachineInstr *MI, unsigned OpIdx,
                                     VSchedGraph &G) {
  MachineInstr *SrcMI = 0;
  // Try to merge it into the VSUnit that defining its source operand.
  if (VSUnit *SrcSU = getDefSU(MI->getOperand(OpIdx), G, SrcMI))
    return G.mapMI2SU(MI, SrcSU, SrcSU->getLatencyTo<true>(SrcMI, MI, G));

  // Try to merge it into the VSUnit that defining its predicate operand.
  if (const MachineOperand *Pred = VInstrInfo::getPredOperand(MI))
    if (VSUnit *SrcSU = getDefSU(*Pred, G, SrcMI))
      return G.mapMI2SU(MI, SrcSU, SrcSU->getLatencyTo<true>(SrcMI, MI, G));

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

void VPreRegAllocSched::buildSUnit(MachineInstr *MI,  VSchedGraph &G) {
  assert(!MI->getDesc().isTerminator() && "Unexpected terminator!");

  switch (MI->getOpcode()) {
  default: break;
  case VTM::VOpMove:
    if (mergeUnaryOp(MI, 1, G))
      return;
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
    return;
  }
  case VTM::PHI:
    // Merge the the PHI into entry root if the BB is not pipelined.
    if (!G.enablePipeLine()) {
      G.mapMI2SU(MI, G.lookupSUnit(MI->getParent()),
                        G.getStepsFromEntry(MI));
      return;
    }
    break;
  case VTM::VOpReadReturn:
    if (mergeUnaryOp(MI, 1, G))
      return;
    break;
  case VTM::VOpMoveArg:
    G.mapMI2SU(MI, G.getEntryRoot(), 0);
    return;
  // The VOpDstMux should be merged to its user.
  case VTM::VOpDstMux: return;
  case VTM::VOpMvPhi:
    if (G.isLoopPHIMove(MI)) {
      unsigned Reg = MI->getOperand(0).getReg();
      assert(MRI->hasOneUse(Reg) && "Incoming copy has more than one use?");
      MachineInstr *PN = &*MRI->use_begin(Reg);
      assert(PN && PN->isPHI() && "Bad user of incoming copy!");
      VSUnit *PHISU = G.lookupSUnit(PN);
      assert(PHISU && "Schedule unit for PHI node not found!");
      G.mapMI2SU(MI, PHISU, 0);
    }
    return;
  }

  // TODO: Remember the register that live out this MBB.
  // and the instruction that only produce a chain.
  FuncUnitId Id = VInstrInfo::getPreboundFUId(MI);
  VSUnit *U = G.createVSUnit(MI, Id.getFUNum());
  if (Id.isBound()) mergeDstMux(U, G);
}

template<int AllowDangling>
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
    if (VSU->getNumUses() == 0 && VSU != Terminator) {
      if (VSU->isDatapath()) continue;

      if (!(AllowDangling || G.isLoopOp(VSU->getRepresentativePtr())))
        llvm_unreachable("Unexpected handing node!");

      // A PHIMove can be scheduled to the same slot with the exit root.
      unsigned Latency = VSU->getMaxLatencyTo<false>(ExitMI, G);
      // We do not need to wait the trivial operation finish before exiting the
      // state, because the first control slot of next state will only contains
      // PHI copies, and the PHIElimination Hook will take care of the data
      // dependence and try to forward the wire value in last control slot
      // if possible, so they can take the time of the last control slot.
      //VIDesc VID(*Instr);
      //if (VID.hasTrivialFU() && !VID.hasDatapath() && Latency)
      //  --Latency;

      Terminator->addDep(VDCtrlDep::CreateDep(VSU,  Latency));
    }
  }
}

void VPreRegAllocSched::scheduleDanglingDatapathOps(VSchedGraph &G){
  typedef VSchedGraph::sched_iterator sched_it;

  unsigned DanglingStep = G.getEndSlot();

  // Schedule all dangling nodes to dangling step.
  for (sched_it I = G.sched_begin(), E = G.sched_end(); I != E; ++I) {
    VSUnit *U = *I;

    if (!U->isDangling()) continue;

    assert(U->isDatapath() && "Unexpected dangling control operation.");
    U->scheduledTo(DanglingStep);
    ++DanglingDatapath;
  }
}

void VPreRegAllocSched::clearDanglingFlagForTree(VSUnit *Root) {
  // Perform depth first search to find node that reachable from Root.
  std::vector<std::pair<VSUnit*, VSUnit::dep_iterator> > WorkStack;
  WorkStack.push_back(std::make_pair(Root, Root->dep_begin()));
  Root->setIsDangling(false);

  while (!WorkStack.empty()) {
    VSUnit *U = WorkStack.back().first;
    VSUnit::dep_iterator ChildIt = WorkStack.back().second;

    if (ChildIt == U->dep_end()) {
      WorkStack.pop_back();
      continue;
    }

    VSUnit *ChildNode = *ChildIt;
    ++WorkStack.back().second;

    if (!ChildNode->isDangling()) continue;

    // If the node is reachable from exit, then it is not dangling.
    ChildNode->setIsDangling(false);
    WorkStack.push_back(std::make_pair(ChildNode, ChildNode->dep_begin()));
  }
}

void VPreRegAllocSched::addDepsForBBEntry(VSchedGraph &G, VSUnit *EntrySU) {
  MachineBasicBlock *MBB = EntrySU->getRepresentativePtr();
  assert(MBB && "Bad entry node type!");

  typedef MachineBasicBlock::pred_iterator pred_iterator;
  for (pred_iterator I = MBB->pred_begin(), E = MBB->pred_end(); I != E; ++I) {
    VSUnit *PredTerminator = G.lookUpTerminator(*I);
    // Ignore the terminator that is not yet created, which mean we are ignoring
    // the loop back edges.
    if (PredTerminator) EntrySU->addDep(VDCtrlDep::CreateDep(PredTerminator, 0));    
  }
}

void VPreRegAllocSched::buildExitRoot(VSchedGraph &G,
                                      MachineInstr *FirstTerminator) {
  SmallVector<MachineInstr*, 8> Exits;
  // We need wait all operation finish before the exit operation active, compute
  // the latency from operations need to wait to the exit operation.
  DetialLatencyInfo::DepLatInfoTy ExitDepInfo;
  MachineBasicBlock *MBB = FirstTerminator->getParent();

  for (instr_it I = FirstTerminator, E = MBB->end(); I != E; ++I) {
    MachineInstr *MI = I;
    if (!I->isTerminator()) {
      assert(MI->getOpcode() == VTM::VOpMvPhi && "Bad MBB!");
      continue;
    }

    // Compute the dependence information.
    G.addInstr(MI);

    // Build the schedule unit for loop back operation.
    if (G.isLoopOp(I)) {
      VSUnit *LoopOp = G.createVSUnit(MI);
      addSchedDepForSU(LoopOp, G);
      continue;
    }
  }
  
  VSUnit *ExitSU = G.createTerminator(MBB);
  for (instr_it I = FirstTerminator, E = MBB->end(); I != E; ++I) {
    MachineInstr *MI = I;
    // Ignore the VOpMvPhis, which are handled, and also ignore the looping-back
    // terminator, which had been handled in the previous loop.
    if (!I->isTerminator() || G.isLoopOp(I)) continue;

    // No need to wait the terminator.
    G.eraseFromWaitSet(MI);

    // Build a exit root or merge the terminators into the exit root.
    bool mapped = G.mapMI2SU(MI, ExitSU, 0);
    (void) mapped;
    assert(mapped && "Cannot merge terminators!");
  }

  assert(ExitSU && "Terminator not found?");

  for (instr_it I = FirstTerminator, E = MBB->end(); I != E; ++I) {
    MachineInstr *MI = I;

    if (MI->getOpcode() == VTM::VOpMvPhi) {
      if (G.isLoopPHIMove(MI)) {
        // Forward the edges to exit root, so the dependences of PHI moves
        // can always finish in time.
        const DetialLatencyInfo::DepLatInfoTy *DepLat = G.getDepLatInfo(MI);
        assert(DepLat && "Operand latency information not available!");
        addSchedDepForMI<VDCtrlDep>(MI, 0/*Offset*/, ExitSU, G, *DepLat);
        // Add the dependence from PHISU to ExitSU, we will constraint the PHI
        // so it will schedule before the last stage of a pipeline BB.
        VSUnit *PHISU = G.lookupSUnit(MI);
        assert(PHISU->isPHI() && "Expect PHISU merged by PHI!");
        ExitSU->addDep(VDCtrlDep::CreateDep(PHISU, 0));
        continue;
      } else { // Also merge the PHI moves.
        bool mapped = G.mapMI2SU(MI, ExitSU, 0);
        (void) mapped;
        assert(mapped && "Cannot merge terminators!");
      }

      // No need to wait VOpMvPhi, because we are going to merge it into the
      // exit root.
      G.eraseFromWaitSet(MI);
    } else if (G.isLoopOp(I))
      continue;

    // Build datapath latency information for the terminator.
    G.buildExitMIInfo(MI, ExitDepInfo);
  }

  // Add the dependence of exit root.
  addSchedDepForSU(ExitSU, G, true);

  // Add the control dependence edge edges to wait all operation finish.
  addSchedDepForMI<VDCtrlDep>(ExitSU->getRepresentativePtr(), 0/*Offset*/,
                              ExitSU, G, ExitDepInfo);

  // If we have a trivial schedule graph that only containing entry and exit
  // simply connect them together.
  VSUnit *Entry = G.getEntryRoot();
  if (Entry->use_empty()) {
    unsigned L = calculateLatencyFromEntry(ExitSU);
    ExitSU->addDep(VDCtrlDep::CreateDep(Entry, L));
  }

  // If there is still schedule unit not connect to exit, connect it now, but
  // they are supposed to be connected in the previous stages, so dangling node
  // is not allow.
  buildTerminatorDeps<false>(G, ExitSU);
}

void VPreRegAllocSched::buildControlPathGraph(VSchedGraph &G,
                                              MachineBasicBlock *MBB) {
  instr_it BI = MBB->begin();
  while(!BI->isTerminator() && BI->getOpcode() != VTM::VOpMvPhi) {
    MachineInstr *MI = BI;    
    G.addInstr(MI);
    buildSUnit(MI, G);
    ++BI;
  }
  G.removeDeadSU();

  // Make sure every VSUnit have a dependence edge except EntryRoot.
  typedef VSchedGraph::iterator it;
  for (it I = G.begin() + 1, E = G.end(); I != E; ++I)
    if ((*I)->isControl()) addSchedDepForSU(*I, G);

  // Merge the loop PHI moves into the PHI Node, after the intra iteration
  // dependence edge added. If we merge the PHI moves into the PHI schedule
  // units before we adding intra iteration edges, the dependences of the PHI
  // moves are added to the PHI schedule unit as intra iteration dependences,
  // which is incorrect, all dependences of a PHI should be inter iteration
  // dependences.
  for (instr_it I = BI; !I->isTerminator(); ++I) {
    G.addInstr(I);

    if (G.isLoopPHIMove(I)) buildSUnit(I, G);
  }

  // Create the exit node, now BI points to the first terminator.
  buildExitRoot(G, BI);

  // Build loop edges if necessary.
  if (G.enablePipeLine()) buildPipeLineDepEdges(G, MBB);

  // Build the memory edges.
  buildMemDepEdges(G, MBB);
}

void VPreRegAllocSched::buildDataPathGraph(VSchedGraph &G) {
  G.prepareForDatapathSched();
  for (su_it I = G.begin() + 1, E = G.end(); I != E; ++I)
    addValDep(G, *I);

  // Clear the dangling flag for all node that used (directly/indirectly) by
  // a scheduled node.
  for (su_it I = G.begin(), E = G.end(); I != E; ++I) {
    VSUnit *U = *I;
    if (U->isScheduled()) clearDanglingFlagForTree(U);
  }

  // Build control dependence for exitroot, dangling node is allowed because we
  // do not handle them explicitly.
  buildTerminatorDeps<true>(G, G.lookUpTerminator(G.getEntryBB()));

  scheduleDanglingDatapathOps(G);

  // Verify the schedule graph.
  G.verify();
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
