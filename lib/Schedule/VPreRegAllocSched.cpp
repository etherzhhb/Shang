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

#include "vtm/Passes.h"
#include "vtm/VFInfo.h"
#include "vtm/BitLevelInfo.h"
#include "vtm/VTM.h"

#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/Analysis/ScalarEvolution.h"
#include "llvm/Analysis/ScalarEvolutionExpressions.h"
#include "llvm/Analysis/ValueTracking.h"

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

  // Terminators in a MBB.
  SmallVector<MachineInstr*, 2> Terms;

  VPreRegAllocSched() : MachineFunctionPass(ID), totalCycle(1) {}

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

  VDValDep *getValDepEdge(VSUnit *Src, unsigned Latency) {
    return new VDValDep(Src, Latency);
  }

  VDCtrlDep *getCtrlDepEdge(VSUnit *Src, unsigned Latency) {
    return new VDCtrlDep(Src, Latency);
  }

  VDMemDep *getMemDepEdge(VSUnit *Src, unsigned Latency, unsigned Diff) {
    return new VDMemDep(Src, Latency, Diff);
  }

  void addValueDeps(VSUnit *A, VSchedGraph &CurState);

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
    /// Only add the dependence if DepSrc is in the same MBB with MI.
    return CurState.lookupSUnit(DepSrc);
  }

  void clear();

  void buildMemDepEdges(VSchedGraph &CurState);

  bool couldBePipelined(const MachineBasicBlock *MBB);
  void buildPipeLineDepEdges(VSchedGraph &State);
  void buildState(VSchedGraph &State);
  void buildExitRoot(VSchedGraph &CurState);
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

  void incTotalCycle() { ++totalCycle; }

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
  AU.addPreserved<BitLevelInfo>();
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
    buildState(State);
    DEBUG(State.viewGraph());
    State.schedule();
    setTotalCycle(State.getEndSlot() + 1);
    DEBUG(State.viewGraph());
    State.emitSchedule();
  }

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

void VPreRegAllocSched::buildMemDepEdges(VSchedGraph &CurState) {
  CurState.preSchedTopSort();
  // The schedule unit and the corresponding memory operand.
  typedef std::vector<std::pair<Value*, VSUnit*> > MemOpMapTy;
  MemOpMapTy VisitedMemOps;
  Loop *IRL = IRLI->getLoopFor(CurState.getMachineBasicBlock()->getBasicBlock());

  for (VSchedGraph::iterator I = CurState.begin(), E = CurState.end(); I != E;
       ++I) {
    VSUnit *DstU = *I;
    MachineInstr *DstMI = DstU->getRepresentativeInst();
    // Skip the non-memory operation.
    if (!DstMI || DstMI->memoperands_empty())
      continue;

    // FIXME:
    assert(DstMI->hasOneMemOperand() && "Can not handle multiple mem operand!");
    assert(!DstMI->hasVolatileMemoryRef() && "Can not handle volatile operation!");

    // Dirty Hack: Is the const_cast safe?
    Value *DstMO = const_cast<Value*>((*DstMI->memoperands_begin())->getValue());
    const Type *DstElemTy = cast<SequentialType>(DstMO->getType())->getElementType();
    size_t DstSize = TD->getTypeStoreSize(DstElemTy);

    VIDesc DstInfo = *DstMI;
    bool isDstLoad = DstInfo.mayLoad();

    if (DstMO == 0) continue;

    assert(!isa<PseudoSourceValue>(DstMO) && "Unexpected frame stuffs!");

    for (MemOpMapTy::iterator I = VisitedMemOps.begin(), E = VisitedMemOps.end();
         I != E; ++I) {
      Value *SrcMO = I->first;
      const Type *SrcElemTy = cast<SequentialType>(SrcMO->getType())->getElementType();
      size_t SrcSize = TD->getTypeStoreSize(SrcElemTy);
      
      VSUnit *SrcU = I->second;
      MachineInstr *SrcMI = SrcU->getRepresentativeInst();
      VIDesc SrcInfo = *SrcMI;
      bool isSrcLoad = SrcInfo.mayLoad();
      
      // Ignore RAR dependence.
      if (isDstLoad && isSrcLoad) continue;

      if (CurState.enablePipeLine()) {
        assert(IRL && "Can not handle machine loop without IR loop!");
        // Compute the iterate distance.
        LoopDep LD = analyzeLoopDep(SrcMO, DstMO, isSrcLoad, isDstLoad, *IRL, true);

        if (LD.hasDep()) {
          unsigned Latency = VInstrInfo::computeLatency(SrcMI, DstMI);
          VDMemDep *MemDep = getMemDepEdge(SrcU, Latency, LD.getItDst());
          DstU->addDep(MemDep);
        }

        // We need to compute if Src depend on Dst even if Dst not depend on Src.
        // Because dependence depends on execute order.
        LD = analyzeLoopDep(DstMO, SrcMO, isDstLoad, isSrcLoad, *IRL, false);

        if (LD.hasDep()) {
          unsigned Latency = VInstrInfo::computeLatency(SrcMI, DstMI);
          VDMemDep *MemDep = getMemDepEdge(DstU, Latency, LD.getItDst());
          SrcU->addDep(MemDep);
        }
      } else {
        if (AA->isNoAlias(SrcMO, SrcSize, DstMO, DstSize)) continue;

        // Ignore the No-Alias pointers.
        unsigned Latency = VInstrInfo::computeLatency(SrcMI, DstMI);
        VDMemDep *MemDep = getMemDepEdge(SrcU, Latency, 0);
        DstU->addDep(MemDep);
      }
    }

    // Add the schedule unit to visited map.
    VisitedMemOps.push_back(std::make_pair(DstMO, DstU));
  }
}

//===----------------------------------------------------------------------===//

void VPreRegAllocSched::addValueDeps(VSUnit *A, VSchedGraph &CurState) {
  std::map<VSUnit*, unsigned> Edges;
  // Collect the dependence information.
  for (VSUnit::instr_iterator I = A->instr_begin(), E = A->instr_end();
       I != E; ++I)
    if (MachineInstr *MI = *I) {
      for (unsigned i = 0, e = MI->getNumOperands(); i != e; ++i) {
        MachineInstr *DepSrc = 0;
        VSUnit *Dep = getDefSU(MI->getOperand(i), CurState, DepSrc);
        // Avoid back-edge and self-edge
        if (Dep == 0 || Dep->getIdx() >= A->getIdx()) continue;

        unsigned Latency = Dep->getLatencyTo(DepSrc, MI);

        unsigned &DepLatency = Edges[Dep];
        DepLatency = std::max(DepLatency, Latency);
      }
    }

  // Build the dependence edge.
  for (std::map<VSUnit*, unsigned>::iterator I = Edges.begin(), E = Edges.end();
       I != E; ++I)
    A->addDep(getValDepEdge(I->first, I->second));

  // If the atom depend on nothing, make it depend on the entry node.
  if (A->dep_empty()) {
    unsigned Latency =
      VInstrInfo::computeLatency(0, A->getRepresentativeInst());
    A->addDep(getValDepEdge(CurState.getEntryRoot(), Latency));
  }
}

bool VPreRegAllocSched::couldBePipelined(const MachineBasicBlock *MBB) {
  MachineLoop *L = LI->getLoopFor(MBB);
  // Not in any loop.
  if (!L) return false;
  // Dirty Hack: Only support one block loop at this moment.
  if (L->getBlocks().size() != 1) return false;

  return FInfo->getInfo().enablePipeLine();
}

void VPreRegAllocSched::buildPipeLineDepEdges(VSchedGraph &State) {
  // Only work on pipelined loops.
  if (!State.enablePipeLine()) return;

  VSUnit *SelfEnable = State.getLoopOp();
  assert(SelfEnable && "Not in loop?");
  assert(SelfEnable != State.getExitRoot() && "Pipeline not enable!");
  // Insert the dependences between successive iterations.
  //VSUnit *Entry = State.getEntryRoot();
  //Entry->addDep(getMemDepEdge(SelfEnable, 0, true, VDMemDep::AntiDep, 1));

  MachineBasicBlock *CurBB = State.getMachineBasicBlock();
  // For each phinode
  for (MachineBasicBlock::iterator I = CurBB->begin(), E = CurBB->getFirstNonPHI();
       I != E; ++I) {
    MachineInstr &PN = *I;
    VSUnit *PhiSU = State.lookupSUnit(&PN);
    assert(PN.isPHI() && "IsSingleValuePHICycle expects a PHI instruction!");
    assert(PhiSU && "Can not find SUnit for PHI!");

    // Add a anti-dependence edge from users of PHI to PHI because we must
    // have:
    // PHI -(RAW dep)-> PHI user -(WAR dep)-> PHI at next iteration.
    unsigned PHIReg = PN.getOperand(0).getReg();
    typedef MachineRegisterInfo::use_iterator use_it;
    for (use_it I = MRI->use_begin(PHIReg), E = MRI->use_end(); I != E; ++I) {
      MachineInstr *UserMI = &*I;
      if (UserMI->getParent() != CurBB) continue;

      VSUnit *UserSU = State.lookupSUnit(UserMI);
      assert(UserSU && "Cannot found UserSU!");
      unsigned Latency = VInstrInfo::computeLatency(UserMI, &PN);
      PhiSU->addDep(getMemDepEdge(UserSU, Latency, 1));
    }

    // Scan the PHI operands.
    for (unsigned i = 1; i != PN.getNumOperands(); i += 2) {
      MachineBasicBlock *SrcBB = PN.getOperand(i + 1).getMBB();
      // Only handle the self loop edge.
      if (SrcBB != CurBB) continue;

      unsigned SrcReg = PN.getOperand(i).getReg();
      MachineInstr *SrcMI = MRI->getVRegDef(SrcReg);
      assert(!SrcMI->isPHI() && "PHI chain not supported yet!");
      VSUnit *InSU = State.lookupSUnit(SrcMI);
      assert(InSU && "Where's the incoming value of the phi?");

      // Insert anti-dependence edge.
      unsigned Latency = VInstrInfo::computeLatency(SrcMI, &PN);
      PhiSU->addDep(getMemDepEdge(InSU, Latency, 1));
    }

    // Next loop can not start before loop operation issued.
    VSUnit *LoopOp = State.getLoopOp();
    unsigned Latency =
      VInstrInfo::computeLatency(LoopOp->getRepresentativeInst(), &PN);
    PhiSU->addDep(getMemDepEdge(LoopOp, Latency, 1));
  }
}

bool VPreRegAllocSched::mergeUnaryOp(MachineInstr *MI, unsigned OpIdx,
                                     VSchedGraph &CurState) {
  MachineInstr *SrcMI = 0;
  // Try to merge it into the VSUnit that defining its source operand.
  if (VSUnit *SrcSU = getDefSU(MI->getOperand(OpIdx), CurState, SrcMI)) {
    CurState.mapMI2SU(MI, SrcSU, SrcSU->getLatencyTo(SrcMI, MI));
    return true;
  }

  // Try to merge it into the VSUnit that defining its predicate operand.
  if (const MachineOperand *Pred = VInstrInfo::getPredOperand(MI)) {
    if (VSUnit *SrcSU = getDefSU(*Pred, CurState, SrcMI)) {
      CurState.mapMI2SU(MI, SrcSU, SrcSU->getLatencyTo(SrcMI, MI));
      return true;
    }
  }

  // Merge it into the EntryRoot.
  CurState.mapMI2SU(MI, CurState.getEntryRoot(),
                    VInstrInfo::computeLatency(0, MI));
  return true;
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
    if (LHSSU == 0) LHSSU = RHSSU = CurState.getEntryRoot();

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

  return false;
}

void VPreRegAllocSched::buildSUnit(MachineInstr *MI,  VSchedGraph &CurState) {
  // If the current instruction was eaten by as terminator?
  if (CurState.eatTerminator(MI)) {
    Terms.push_back(MI);
    return;
  }

  switch (MI->getOpcode()) {
  default: break;
  case VTM::VOpBitSlice:
  case VTM::VOpBitRepeat:
  case VTM::VOpMove_ra:
  case VTM::VOpMove_ri:
  case VTM::VOpMove_rm:
  case VTM::VOpMove_rs:
  case VTM::VOpMove_rw:
    if (mergeUnaryOp(MI, 1, CurState))
      return;
    break;
  case VTM::VOpReadReturn:
    if (mergeUnaryOp(MI, 2, CurState))
      return;
    break;
  case VTM::VOpBitCat:
    if (mergeBitCat(MI, CurState))
      return;
    break;
  }

  VIDesc VTID = *MI;
  FuncUnitId Id = VTID.getPrebindFUId();

  // TODO: Remember the register that live out this MBB.
  // and the instruction that only produce a chain.
  CurState.createVSUnit(MI, Id.getFUNum());
}

void VPreRegAllocSched::buildExitRoot(VSchedGraph &CurState) {
  MachineInstr *FstExit = Terms.front();
  VSUnit *Exit = CurState.createVSUnit(FstExit);

  // Add others terminator to the exit node.
  while (Terms.size() != 1) {
    MachineInstr *Term = Terms.pop_back_val();
    CurState.mapMI2SU(Term, Exit, 0);
  }
  Terms.clear();

  addValueDeps(Exit, CurState);

  for (VSchedGraph::iterator I = CurState.begin(), E = CurState.end();
       I != E; ++I) {
    VSUnit *VSU = *I;
      // Since the exit root already added to state sunit list, skip the
      // exit itself.
    if (VSU->getNumUses() == 0 && !VSU->isEntry() && VSU != Exit) {
      // Dirty Hack.
      MachineInstr *Instr = VSU->getRepresentativeInst();
      unsigned Latency = VInstrInfo::computeLatency(Instr, FstExit);
      // We do not need to wait the trivial operation finish before exiting the
      // state, because the first control slot of next state will only contains
      // PHI copies, and the PHIElimination Hook will take care of the data
      // dependence and try to forward the wire value in last control slot
      // if possible, so they can take the time of the last control slot.
      //VIDesc VID(*Instr);
      //if (VID.hasTrivialFU() && !VID.hasDatapath() && Latency)
      //  --Latency;

      Exit->addDep(getCtrlDepEdge(VSU,  Latency));
    }
  }

  // Do not forget the entry root.
  Exit->addDep(getCtrlDepEdge(CurState.getEntryRoot(),
                              VInstrInfo::computeLatency(0, FstExit)));
}

void VPreRegAllocSched::buildState(VSchedGraph &State) {
  for (MachineBasicBlock::iterator BI = State->begin(), BE = State->end();
      BI != BE; ++BI)
    buildSUnit(&*BI, State);

  State.removeDeadSU();

  // Make sure every VSUnit have a dependence edge except EntryRoot.
  for (VSchedGraph::iterator I = ++State.begin(), E = State.end(); I != E; ++I)
    addValueDeps(*I, State);

  assert(!Terms.empty() && "Can not found any terminator!");
  // Create the exit node.
  buildExitRoot(State);

  // Build loop edges if necessary.
  buildPipeLineDepEdges(State);

  // Build the memory edges.
  buildMemDepEdges(State);
}

void VPreRegAllocSched::cleanUpSchedule() {
  cleanUpRegisterClass(VTM::DRRegisterClass);
}

void VPreRegAllocSched::cleanUpRegisterClass(const TargetRegisterClass *RC) {
  // And Emit the wires defined in this module.
  const std::vector<unsigned>& Wires = MRI->getRegClassVirtRegs(RC);

  for (std::vector<unsigned>::const_iterator I = Wires.begin(), E = Wires.end();
       I != E; ++I) {
    unsigned SrcReg = *I;
    MachineRegisterInfo::def_iterator DI = MRI->def_begin(SrcReg);

    if (DI == MRI->def_end() || DI->isPHI() || !MRI->use_empty(SrcReg))
      continue;

    assert(++MRI->def_begin(SrcReg) == MRI->def_end() && "Not in SSA From!");

    MachineInstr *DefMI = &*DI;
    ucOp Op = ucOp::getParent(DI);
    assert(Op->getOpcode() == VTM::COPY && "Unexpected opcode!");
    unsigned OpStart = DI.getOperandNo() - 2;
    // FIXME: Unlink the instruction from parent before remove the operand.
    for (unsigned i = 0, e = TII->get(VTM::COPY).getNumOperands() + 2;
         i != e; ++i)
      DefMI->RemoveOperand(OpStart);
  }
}
