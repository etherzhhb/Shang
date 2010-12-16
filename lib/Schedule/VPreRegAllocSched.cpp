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
#include "ForceDirectedScheduling.h"
//#include "MemDepAnalysis.h"
#include "vtm/BitLevelInfo.h"
#include "vtm/Passes.h"
#include "vtm/VFuncInfo.h"
#include "vtm/VTargetMachine.h"
#include "vtm/VTM.h"

#include "llvm/Analysis/AliasAnalysis.h"

#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/MachineLoopInfo.h"
#include "llvm/CodeGen/LiveVariables.h"
#include "llvm/CodeGen/PseudoSourceValue.h"

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
  // The loop Info
  MachineLoopInfo *LI;
  LiveVariables *LiveVars;
  const VTargetMachine &VTarget;
  MachineRegisterInfo *MRI;
  VFuncInfo *FuncInfo;
  BitLevelInfo *BLI;
  AliasAnalysis *AA;
  TargetData *TD;

  // Total states
  // Cycle is start from 1 because  cycle 0 is reserve for idle state.
  unsigned short totalCycle;

  unsigned computeLatency(const MachineInstr *SrcInstr,
                          const MachineInstr *DstInstr) {
    if (SrcInstr == 0) {
      // DirtyHack: There must be at least 1 slot between entry and exit.
      if (DstInstr && VTFInfo(*DstInstr)->isTerminator()) return 1;

      return 0;
    }

    VTFInfo SrcTID = *SrcInstr;
    unsigned latency = SrcTID.getLatency();

    if (DstInstr == 0) return latency;

    VTFInfo DstTID = *DstInstr;

    if (DstTID.isReadAtEmit()) {
      // We need to wait one more slot to read the result.
      if (latency == 0) return 1;

      if (SrcTID.isWriteUntilFinish()) return latency + 1;
    }
    
    return latency;
  }

  VDValDep *getValDepEdge(VSUnit *Src, unsigned Latency, bool isSigned = false,
                          enum VDValDep::ValDepTypes T = VDValDep::Normal) {
    return new VDValDep(Src, Latency, isSigned, T);
  }

  VDCtrlDep *getCtrlDepEdge(VSUnit *Src, unsigned Latency) {
    return new VDCtrlDep(Src, Latency);
  }

  VDMemDep *getMemDepEdge(VSUnit *Src, unsigned Latency, bool isBackEdge,
                          enum VDMemDep::MemDepTypes DepType,
                          unsigned Diff);

  void addValueDeps(VSUnit *A, VSchedGraph &CurState);

  void clear();

  void buildMemDepEdges(VSchedGraph &CurState);

  bool couldBePipelined(const MachineBasicBlock *MBB);
  void buildPipeLineDepEdges(VSchedGraph &State);
  void buildState(VSchedGraph &State);
  void buildExitRoot(VSchedGraph &CurState);
  void buildSUnit(MachineInstr *MI, VSchedGraph &CurState);

  /// @name FunctionPass interface
  //{
  static char ID;
  VPreRegAllocSched(const VTargetMachine &TM);

  ~VPreRegAllocSched();
  bool runOnMachineFunction(MachineFunction &MF);

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
};
}

char VPreRegAllocSched::ID = 0;

Pass *llvm::createVPreRegAllocSchedPass(const VTargetMachine &TM) {
  return new VPreRegAllocSched(TM);
}

void VPreRegAllocSched::getAnalysisUsage(AnalysisUsage &AU) const {
  MachineFunctionPass::getAnalysisUsage(AU);
  AU.addRequired<LiveVariables>();
  AU.addRequired<MachineLoopInfo>();
  AU.addRequired<BitLevelInfo>();
  AU.addRequired<AliasAnalysis>();
  // AU.addRequired<MemDepInfo>();
  AU.setPreservesCFG();
  AU.addPreserved<BitLevelInfo>();
}

bool VPreRegAllocSched::runOnMachineFunction(MachineFunction &MF) {
  LiveVars = &getAnalysis<LiveVariables>();
  MRI = &MF.getRegInfo();
  FuncInfo = MF.getInfo<VFuncInfo>();
  BLI = &getAnalysis<BitLevelInfo>();
  AA = &getAnalysis<AliasAnalysis>();
  LI = &getAnalysis<MachineLoopInfo>();

  for (MachineFunction::iterator I = MF.begin(), E = MF.end();
       I != E; ++I) {
    MachineBasicBlock *MBB = &*I;

    VSchedGraph State(VTarget, MBB, couldBePipelined(MBB), getTotalCycle());

    buildState(State);
    DEBUG(State.viewGraph());
    State.schedule();
    setTotalCycle(State.getEndSlot() + 1);
    DEBUG(State.viewGraph());

    State.emitSchedule(*BLI);
    FuncInfo->remeberTotalSlot(MBB, State.getStartSlot(),
                                    State.getTotalSlot(),
                                    State.getIISlot());
  }

  return false;
}


bool VPreRegAllocSched::couldBePipelined(const MachineBasicBlock *MBB) {
  MachineLoop *L = LI->getLoopFor(MBB);
  // Not in any loop.
  if (!L) return false;
  // Dirty Hack: Only support one block loop at this moment.
  if (L->getBlocks().size() != 1) return false;

  return FuncInfo->getConstraints().enablePipeLine();
}

void VPreRegAllocSched::buildMemDepEdges(VSchedGraph &CurState) {
  CurState.preSchedTopSort();
  // The schedule unit and the corresponding memory operand.
  typedef std::multimap<const Value*, VSUnit*> MemOpMapTy;
  MemOpMapTy VisitedMemOps;

  for (VSchedGraph::iterator I = CurState.begin(), E = CurState.end(); I != E;
       ++I) {
    VSUnit *DstU = *I;
    MachineInstr *DstMI = DstU->getFirstInstr();
    // Skip the non-memory operation.
    if (!DstMI || DstMI->memoperands_empty())
      continue;

    // FIXME:
    assert(DstMI->hasOneMemOperand() && "Can not handle multiple mem operand!");
    assert(!DstMI->hasVolatileMemoryRef() && "Can not handle volatile operation!");

    const Value *DstMO = (*DstMI->memoperands_begin())->getValue();
    const Type *DstElemTy = cast<SequentialType>(DstMO->getType())->getElementType();
    size_t DstSize = TD->getTypeStoreSize(DstElemTy);

    VTFInfo DstInfo = *DstMI;
    bool isDstLoad = DstInfo.mayLoad();

    if (DstMO == 0) continue;

    assert(!isa<PseudoSourceValue>(DstMO) && "Unexpected frame stuffs!");

    for (MemOpMapTy::iterator I = VisitedMemOps.begin(), E = VisitedMemOps.end();
         I != E; ++I) {
      const Value *SrcMO = I->first;
      const Type *SrcElemTy = cast<SequentialType>(SrcMO->getType())->getElementType();
      size_t SrcSize = TD->getTypeStoreSize(SrcElemTy);
      
      VSUnit *SrcU = I->second;
      MachineInstr *SrcMI = SrcU->getFirstInstr();
      VTFInfo SrcInfo = *SrcMI;
      bool isSrcLoad = SrcInfo.mayLoad();
      
      // Ignore RAR dependence.
      if (isDstLoad && isSrcLoad) continue;

      if (CurState.enablePipeLine()) {
        // Compute the iterate distance.
      } else {
        if (AA->isNoAlias(SrcMO, SrcSize, DstMO, DstSize)) continue;

      // Ignore the No-Alias pointers.
        VDMemDep *MemDep = getMemDepEdge(SrcU, SrcInfo.getLatency(), false,
                                         VDMemDep::TrueDep, 0);
        DstU->addDep(MemDep);
      }
    }

    // Add the schedule unit to visited map.
    VisitedMemOps.insert(std::make_pair(DstMO, DstU));
  }
}

void VPreRegAllocSched::clear() {
  // Reset total Cycle
  totalCycle = 1;
}

void VPreRegAllocSched::releaseMemory() {
  clear();
}

//===----------------------------------------------------------------------===//
// Create atom
void VPreRegAllocSched::addValueDeps(VSUnit *A, VSchedGraph &CurState) {
  for (VSUnit::instr_iterator I = A->instr_begin(), E = A->instr_end();
      I != E; ++I) {
    const MachineInstr *MI = *I;
    assert(MI && "Expect Schedule Unit with machine instruction!");

    for (unsigned i = 0, e = MI->getNumOperands(); i != e; ++i) {
      const MachineOperand &MO = MI->getOperand(i);
      // Only care about the register dependences.
      // FIXME: What about chains?
      if (!MO.isReg()) continue;

      // The instruction do not depend the register defined by itself.
      if (MO.isDef()) continue;

      unsigned Reg = MO.getReg();
      
      // TODO: assert Reg can not be physical register.
      // It seems that sometimes the Register will be 0?
      if (!Reg) continue;
      assert(TargetRegisterInfo::isVirtualRegister(Reg)
             && "Unexpected physics register!");


      MachineInstr *DepSrc = MRI->getVRegDef(Reg);
      /// Only add the dependence if DepSrc is in the same MBB with MI.
      if (VSUnit *Dep = CurState.lookupSUnit(DepSrc))
        A->addDep(getValDepEdge(Dep, computeLatency(DepSrc, MI)));
    }
  }

  // If the atom depend on nothing, make it depend on the entry node.
  if (A->dep_empty()) {
    VSUnit *EntryRoot = CurState.getEntryRoot();
    A->addDep(getValDepEdge(EntryRoot, 0));
  }
}

void VPreRegAllocSched::buildPipeLineDepEdges(VSchedGraph &State) {
  // Only work on pipelined loops.
  if (!State.enablePipeLine()) return;

  VSUnit *SelfEnable = State.getLoopOp();
  assert(SelfEnable && "Not in loop?");

  MachineBasicBlock *CurBB = State.getMachineBasicBlock();
  // For each phinode
  for (MachineBasicBlock::iterator I = CurBB->begin(), E = CurBB->getFirstNonPHI();
       I != E; ++I) {
    MachineInstr &PN = *I;
    assert(PN.isPHI() && "IsSingleValuePHICycle expects a PHI instruction");
    unsigned DstReg = PN.getOperand(0).getReg();

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

      // Transfer the dependence to the Users of this PHI node.
      for (MachineRegisterInfo::use_iterator UI = MRI->use_begin(DstReg),
           UE = MRI->use_end(); UI != UE; ++UI) {
        MachineInstr &UseMI = *UI;
        VSUnit *UseSU = State.lookupSUnit(&UseMI);
        assert(UseSU && "Where's the use of the phi?");
        UseSU->addDep(getMemDepEdge(InSU, computeLatency(SrcMI, &UseMI),
                                    true, VDMemDep::AntiDep, 1));
      }
    }
  }
}

void VPreRegAllocSched::buildSUnit(MachineInstr *MI,  VSchedGraph &CurState) {
  VTFInfo VTID = *MI;
  // If the current instruction was eaten by as terminator?
  if (CurState.eatTerminator(VTID)) return;

  FuncUnitId Id = VTID.getPrebindFUId();

  // TODO: Remember the register that live out this MBB.
  // and the instruction that only produce a chain.
  VSUnit *SU = CurState.createVSUnit(&MI, 1, Id.getFUNum());
  addValueDeps(SU, CurState);
}

void VPreRegAllocSched::buildExitRoot(VSchedGraph &CurState) {
  SmallVectorImpl<MachineInstr*> &Terms = CurState.getTerms();

  VSUnit *Exit = CurState.createVSUnit(Terms.data(), Terms.size());
  addValueDeps(Exit, CurState);

  MachineInstr *FstExit = Terms.front();

  for (VSchedGraph::iterator I = CurState.begin(), E = CurState.end();
      I != E; ++I) {
    VSUnit *VSU = *I;
    if (VSU->getNumUses() == 0 && !VSU->isEntry()
      // Since the exit root already added to state sunit list, skip the
      // exit itself.
      && VSU != Exit) {
      // Dirty Hack.
      MachineInstr *Instr = *VSU->instr_begin();
      Exit->addDep(getCtrlDepEdge(VSU, computeLatency(Instr, FstExit)));
    }
  }

  // Do not forget the entry root.
  Exit->addDep(getCtrlDepEdge(CurState.getEntryRoot(),
                              computeLatency(0, FstExit)));
}

void VPreRegAllocSched::buildState(VSchedGraph &State) {
  for (MachineBasicBlock::iterator BI = State->begin(), BE = State->end();
      BI != BE; ++BI)
    buildSUnit(&*BI, State);

  // We need at an explicit terminator to transfer the control flow explicitly.
  if (State.getTerms().empty()) {
    MachineBasicBlock *MBB = State.getMachineBasicBlock();
    if (MBB->succ_size() == 0) { // We may meet an unreachable.
      MachineInstr &Term = *BuildMI(MBB, DebugLoc(),
                                    VTarget.getInstrInfo()->get(VTM::VOpRet));
      State.eatTerminator(VTFInfo(Term));
    } else {
      assert(MBB->succ_size() == 1 && "Expect fall through block!");
      // Create "VOpToState 1/*means always true*/, target mbb"
      MachineInstr &Term = *BuildMI(MBB, DebugLoc(), 
                                    VTarget.getInstrInfo()->get(VTM::VOpToState))
        .addImm(1, 1).addMBB(*MBB->succ_begin());
      State.eatTerminator(VTFInfo(Term));
    }
  }

  // Create the exit node.
  buildExitRoot(State);

  // Build loop edges if necessary.
  buildPipeLineDepEdges(State);

  // Build the memory edges.
  buildMemDepEdges(State);
}

VDMemDep *VPreRegAllocSched::getMemDepEdge(VSUnit *Src, unsigned Latency,
                                    bool isBackEdge,
                                    enum VDMemDep::MemDepTypes DepType,
                                    unsigned Diff) {
  return new VDMemDep(Src, Latency, isBackEdge, DepType, Diff);
}

void VPreRegAllocSched::print(raw_ostream &O, const Module *M) const {}

VPreRegAllocSched::VPreRegAllocSched(const VTargetMachine &TM)
  : MachineFunctionPass(ID), LI(0), LiveVars(0), VTarget(TM),
  MRI(0), totalCycle(1) {}

VPreRegAllocSched::~VPreRegAllocSched() {
  clear();
}
