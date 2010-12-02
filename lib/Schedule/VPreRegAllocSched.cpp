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


#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/MachineLoopInfo.h"
#include "llvm/CodeGen/LiveVariables.h"
#include "llvm/Support/Allocator.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/GraphWriter.h"
#include "llvm/Support/MathExtras.h"

#define DEBUG_TYPE "vtm-sgraph"
#include "llvm/Support/Debug.h"

using namespace llvm;

//===----------------------------------------------------------------------===//

namespace {
  /// @brief Hardware atom construction pass
///
struct VPreRegAllocSched : public MachineFunctionPass {
  // The loop Info
  MachineLoopInfo *LI;
  LiveVariables *LiveVars;
  const VTargetMachine &VTarget;
  MachineRegisterInfo *MRI;
  VFuncInfo *FuncInfo;
  BitLevelInfo *BLI;
  SmallVector<MachineInstr*, 4> Terminators;

  // Total states
  // Cycle is start from 1 because  cycle 0 is reserve for idle state.
  unsigned short totalCycle;
  unsigned short InstIdx;

  void buildState(VSchedGraph &State);

  unsigned computeLatency(const MachineInstr *SrcInstr,
                          const MachineInstr *DstInstr) {
    if (SrcInstr == 0) {
      // DirtyHack: There must be at least 1 slot between entry and exit.
      if (DstInstr && VTFInfo(*DstInstr)->isTerminator()) return 1;

      return 0;
    }

    VTFInfo SrcTID = SrcInstr->getDesc();
    unsigned latency = SrcTID.getLatency(VTarget);

    if (DstInstr == 0) return latency;

    VTFInfo DstTID = DstInstr->getDesc();

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

  void addMemDepEdges(std::vector<VSUnit*> &MemOps, BasicBlock &BB);

  bool haveSelfLoop(const MachineBasicBlock *MBB);


  /// @name FunctionPass interface
  //{
  static char ID;
  VPreRegAllocSched(const VTargetMachine &TM);

  ~VPreRegAllocSched();

  void buildExitRoot(VSchedGraph &CurState);
  void buildSUnit(MachineInstr *MI, VSchedGraph &CurState);

  bool runOnMachineFunction(MachineFunction &MF);
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
  // AU.addRequired<MemDepInfo>();
  AU.setPreservesCFG();
  AU.addPreserved<BitLevelInfo>();
}

bool VPreRegAllocSched::runOnMachineFunction(MachineFunction &MF) {
  LiveVars = &getAnalysis<LiveVariables>();
  MRI = &MF.getRegInfo();
  FuncInfo = MF.getInfo<VFuncInfo>();
  BLI = &getAnalysis<BitLevelInfo>();

  for (MachineFunction::iterator I = MF.begin(), E = MF.end();
       I != E; ++I) {
    MachineBasicBlock *MBB = &*I;
    
    VSchedGraph State(VTarget, MBB, false, getTotalCycle());

    buildState(State);
    DEBUG(State.viewGraph());
    State.schedule();
    setTotalCycle(State.getEndSlot() + 1);
    DEBUG(State.viewGraph());

    State.emitSchedule(*BLI);
    FuncInfo->remeberTotalSlot(MBB, State.getStartSlot(),
                                    State.getTotalSlot(),
                                    State.getII());
  }


  //LI = &getAnalysis<LoopInfo>();
  //RC = &getAnalysis<ResourceConfig>();
  //// MDA = &getAnalysis<MemDepInfo>();
  //TD = getAnalysisIfAvailable<TargetData>();
  //assert(TD && "Can not work without TD!");

  //std::vector<VSUnit*> MemOps;

  //for (Function::iterator I = F.begin(), E = F.end(); I != E; ++I) {
  //  // Setup the state.
  //  BasicBlock *BB = &*I;
  //  VSchedGraph *State = getState(BB);
  //  ValueToSUnits[BB] = State;
  //  SetControlRoot(State);
  //  DEBUG(dbgs() << "Building atom for BB: " << BB->getName() << '\n');
  //  for (BasicBlock::iterator BI = BB->begin(), BE = BB->end();
  //      BI != BE; ++BI) {
  //    Instruction &Inst = *BI;
  //    VSUnit *A = visit(Inst);
  //    
  //    if (!A) continue;
  //    // Add the SUnit to state.
  //    // FIXME: Some atom will add them to parent atoms vector automatically,
  //    // but this is not a good idea.
  //    if (!A->getParent())
  //      State->addSUnit(A);

  //    // Remember the atom.
  //    ValueToSUnits.insert(std::make_pair(&Inst, A));
  //    // Remember the MemOps, we will compute the dependencies about them later.
  //    if (isa<LoadInst>(Inst) || isa<StoreInst>(Inst))
  //      MemOps.push_back(cast<VSUnit>(A));
  //  }
  //  // preform memory dependencies analysis to add corresponding edges.
  //  addMemDepEdges(MemOps, *BB);

  //  bool selfLoop = haveSelfLoop(BB);
  //  

  //  VSUnit *Exit = cast<VSUnit>(getControlRoot());
  //  State->setExitRoot(Exit);
  //  State->setHaveSelfLoop(selfLoop);

  //  MemOps.clear();
  //}

  //DEBUG(print(dbgs(), 0));

  return false;
}


bool VPreRegAllocSched::haveSelfLoop(const MachineBasicBlock *MBB) {
  //Loop *L = LI->getLoopFor(BB);

  //// Not in any loop.
  //if (!L) return false;
  //// Dirty Hack: Only support one block loop at this moment.
  //if (L->getBlocks().size() != 1) return false;

  return true;
}



//HWADelay *VPreRegAllocSched::addLoopPredBackEdge(const MachineBasicBlock *MBB) {
//  assert(haveSelfLoop(MBB) && "Loop SCC only exist in self loop!");
//  
//  VSchedGraph *State = getStateFor(MBB);
//  // And get the predicate
//  //BranchInst *Br = cast<BranchInst>(MBB->getTerminator());
//  //ICmpInst *ICmp = cast<ICmpInst>(Br->getCondition());
//  //VSUnit *Pred = cast<VSUnit>(getSUnitFor(*ICmp));
//
//  //// The Next loop depend on the result of predicate.
//  //// Dirty Hack: The FSM have a delay of 1.
//  //HWADelay *Delay = getDelay(Pred, 1);
//  //VDMemDep *LoopDep = getMemDepEdge(Delay, true, VDMemDep::TrueDep, 1);
//  //State->addDep(LoopDep);
//
//  // return Delay;
//  return 0;
//}

void VPreRegAllocSched::addMemDepEdges(std::vector<VSUnit*> &MemOps, BasicBlock &BB) {
  //typedef std::vector<VSUnit*> OpInstVec;
  //typedef MemDepInfo::DepInfo DepInfoType;
  //for (OpInstVec::iterator SrcI = MemOps.begin(), SrcE = MemOps.end();
  //    SrcI != SrcE; ++SrcI) {
  //  VSUnit *Src = *SrcI;
  //  bool isSrcLoad = isa<LoadInst>(Src->getValue());

  //  for (OpInstVec::iterator DstI = MemOps.begin(), DstE = MemOps.end();
  //      DstI != DstE; ++DstI) {
  //    VSUnit *Dst = *DstI;
  //    bool isDstLoad = isa<LoadInst>(Dst->getValue());

  //    //No self loops and RAR dependence.
  //    if (Src == Dst || (isSrcLoad && isDstLoad))
  //      continue;
  //    
  //    bool srcBeforeDest = Src->getIdx() < Dst->getIdx();

  //    // Get dependencies information.
  //    DepInfoType Dep = MDA->getDepInfo(Src->getInst<Instruction>(),
  //                                       Dst->getInst<Instruction>(),
  //                                       BB, srcBeforeDest);
  //    if (!Dep.hasDep())
  //      continue;

  //    // Add dependencies edges.
  //    // The edge is back edge if destination before source.
  //    VDMemDep *MemDep = getMemDepEdge(Src, Src->getIdx() > Dst->getIdx(), 
  //                                     Dep.getDepType(), Dep.getItDst());
  //    Dst->addDep(MemDep);
  //  }
  //}
}

void VPreRegAllocSched::clear() {
  // Reset total Cycle
  totalCycle = 1;
  InstIdx = 0;
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

void VPreRegAllocSched::buildSUnit(MachineInstr *MI,  VSchedGraph &CurState) {
  VTFInfo VTID = MI->getDesc();

  VFUs::FUTypes FUTy = VTID.getFUType();
  
  if (VTID->isTerminator()) {
    // Build the schedule unit for terminators later. 
    Terminators.push_back(MI);
    return;
  }

  unsigned Latency = VTID.getLatency(VTarget);

  FuncUnitId Id = VTID.getPrebindFUId();

  // TODO: Remember the register that live out this MBB.
  // and the instruction that only produce a chain.
  VSUnit *A = new VSUnit(&MI, 1, Latency, ++InstIdx, Id.getFUNum());

  addValueDeps(A, CurState);

  // Add the atom to the state.
  CurState.addSUnit(A);
}

void VPreRegAllocSched::buildExitRoot(VSchedGraph &CurState) {
  VSUnit *Exit = new VSUnit(Terminators.data(), Terminators.size(), 0,
                            ++InstIdx);
  addValueDeps(Exit, CurState);

  MachineInstr *FstExit = Terminators.front();

  for (VSchedGraph::iterator I = CurState.begin(), E = CurState.end();
      I != E; ++I) {
    VSUnit *VSU = *I;
    if (VSU->getNumUses() == 0 && !VSU->isEntry()) {
      // Dirty Hack.
      MachineInstr *Instr = *VSU->instr_begin();
      Exit->addDep(getCtrlDepEdge(VSU, computeLatency(Instr, FstExit)));
    }
  }

  // Do not forget the entry root.
  Exit->addDep(getCtrlDepEdge(CurState.getEntryRoot(),
                              computeLatency(0, FstExit)));

  // We may have multiple terminator.
  Terminators.clear();

  // Add the atom to the state.
  CurState.addSUnit(Exit);
}

void VPreRegAllocSched::buildState(VSchedGraph &State) {
  // Create a dummy entry node.
  State.addSUnit(new VSUnit(++InstIdx));

  for (MachineBasicBlock::iterator BI = State->begin(), BE = State->end();
      BI != BE; ++BI)
    buildSUnit(&*BI, State);

  // We need at an explicit terminator to transfer the control flow explicitly.
  if (Terminators.empty()) {
    MachineBasicBlock *MBB = State.getMachineBasicBlock();
    assert(MBB->succ_size() == 1 && "Expect fall through block!");
    // Create "VOpToState 1/*means always true*/, target mbb"
    MachineInstr &Term = *BuildMI(MBB, DebugLoc(), 
                                  VTarget.getInstrInfo()->get(VTM::VOpToState))
      .addImm(1, 1).addMBB(*MBB->succ_begin());
    Terminators.push_back(&Term);
  }

  // Create the exit node.
  buildExitRoot(State);
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
  MRI(0), totalCycle(1), InstIdx(0)
{}

VPreRegAllocSched::~VPreRegAllocSched() {
  clear();
}
