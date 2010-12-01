//===------------ HWAtom.cpp - Translate LLVM IR to HWAtom  -----*- C++ -*-===//
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
// This file implement the HWAtomInfo pass, which construct the HWAtom
// from LLVM IR.
//
//===----------------------------------------------------------------------===//

#include "HWAtom.h"
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
struct HWAtomInfo : public MachineFunctionPass {
  // The loop Info
  MachineLoopInfo *LI;
  LiveVariables *LiveVars;
  const VTargetMachine &VTarget;
  MachineRegisterInfo *MRI;
  VFuncInfo *FuncInfo;
  BitLevelInfo *BLI;
  // Nodes that detach from the exit node.
  SmallVector<MachineInstr*, 16> DetachNodes;
  SmallVector<MachineInstr*, 4> Terminators;

  // Total states
  // Cycle is start from 1 because  cycle 0 is reserve for idle state.
  unsigned short totalCycle;
  unsigned short InstIdx;

  void buildState(FSMState &State);

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

  HWValDep *getValDepEdge(HWAtom *Src, unsigned Latency, bool isSigned = false,
                          enum HWValDep::ValDepTypes T = HWValDep::Normal) {
    return new HWValDep(Src, Latency, isSigned, T);
  }

  HWCtrlDep *getCtrlDepEdge(HWAtom *Src, unsigned Latency) {
    return new HWCtrlDep(Src, Latency);
  }

  HWMemDep *getMemDepEdge(HWAtom *Src, unsigned Latency, bool isBackEdge,
                          enum HWMemDep::MemDepTypes DepType,
                          unsigned Diff);

  void addValueDeps(HWAtom *A, FSMState &CurState,
                    SmallVectorImpl<const MachineOperand*> &Defs);

  void clear();

  void addMemDepEdges(std::vector<HWAtom*> &MemOps, BasicBlock &BB);

  bool haveSelfLoop(const MachineBasicBlock *MBB);


  /// @name FunctionPass interface
  //{
  static char ID;
  HWAtomInfo(const VTargetMachine &TM);

  ~HWAtomInfo();

  void buildExitRoot(FSMState &CurState);
  void buildAtom(MachineInstr *MI, FSMState &CurState);

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

char HWAtomInfo::ID = 0;

Pass *llvm::createHWAtonInfoPass(const VTargetMachine &TM) {
  return new HWAtomInfo(TM);
}

void HWAtomInfo::getAnalysisUsage(AnalysisUsage &AU) const {
  MachineFunctionPass::getAnalysisUsage(AU);
  AU.addRequired<LiveVariables>();
  AU.addRequired<MachineLoopInfo>();
  AU.addRequired<BitLevelInfo>();
  // AU.addRequired<MemDepInfo>();
  AU.setPreservesCFG();
  AU.addPreserved<BitLevelInfo>();
}

bool HWAtomInfo::runOnMachineFunction(MachineFunction &MF) {
  LiveVars = &getAnalysis<LiveVariables>();
  MRI = &MF.getRegInfo();
  FuncInfo = MF.getInfo<VFuncInfo>();
  BLI = &getAnalysis<BitLevelInfo>();

  for (MachineFunction::iterator I = MF.begin(), E = MF.end();
       I != E; ++I) {
    MachineBasicBlock *MBB = &*I;
    
    FSMState State(VTarget, MBB, false, getTotalCycle());

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

  //std::vector<HWAtom*> MemOps;

  //for (Function::iterator I = F.begin(), E = F.end(); I != E; ++I) {
  //  // Setup the state.
  //  BasicBlock *BB = &*I;
  //  FSMState *State = getState(BB);
  //  ValueToHWAtoms[BB] = State;
  //  SetControlRoot(State);
  //  DEBUG(dbgs() << "Building atom for BB: " << BB->getName() << '\n');
  //  for (BasicBlock::iterator BI = BB->begin(), BE = BB->end();
  //      BI != BE; ++BI) {
  //    Instruction &Inst = *BI;
  //    HWAtom *A = visit(Inst);
  //    
  //    if (!A) continue;
  //    // Add the Atom to state.
  //    // FIXME: Some atom will add them to parent atoms vector automatically,
  //    // but this is not a good idea.
  //    if (!A->getParent())
  //      State->addAtom(A);

  //    // Remember the atom.
  //    ValueToHWAtoms.insert(std::make_pair(&Inst, A));
  //    // Remember the MemOps, we will compute the dependencies about them later.
  //    if (isa<LoadInst>(Inst) || isa<StoreInst>(Inst))
  //      MemOps.push_back(cast<HWAtom>(A));
  //  }
  //  // preform memory dependencies analysis to add corresponding edges.
  //  addMemDepEdges(MemOps, *BB);

  //  bool selfLoop = haveSelfLoop(BB);
  //  

  //  HWAtom *Exit = cast<HWAtom>(getControlRoot());
  //  State->setExitRoot(Exit);
  //  State->setHaveSelfLoop(selfLoop);

  //  MemOps.clear();
  //}

  //DEBUG(print(dbgs(), 0));

  return false;
}


bool HWAtomInfo::haveSelfLoop(const MachineBasicBlock *MBB) {
  //Loop *L = LI->getLoopFor(BB);

  //// Not in any loop.
  //if (!L) return false;
  //// Dirty Hack: Only support one block loop at this moment.
  //if (L->getBlocks().size() != 1) return false;

  return true;
}



//HWADelay *HWAtomInfo::addLoopPredBackEdge(const MachineBasicBlock *MBB) {
//  assert(haveSelfLoop(MBB) && "Loop SCC only exist in self loop!");
//  
//  FSMState *State = getStateFor(MBB);
//  // And get the predicate
//  //BranchInst *Br = cast<BranchInst>(MBB->getTerminator());
//  //ICmpInst *ICmp = cast<ICmpInst>(Br->getCondition());
//  //HWAtom *Pred = cast<HWAtom>(getAtomFor(*ICmp));
//
//  //// The Next loop depend on the result of predicate.
//  //// Dirty Hack: The FSM have a delay of 1.
//  //HWADelay *Delay = getDelay(Pred, 1);
//  //HWMemDep *LoopDep = getMemDepEdge(Delay, true, HWMemDep::TrueDep, 1);
//  //State->addDep(LoopDep);
//
//  // return Delay;
//  return 0;
//}

void HWAtomInfo::addMemDepEdges(std::vector<HWAtom*> &MemOps, BasicBlock &BB) {
  //typedef std::vector<HWAtom*> OpInstVec;
  //typedef MemDepInfo::DepInfo DepInfoType;
  //for (OpInstVec::iterator SrcI = MemOps.begin(), SrcE = MemOps.end();
  //    SrcI != SrcE; ++SrcI) {
  //  HWAtom *Src = *SrcI;
  //  bool isSrcLoad = isa<LoadInst>(Src->getValue());

  //  for (OpInstVec::iterator DstI = MemOps.begin(), DstE = MemOps.end();
  //      DstI != DstE; ++DstI) {
  //    HWAtom *Dst = *DstI;
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
  //    HWMemDep *MemDep = getMemDepEdge(Src, Src->getIdx() > Dst->getIdx(), 
  //                                     Dep.getDepType(), Dep.getItDst());
  //    Dst->addDep(MemDep);
  //  }
  //}
}

void HWAtomInfo::clear() {
  // Reset total Cycle
  totalCycle = 1;
  InstIdx = 0;
}

void HWAtomInfo::releaseMemory() {
  clear();
}

//===----------------------------------------------------------------------===//
// Create atom
void HWAtomInfo::addValueDeps(HWAtom *A, FSMState &CurState,
                              SmallVectorImpl<const MachineOperand*> &Defs) {
  for (HWAtom::instr_iterator I = A->instr_begin(), E = A->instr_end();
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

      if (MO.isDef()) {
        Defs.push_back(&MO);
        continue;
      }

      MachineInstr *DepSrc = MRI->getVRegDef(Reg);
      /// Only add the dependence if DepSrc is in the same MBB with MI.
      if (HWAtom *Dep = CurState.lookupAtom(DepSrc))
        A->addDep(getValDepEdge(Dep, computeLatency(DepSrc, MI)));
    }
  }

  // If the atom depend on nothing, make it depend on the entry node.
  if (A->dep_empty()) {
    HWAtom *EntryRoot = CurState.getEntryRoot();
    A->addDep(getValDepEdge(EntryRoot, 0));
  }
}

void HWAtomInfo::buildAtom(MachineInstr *MI,  FSMState &CurState) {
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
  HWAtom *A = new HWAtom(&MI, 1, Latency, ++InstIdx, Id.getFUNum());
  SmallVector<const MachineOperand*, 4> Defs;

  addValueDeps(A, CurState, Defs);
  
  // Assume all def is dead, and try to prove it wrong.
  bool AllDefDead = true;

  if (!Defs.empty()) {
    const MachineBasicBlock *MBB = MI->getParent();
    for (SmallVector<const MachineOperand*, 4>::iterator I = Defs.begin(),
         E = Defs.end(); I != E; ++I) {
      const MachineOperand *Op = *I;
      if (!Op->isDead()) AllDefDead = false;

      if (LiveVars->isLiveOut(Op->getReg(), *MBB)) {
        // If the node defines any live out register, it may be a detach node.
        DetachNodes.push_back(MI);
        break;
      }
    }
  }

  // If All define dead, this node is detached.
  if (AllDefDead) DetachNodes.push_back(MI);

  // Add the atom to the state.
  CurState.addAtom(A);
}

void HWAtomInfo::buildExitRoot(FSMState &CurState) {
  HWAtom *Exit = new HWAtom(Terminators.data(), Terminators.size(), 0,
                            ++InstIdx);
  
  SmallVector<const MachineOperand*, 2> Defs;
  addValueDeps(Exit, CurState, Defs);

  MachineInstr *FstExit = Terminators.front();

  // All operation must finished before leaving the state.
  while (!DetachNodes.empty()) {
    MachineInstr *I = DetachNodes.pop_back_val();
    HWAtom *Dep = CurState.lookupAtom(I);
    assert(Dep && "Can not find dep!");
    Exit->addDep(getCtrlDepEdge(Dep, computeLatency(I, FstExit)));
  }

  // Do not forget the entry root.
  Exit->addDep(getCtrlDepEdge(CurState.getEntryRoot(),
                              computeLatency(0, FstExit)));

  // We may have multiple terminator.
  Terminators.clear();

  // Add the atom to the state.
  CurState.addAtom(Exit);
}

void HWAtomInfo::buildState(FSMState &State) {
  // Create a dummy entry node.
  State.addAtom(new HWAtom(++InstIdx));

  for (MachineBasicBlock::iterator BI = State->begin(), BE = State->end();
      BI != BE; ++BI)
    buildAtom(&*BI, State);

  // Create the exit node.
  buildExitRoot(State);
}

HWMemDep *HWAtomInfo::getMemDepEdge(HWAtom *Src, unsigned Latency,
                                    bool isBackEdge,
                                    enum HWMemDep::MemDepTypes DepType,
                                    unsigned Diff) {
  return new HWMemDep(Src, Latency, isBackEdge, DepType, Diff);
}

void HWAtomInfo::print(raw_ostream &O, const Module *M) const {}

HWAtomInfo::HWAtomInfo(const VTargetMachine &TM)
  : MachineFunctionPass(ID), LI(0), LiveVars(0), VTarget(TM),
  MRI(0), totalCycle(1), InstIdx(0)
{}

HWAtomInfo::~HWAtomInfo() {
  clear();
}
