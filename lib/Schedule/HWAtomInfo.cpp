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

#define DEBUG_TYPE "vbe-hw-atom-info"
#include "llvm/Support/Debug.h"

using namespace llvm;

//===----------------------------------------------------------------------===//

namespace {
  /// @brief Hardware atom construction pass
///
struct HWAtomInfo : public MachineFunctionPass {
  // Allocator
  BumpPtrAllocator HWAtomAllocator;
  // The loop Info
  MachineLoopInfo *LI;
  LiveVariables *LiveVars;
  const VTargetMachine &VTarget;
  MachineRegisterInfo *MRI;
  VFuncInfo *FuncInfo;
  BitLevelInfo *BLI;
  // Nodes that detach from the exit node.
  std::vector<HWAtom*> DetachNodes;

  // Total states
  // Cycle is start from 1 because  cycle 0 is reserve for idle state.
  unsigned short totalCycle;
  unsigned short InstIdx;

  HWAtom *getExitRoot(MachineInstr *MI);

  FSMState *buildState(MachineBasicBlock *MBB);

  typedef DenseMap<const MachineInstr*, HWAtom*> AtomMapType;
  AtomMapType InstToHWAtoms;

  typedef DenseMap<const MachineBasicBlock*, FSMState*> StateMapType;
  StateMapType MachBBToStates;

  unsigned computeLatency(const HWAtom *Src, const MachineInstr *DstInstr) {
    MachineInstr *SrcInstr = Src->getInstr();

    if (SrcInstr == 0) return 0;

    VTIDReader SrcTID(SrcInstr);
    unsigned latency = SrcTID.getLatency(VTarget);
    
    VTIDReader DstTID(DstInstr);

    if (DstInstr == 0) return latency;

    // We need to wait one more slot to read the result.
    if (SrcTID.isWriteUntilFinish() && DstTID.isReadAtEmit())
      return latency + 1;
    
    return latency;
  }

  HWValDep *getValDepEdge(HWAtom *Src, unsigned Latency, bool isSigned = false,
                          enum HWValDep::ValDepTypes T = HWValDep::Normal) {
    return new (HWAtomAllocator) HWValDep(Src, Latency, isSigned, T);
  }

  HWCtrlDep *getCtrlDepEdge(HWAtom *Src, unsigned Latency) {
    return new (HWAtomAllocator) HWCtrlDep(Src, Latency);
  }

  HWMemDep *getMemDepEdge(HWAtom *Src, unsigned Latency, bool isBackEdge,
                          enum HWMemDep::MemDepTypes DepType,
                          unsigned Diff);

  void analyzeOperands(const MachineInstr *MI, SmallVectorImpl<HWEdge*> &Deps,
                       SmallVectorImpl<const MachineOperand*> &Defs);

  void clear();

  void addMemDepEdges(std::vector<HWAtom*> &MemOps, BasicBlock &BB);

  bool haveSelfLoop(const MachineBasicBlock *MBB);


  /// @name FunctionPass interface
  //{
  static char ID;
  HWAtomInfo(const VTargetMachine &TM);

  ~HWAtomInfo();

  HWAtom *buildExitRoot(MachineInstr *MI);
  HWAtom *buildAtom(MachineInstr *MI);

  bool runOnMachineFunction(MachineFunction &MF);
  void releaseMemory();
  void getAnalysisUsage(AnalysisUsage &AU) const;
  void print(raw_ostream &O, const Module *M) const;
  //}

  void scheduleState(FSMState *State);

  HWAtom *getAtomFor(const MachineInstr *MI) const {
    AtomMapType::const_iterator At = InstToHWAtoms.find(MI);

    if(At != InstToHWAtoms.end())
      return  At->second;

    return 0;
  }

  FSMState *getStateFor(const MachineBasicBlock *MBB) const {
    StateMapType::const_iterator At = MachBBToStates.find(MBB);
    assert(At != MachBBToStates.end() && "State can not be found!");
    return At->second;    
  }

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
  AU.setPreservesAll();
}

bool HWAtomInfo::runOnMachineFunction(MachineFunction &MF) {
  LiveVars = &getAnalysis<LiveVariables>();
  MRI = &MF.getRegInfo();
  FuncInfo = MF.getInfo<VFuncInfo>();
  BLI = &getAnalysis<BitLevelInfo>();

  for (MachineFunction::iterator I = MF.begin(), E = MF.end();
       I != E; ++I) {
    MachineBasicBlock *MBB = &*I;
    FSMState *State = buildState(MBB);

    State->viewGraph();
    scheduleState(State);
    State->viewGraph();

    State->emitSchedule(*BLI);
    FuncInfo->remeberTotalSlot(MBB, State->getStartSlot(),
                                    State->getTotalSlot(),
                                    State->getII());
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
  HWAtomAllocator.Reset();
  MachBBToStates.clear();
  InstToHWAtoms.clear();
  // Reset total Cycle
  totalCycle = 1;
  InstIdx = 0;
}

void HWAtomInfo::releaseMemory() {
  clear();
}

//===----------------------------------------------------------------------===//
// Create atom
void HWAtomInfo::analyzeOperands(const MachineInstr *MI,
                                 SmallVectorImpl<HWEdge*> &Deps,
                                 SmallVectorImpl<const MachineOperand*> &Defs) {
  const MachineBasicBlock *MBB = MI->getParent();

  for (unsigned i = 0, e = MI->getNumOperands(); i != e; ++i) {
    const MachineOperand &MO = MI->getOperand(i);
    // Only care about the register dependences.
    // FIXME: What about chains?
    if (!MO.isReg()) continue;

    unsigned Reg = MO.getReg();
    
    // TODO: assert Reg can not be physical register.
    // It seems that sometimes the Register will be 0?
    if (!Reg) continue;

    if (MO.isDef()) {
      Defs.push_back(&MO);
      continue;
    }
    
    // We are building a dependence graph of a MBB only.
    if (MBB->isLiveIn(Reg)) continue;

    if (HWAtom *Dep = getAtomFor(MRI->getVRegDef(Reg)))
      Deps.push_back(getValDepEdge(Dep, computeLatency(Dep, MI)));
  }

  // If the atom depend on nothing, make it depend on the entry node.
  if (Deps.empty()) {
    HWAtom *EntryRoot = getStateFor(MBB)->getEntryRoot();
    Deps.push_back(getValDepEdge(EntryRoot, 0));
  }
}

HWAtom *HWAtomInfo::buildAtom(MachineInstr *MI) {
  assert(!InstToHWAtoms.count(MI) && "MI exist!");

  VTIDReader VTID(MI);

  VFUs::FUTypes FUTy = VTID.getFUType();
  
  if (VTID->isTerminator())
    return buildExitRoot(MI);    

  SmallVector<HWEdge*, 4> Deps;
  SmallVector<const MachineOperand*, 4> Defs;
  analyzeOperands(MI, Deps, Defs);

  unsigned Latency = VTID.getLatency(VTarget);

  FuncUnitId Id = VTID.getPrebindFUId();

  // TODO: Remember the register that live out this MBB.
  // and the instruction that only produce a chain.
  HWAtom *A = new (HWAtomAllocator) HWAtom(MI, Deps.begin(), Deps.end(),
                                           Latency, ++InstIdx, Id.getFUNum());
  
  // Assume all def is dead, and try to prove it wrong.
  bool AllDefDead = true;

  if (Defs.empty()) {
    DetachNodes.push_back(A);
    return A;
  } else {
    const MachineBasicBlock *MBB = MI->getParent();
    for (SmallVector<const MachineOperand*, 4>::iterator I = Defs.begin(),
         E = Defs.end(); I != E; ++I) {
      const MachineOperand *Op = *I;
      if (!Op->isDead()) AllDefDead = false;

      if (LiveVars->isLiveOut(Op->getReg(), *MBB)) {
        // If the node defines any live out register, it may be a detach node.
        DetachNodes.push_back(A);
        break;
      }
    }
  }

  // If All define dead, this node is detached.
  if (AllDefDead) DetachNodes.push_back(A);

  return A;
}

HWAtom *HWAtomInfo::buildExitRoot(MachineInstr *MI) {
  SmallVector<HWEdge*, 16> Deps;

  for (std::vector<HWAtom*>::iterator I = DetachNodes.begin(),
      E = DetachNodes.end(); I != E; ++I)
    Deps.push_back(getCtrlDepEdge(*I, computeLatency(*I, MI)));

  SmallVector<const MachineOperand*, 1> Defs;
  analyzeOperands(MI, Deps, Defs);

  HWAtom *A = new (HWAtomAllocator) HWAtom(MI, Deps.begin(), Deps.end(),
                                           0, ++InstIdx);
  // We may have multiple terminator.
  DetachNodes.clear();
  DetachNodes.push_back(A);
  return A;
}

FSMState *HWAtomInfo::buildState(MachineBasicBlock *MBB) {
  // FIXME: check if MBB have self loop.
  FSMState *State =  new (HWAtomAllocator) FSMState(VTarget, MBB, false,
                                                    getTotalCycle(),
                                                    ++InstIdx);

  MachBBToStates.insert(std::make_pair(MBB, State));
  DetachNodes.clear();

  // Create a dummy entry node.
  State->addAtom(new (HWAtomAllocator) HWAtom(0, 0, ++InstIdx));
  
  for (MachineBasicBlock::iterator BI = MBB->begin(), BE = MBB->end();
      BI != BE; ++BI) {
    MachineInstr &MInst = *BI;
    if (HWAtom *A = buildAtom(&MInst)) {
      State->addAtom(A);
      InstToHWAtoms.insert(std::make_pair(&MInst, A));
    }
  }

  return State;
}

HWMemDep *HWAtomInfo::getMemDepEdge(HWAtom *Src, unsigned Latency,
                                    bool isBackEdge,
                                    enum HWMemDep::MemDepTypes DepType,
                                    unsigned Diff) {
  return new (HWAtomAllocator) HWMemDep(Src, Latency, isBackEdge, DepType, Diff);
}

void HWAtomInfo::print(raw_ostream &O, const Module *M) const {}


void HWAtomInfo::scheduleState(FSMState *State) {
  State->scheduleState();
  setTotalCycle(State->getEndSlot() + 1);
}

HWAtomInfo::HWAtomInfo(const VTargetMachine &TM)
  : MachineFunctionPass(ID), LI(0), LiveVars(0), VTarget(TM),
  MRI(0), totalCycle(1), InstIdx(0)
{}

HWAtomInfo::~HWAtomInfo() {
  clear();
}
