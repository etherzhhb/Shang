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

#include "HWAtomInfo.h"
#include "MemDepAnalysis.h"

#include "llvm/Analysis/LoopInfo.h"

#define DEBUG_TYPE "vbe-hw-atom-info"
#include "llvm/Support/Debug.h"

using namespace llvm;
using namespace esyn;

//===----------------------------------------------------------------------===//

char HWAtomInfo::ID = 0;
RegisterPass<HWAtomInfo> X("vbe-hw-atom-info",
                           "vbe - Construct the Hardware atom respresent"
                           " on llvm IR");

void HWAtomInfo::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<LoopInfo>();
  AU.addRequired<ResourceConfig>();
  AU.addRequired<MemDepInfo>();
  AU.setPreservesAll();
}

bool HWAtomInfo::runOnFunction(Function &F) {
  LI = &getAnalysis<LoopInfo>();
  RC = &getAnalysis<ResourceConfig>();
  MDA = &getAnalysis<MemDepInfo>();

  std::vector<HWAOpInst*> MemOps;

  for (Function::iterator I = F.begin(), E = F.end(); I != E; ++I) {
    // Setup the state.
    BasicBlock *BB = &*I;
    FSMState *State = getState(BB);
    ValueToHWAtoms[BB] = State;
    SetControlRoot(State);
    DEBUG(dbgs() << "Building atom for BB: " << BB->getName() << '\n');
    for (BasicBlock::iterator BI = BB->begin(), BE = BB->end();
        BI != BE; ++BI) {
      Instruction &Inst = *BI;
      HWAtom *A = visit(Inst);
      
      if (!A) continue;

      // Remember the atom.
      ValueToHWAtoms.insert(std::make_pair(&Inst, A));
      // Remember the MemOps, we will compute the dependencies about them later.
      if (isa<LoadInst>(Inst) || isa<StoreInst>(Inst))
        MemOps.push_back(cast<HWAOpInst>(A));
    }
    // preform memory dependencies analysis to add corresponding edges.
    addMemDepEdges(MemOps, *BB);

    bool selfLoop = haveSelfLoop(BB);
    
    // Add SCC for loop.
    if (selfLoop)
      addLoopPredBackEdge(BB);

    HWAOpInst *Exit = cast<HWAPostBind>(getControlRoot());
    State->setExitRoot(Exit);
    State->setHaveSelfLoop(selfLoop);

    MemOps.clear();
  }

  DEBUG(print(dbgs(), 0));

  return false;
}


bool HWAtomInfo::haveSelfLoop(BasicBlock *BB) {
  Loop *L = LI->getLoopFor(BB);

  // Not in any loop.
  if (!L) return false;
  // Dirty Hack: Only support one block loop at this moment.
  if (L->getBlocks().size() != 1) return false;

  return true;
}

void HWAtomInfo::addLoopPredBackEdge(BasicBlock *BB) {
  assert(haveSelfLoop(BB) && "Loop SCC only exist in self loop!");

  Loop *L = LI->getLoopFor(BB);
  
  PHINode *IV = L->getCanonicalInductionVariable();
  // And we need loop in canonical form.
  if (!IV) return;

  FSMState *State = getState(BB);
  // Get the induction variable increment.
  //Instruction *IVInc = cast<Instruction>(IV->getIncomingValueForBlock(BB));
  //HWAOpInst *IVIncAtom = cast<HWAOpInst>(getAtomFor(*IVInc));
  // And get the predicate
  BranchInst *Br = cast<BranchInst>(BB->getTerminator());
  ICmpInst *ICmp = cast<ICmpInst>(Br->getCondition());
  HWAOpInst *Pred = cast<HWAOpInst>(getAtomFor(*ICmp));

  // The Next loop depend on the result of predicate.
  HWMemDep *LoopDep = getMemDepEdge(Pred, true, HWMemDep::TrueDep, 1);
  //IVIncAtom->addDep(LoopDep);
  State->addDep(LoopDep);
}

void HWAtomInfo::addMemDepEdges(std::vector<HWAOpInst*> &MemOps, BasicBlock &BB) {
  typedef std::vector<HWAOpInst*> OpInstVec;
  typedef MemDepInfo::DepInfo DepInfoType;
  for (OpInstVec::iterator SrcI = MemOps.begin(), SrcE = MemOps.end();
      SrcI != SrcE; ++SrcI) {
    HWAOpInst *Src = *SrcI;
    bool isSrcLoad = isa<LoadInst>(Src->getValue());

    for (OpInstVec::iterator DstI = MemOps.begin(), DstE = MemOps.end();
        DstI != DstE; ++DstI) {
      HWAOpInst *Dst = *DstI;
      bool isDstLoad = isa<LoadInst>(Dst->getValue());

      //No self loops
      if (Src == Dst)
        continue;
      
      bool srcBeforeDest = SrcI < DstI;

      // Get dependencies information.
      DepInfoType Dep = MDA->getDepInfo(Src->getInst<Instruction>(),
                                         Dst->getInst<Instruction>(),
                                         BB, srcBeforeDest);
      if (!Dep.hasDep())
        continue;

      // Add dependencies edges.
      // The edge is back edge if destination before source.
      HWMemDep *MemDep = getMemDepEdge(Src, !srcBeforeDest, 
                                       Dep.getDepType(), Dep.getItDst());
      Dst->addDep(MemDep);
    }
  }
}

void HWAtomInfo::clear() {
  HWAtomAllocator.Reset();
  UniqiueHWAtoms.clear();
  ValueToHWAtoms.clear();
  // Reset total Cycle
  totalCycle = 1;
  NumRegs = 1;
  InstIdx = 0;
  RegForValues.clear();
}

void HWAtomInfo::releaseMemory() {
  clear();
}

//===----------------------------------------------------------------------===//
// Construct atom from LLVM-IR
// What if the one of the operand is Constant?
HWAtom *HWAtomInfo::visitTerminatorInst(TerminatorInst &I) {
  // State end depand on or others atoms
  SmallVector<HWEdge*, 16> Deps;
  addOperandDeps(I, Deps);
  HWAtom *Pred = 0;
  // Do not add the operand twice
  if (!Deps.empty()) {
    Pred = Deps[0]->getSrc();
  }

  unsigned OpSize = Deps.size();

  // All node should finish before terminator run.
  FSMState *State = getState(I.getParent());
  for (usetree_iterator TI = State->usetree_begin(), TE = State->usetree_end();
      TI != TE; ++TI)
    if (*TI != Pred && TI->use_empty()) {
      HWAtom *A = *TI;
      bool isExport = false;
      if (HWAOpInst *OI = dyn_cast<HWAOpInst>(A)) {
        isExport = !OI->getInst<Instruction>().getType()->isVoidTy();
      }
      // We must wait until all atom finish.
      Deps.push_back(getCtrlDepEdge(A, isExport));
    }

  // Create delay atom for phi node.
  addPhiDelays(*I.getParent(), Deps);

  // Get the atom, Terminator do not have any latency
  // Do not count basicblocks as operands
  HWAPostBind *Atom = getPostBind(I, Deps, OpSize, RC->allocaTrivialFU(0));
  // This is a control atom.
  SetControlRoot(Atom);
  return Atom;
}


HWAtom *HWAtomInfo::visitPHINode(PHINode &I) {
  //// Create a physics register for phi node.
  //HWReg *Reg = getRegNumForLiveVal(I);
  //// Merge the export register to PHI node.
  //for (unsigned i = 0, e = I.getNumIncomingValues(); i != e; ++i)
  //  setRegNum(*I.getIncomingValue(i), Reg);

  return 0;
}

HWAtom *HWAtomInfo::visitSelectInst(SelectInst &I) {
  SmallVector<HWEdge*, 4> Deps;
  addOperandDeps(I, Deps);

  // FIXME: Read latency from configure file
  return getPostBind(I, Deps, RC->allocaTrivialFU(1));
}

HWAtom *HWAtomInfo::visitCastInst(CastInst &I) {
  SmallVector<HWEdge*, 1> Deps;
  Deps.push_back(getValDepInState(*I.getOperand(0), I.getParent()));
  // CastInst do not have any latency
  return getPostBind(I, Deps, RC->allocaTrivialFU(0));
}

HWAtom *HWAtomInfo::visitLoadInst(LoadInst &I) {
  SmallVector<HWEdge*, 2> Deps;
  addOperandDeps(I, Deps);

  // Dirty Hack: allocate membus 1 to all load/store at this moment
  HWAtom *LoadAtom = getPreBind(I, Deps, RC->allocaFU(HWResource::MemoryBus, 1));

  return LoadAtom;
}

HWAtom *HWAtomInfo::visitStoreInst(StoreInst &I) {
  SmallVector<HWEdge*, 2> Deps;
  addOperandDeps(I, Deps);

  // Push the control root base on dependence analysis
  // Deps.push_back(getControlRoot());

  HWResource *Res = RC->getResource(HWResource::MemoryBus);
  assert(Res && "Can find resource!");

  // Dirty Hack: allocate membus 0 to all load/store at this moment
  HWAtom *StoreAtom = getPreBind(I, Deps, Res->allocaFU(1));
  // Set as new atom
  SetControlRoot(StoreAtom);

  return StoreAtom;
}

HWAtom *HWAtomInfo::visitGetElementPtrInst(GetElementPtrInst &I) {
  const Type *Ty = I.getOperand(0)->getType();
  assert(isa<SequentialType>(Ty) && "GEP type not support yet!");
  assert(I.getNumIndices() < 2 && "Too much indices in GEP!");

  SmallVector<HWEdge*, 2> Deps;
  addOperandDeps(I, Deps);
  return getPostBind(I, Deps, RC->allocaFU(HWResource::AddSub));
}

HWAtom *HWAtomInfo::visitICmpInst(ICmpInst &I) {
  // Get the operand;
  SmallVector<HWEdge*, 2> Deps;
  // LHS
  Deps.push_back(getValDepInState(*I.getOperand(0), I.getParent(), I.isSigned()));
  // RHS
  Deps.push_back(getValDepInState(*I.getOperand(1), I.getParent(), I.isSigned()));

  // It is trivial if one of the operand is constant
  if (isa<Constant>(I.getOperand(0)) || isa<Constant>(I.getOperand(1)))
    return getPostBind(I, Deps, RC->allocaTrivialFU(1));
  else // We need to do a subtraction for the comparison.
    return getPostBind(I, Deps, RC->allocaTrivialFU(1));
}

HWAtom *HWAtomInfo::visitBinaryOperator(Instruction &I) {
  // Get the operand;
  SmallVector<HWEdge*, 2> Deps;
  bool isSigned = false;
  bool isTrivial = isa<Constant>(I.getOperand(0)) || isa<Constant>(I.getOperand(1));
  HWFUnit FU;
  // Select the resource
  switch (I.getOpcode()) {
    case Instruction::Add:
    case Instruction::Sub:
      //T = isTrivial ? HWResource::Trivial : HWResource::AddSub;
      FU = RC->allocaFU(HWResource::AddSub);
      break;
    case Instruction::Mul:
      FU = RC->allocaFU(HWResource::Mul);
      break;
    case Instruction::And:
    case Instruction::Or:
    case Instruction::Xor:
      FU = RC->allocaTrivialFU(1);
      break;
    case Instruction::AShr:
      // Add the signed prefix for lhs
      isSigned = true;
      FU = isTrivial ? RC->allocaTrivialFU(1) : RC->allocaFU(HWResource::ASR);
      break;
    case Instruction::LShr:
      FU = isTrivial ? RC->allocaTrivialFU(1) : RC->allocaFU(HWResource::LSR);
      break;
    case Instruction::Shl:
      FU = isTrivial ? RC->allocaTrivialFU(1) : RC->allocaFU(HWResource::SHL);
      break;
    default: 
      llvm_unreachable("Instruction not support yet!");
  }
  // LHS
  Deps.push_back(getValDepInState(*I.getOperand(0), I.getParent(), isSigned));
  // RHS
  Deps.push_back(getValDepInState(*I.getOperand(1), I.getParent()));
  
  return getPostBind(I, Deps, FU);
}

//===----------------------------------------------------------------------===//
// Create atom

HWAPreBind *HWAtomInfo::bindToResource(HWAPostBind &PostBind, unsigned Instance)
{
  assert(Instance && "Instance can not be 0 !");
  UniqiueHWAtoms.RemoveNode(&PostBind);

  FoldingSetNodeID ID;
  ID.AddInteger(atomPreBind);
  ID.AddPointer(&PostBind.getValue());

  void *IP = 0;
  HWAPreBind *A =
    static_cast<HWAPreBind*>(UniqiueHWAtoms.FindNodeOrInsertPos(ID, IP));

  assert(!A && "Why the instruction is bind?");

  HWFUnit FU = RC->allocaFU(PostBind.getResClass(), Instance);

  A = new (HWAtomAllocator) HWAPreBind(ID.Intern(HWAtomAllocator),
                                       PostBind, FU);

  // Update the Map.
  ValueToHWAtoms[&PostBind.getValue()] = A;
  UniqiueHWAtoms.InsertNode(A, IP);

  return A;
}

HWAPreBind *HWAtomInfo::getPreBind(Instruction &I,
                                   SmallVectorImpl<HWEdge*> &Deps,
                                   size_t OpNum, HWFUnit FUID) {
  FoldingSetNodeID ID;
  ID.AddInteger(atomPreBind);
  ID.AddPointer(&I);

  void *IP = 0;
  HWAPreBind *A =
    static_cast<HWAPreBind*>(UniqiueHWAtoms.FindNodeOrInsertPos(ID, IP));

  if (!A) {
    A = new (HWAtomAllocator) HWAPreBind(ID.Intern(HWAtomAllocator),
      I, Deps.begin(), Deps.end(), OpNum, FUID, ++InstIdx);
    UniqiueHWAtoms.InsertNode(A, IP);
  }
  return A;
}

HWAPostBind *HWAtomInfo::getPostBind(Instruction &I,
                                     SmallVectorImpl<HWEdge*> &Deps,
                                     size_t OpNum, HWFUnit FUID) {
  FoldingSetNodeID ID;
  ID.AddInteger(atomPostBind);
  ID.AddPointer(&I);

  void *IP = 0;
  HWAPostBind *A =
    static_cast<HWAPostBind*>(UniqiueHWAtoms.FindNodeOrInsertPos(ID, IP));

  if (!A) {
    A = new (HWAtomAllocator) HWAPostBind(ID.Intern(HWAtomAllocator),
      I, Deps.begin(), Deps.end(), OpNum, FUID, ++InstIdx);
    UniqiueHWAtoms.InsertNode(A, IP);
  }
  return A;
}

FSMState *HWAtomInfo::getState(BasicBlock *BB) {
  assert(BB && "BB can not be null!");
  FoldingSetNodeID ID;
  ID.AddInteger(atomVRoot);
  ID.AddPointer(BB);

  void *IP = 0;
  FSMState *A =
    static_cast<FSMState*>(UniqiueHWAtoms.FindNodeOrInsertPos(ID, IP));

  if (!A) {
    A = new (HWAtomAllocator) FSMState(ID.Intern(HWAtomAllocator), *BB,
                                       ++InstIdx);
    UniqiueHWAtoms.InsertNode(A, IP);
  }
  return A;
}

HWAWrReg *HWAtomInfo::getWrReg(HWAtom *Src, HWRegister *Reg) {
  FoldingSetNodeID ID;
  ID.AddInteger(atomWrReg);
  ID.AddPointer(Src);
  ID.AddPointer(Reg);

  void *IP = 0;
  HWAWrReg *A =
    static_cast<HWAWrReg*>(UniqiueHWAtoms.FindNodeOrInsertPos(ID, IP));

  if (!A) {
    A = new (HWAtomAllocator) HWAWrReg(ID.Intern(HWAtomAllocator),
                                       *getValDepEdge(Src, false), Reg);
    UniqiueHWAtoms.InsertNode(A, IP);
  }
  return A;
}

HWADelay *HWAtomInfo::getDelay(HWAtom *Src, unsigned Delay) {
  FoldingSetNodeID ID;
  ID.AddInteger(atomDelay);
  ID.AddPointer(Src);
  ID.AddInteger(Delay);

  void *IP = 0;
  HWADelay *A =
    static_cast<HWADelay*>(UniqiueHWAtoms.FindNodeOrInsertPos(ID, IP));

  if (!A) {
    A = new (HWAtomAllocator) HWADelay(ID.Intern(HWAtomAllocator),
      *getCtrlDepEdge(Src, false), Delay, ++InstIdx);
    UniqiueHWAtoms.InsertNode(A, IP);
  }
  return A;
}


HWARdReg *HWAtomInfo::getRdReg(HWAtom *Src, HWRegister *Reg, Value &V) {
  FoldingSetNodeID ID;
  ID.AddInteger(atomRdReg);
  ID.AddPointer(Src);
  ID.AddPointer(Reg);
  ID.AddPointer(&V);

  void *IP = 0;
  HWARdReg *A =
    static_cast<HWARdReg*>(UniqiueHWAtoms.FindNodeOrInsertPos(ID, IP));

  if (!A) {
    A = new (HWAtomAllocator) HWARdReg(ID.Intern(HWAtomAllocator),
      *getValDepEdge(Src, false), Reg, V);
    UniqiueHWAtoms.InsertNode(A, IP);
  }
  return A;
}


HWMemDep *HWAtomInfo::getMemDepEdge(HWAtom *Src, bool isBackEdge,
                                    enum HWMemDep::MemDepTypes DepType,
                                    unsigned Diff) {
  return new (HWAtomAllocator) HWMemDep(Src, isBackEdge, DepType, Diff);
}

void HWAtomInfo::print(raw_ostream &O, const Module *M) const {}

void HWAtomInfo::addPhiDelays(BasicBlock &BB, SmallVectorImpl<HWEdge*> &Deps) {
  FSMState *State = getState(&BB);
  for (succ_iterator SI = succ_begin(&BB), SE = succ_end(&BB); SI != SE; ++SI){
    BasicBlock *SuccBB = *SI;
    for (BasicBlock::iterator II = SuccBB->begin(),
        IE = SuccBB->getFirstNonPHI(); II != IE; ++II) {
      PHINode *PN = cast<PHINode>(II);
      Instruction *Inst =
        dyn_cast<Instruction>(PN->getIncomingValueForBlock(&BB));
      // No instruction value do not need to delay.
      if (!Inst)
        continue;
      // We do not need to delay if the value not define in the current BB.
      if (Inst->getParent() != &BB)
        continue;
      // We do not need to delay if source is PHI, because it is ready before
      // entering this FSMState.
      if (isa<PHINode>(Inst))
        continue;

      HWAOpInst *OpInst = cast<HWAOpInst>(getAtomFor(*Inst));
      //// We do not delay the trivial operation, because the delay
      //// is cause by the function unit register.
      //if (OpInst->isTrivial())
      //  continue;

      HWADelay *Delay = getDelay(OpInst, 1);
      Deps.push_back(getCtrlDepEdge(Delay));

      if (&BB == SuccBB) {// Self Loop?
        // The Next loop depend on the result of phi.
        HWMemDep *PHIDep = getMemDepEdge(Delay, true, HWMemDep::TrueDep, 1);
        //IVIncAtom->addDep(LoopDep);
        State->addDep(PHIDep);
      }
    }
  }
}