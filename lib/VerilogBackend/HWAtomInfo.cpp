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
    SetControlRoot(getEntryRoot(BB));
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

    BBToStates[BB] = new FSMState(getEntryRoot(BB),
                                   cast<HWAPostBind>(getControlRoot()));

    MemOps.clear();
  }

  DEBUG(print(dbgs(), 0));

  return false;
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
      HWMemDep *MemDep = getMemDepEdge(Src, getEntryRoot(&BB),
                                        Dep.getDepType(), Dep.getItDst());
      Dst->addDep(MemDep);
    }
  }
}

void HWAtomInfo::clear() {
  HWAtomAllocator.Reset();
  UniqiueHWAtoms.clear();
  ValueToHWAtoms.clear();
  BBToStates.clear();
  // Reset total Cycle
  totalCycle = 1;
  NumRegs = 1;
  LiveValueReg.clear();
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
  HWAVRoot *Root = getEntryRoot(I.getParent());
  for (usetree_iterator TI = Root->begin(), TE = Root->end();
      TI != TE; ++TI)
    if (*TI != Pred && TI->use_empty()) {
      HWAtom *A = *TI;
      if (HWAOpInst *OI = dyn_cast<HWAOpInst>(A)) {
        Instruction &Inst = OI->getInst<Instruction>();
        if (!Inst.getType()->isVoidTy()) {
          Deps.push_back(getValDepEdge(A, getRegNumForLiveVal(Inst)));
          continue;
        }
      }
      Deps.push_back(getCtrlDepEdge(A));
    }

  // Get the atom, Terminator do not have any latency
  // Do not count basicblocks as operands
  HWAPostBind *Atom = getPostBind(I, Deps, OpSize, 0, HWResource::Trivial);
  // This is a control atom.
  SetControlRoot(Atom);
  return Atom;
}


HWAtom *HWAtomInfo::visitPHINode(PHINode &I) {
  // Create a physics register for phi node.
  HWReg *Reg = getRegNumForLiveVal(I);
  // Merge the export register to PHI node.
  for (unsigned i = 0, e = I.getNumIncomingValues(); i != e; ++i)
    setRegNum(*I.getIncomingValue(i), Reg);

  return 0;
}

HWAtom *HWAtomInfo::visitSelectInst(SelectInst &I) {
  SmallVector<HWEdge*, 4> Deps;
  addOperandDeps(I, Deps);

  // FIXME: Read latency from configure file
  return getPostBind(I, Deps, 1, HWResource::Trivial);
}

HWAtom *HWAtomInfo::visitCastInst(CastInst &I) {
  SmallVector<HWEdge*, 1> Deps;
  Deps.push_back(getValDepInState(*I.getOperand(0), I.getParent()));
  // CastInst do not have any latency
  return getPostBind(I, Deps, 0, HWResource::Trivial);
}

HWAtom *HWAtomInfo::visitLoadInst(LoadInst &I) {
  SmallVector<HWEdge*, 2> Deps;
  addOperandDeps(I, Deps);

  // Push the control root base on dependence analysis
  // Deps.push_back(getControlRoot());

  HWResource *Res = RC->getResource(HWResource::MemoryBus);
  assert(Res && "Can find resource!");

  // Dirty Hack: allocate membus 0 to all load/store at this moment
  HWAtom *LoadAtom = getPreBind(I, Deps, HWResource::MemoryBus,
                                Res->getLatency(), 0);
  // Set as new atom
  SetControlRoot(LoadAtom);

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
  HWAtom *StoreAtom = getPreBind(I, Deps, HWResource::MemoryBus,
                                 Res->getLatency(), 0);
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

  return getPostBind(I, Deps, 1, HWResource::AddSub);
}

HWAtom *HWAtomInfo::visitICmpInst(ICmpInst &I) {
  // Get the operand;
  SmallVector<HWEdge*, 2> Deps;
  // LHS
  Deps.push_back(getValDepInState(*I.getOperand(0), I.getParent(), I.isSigned()));
  // RHS
  Deps.push_back(getValDepInState(*I.getOperand(1), I.getParent(), I.isSigned()));

  enum HWResource::ResTypes T;
  // It is trivial if one of the operand is constant
  if (isa<Constant>(I.getOperand(0)) || isa<Constant>(I.getOperand(1)))
    T = HWResource::Trivial;
  else // We need to do a subtraction for the comparison.
    T = HWResource::Trivial;

  // FIXME: Read latency from configure file
  // 
  return getPostBind(I, Deps, 1, T);
}

HWAtom *HWAtomInfo::visitBinaryOperator(Instruction &I) {
  // Get the operand;
  SmallVector<HWEdge*, 2> Deps;
  bool isSigned = false;
  bool isTrivial = isa<Constant>(I.getOperand(0)) || isa<Constant>(I.getOperand(1));
  enum HWResource::ResTypes T;
  // Select the resource
  switch (I.getOpcode()) {
    case Instruction::Add:
    case Instruction::Sub:
      //T = isTrivial ? HWResource::Trivial : HWResource::AddSub;
      T = HWResource::AddSub;
      break;
    case Instruction::Mul:
      T = HWResource::Mul;
      break;
    case Instruction::And:
    case Instruction::Or:
    case Instruction::Xor:
      T = HWResource::Trivial;
      break;
    case Instruction::AShr:
      // Add the signed prefix for lhs
      isSigned = true;
      T = isTrivial ? HWResource::Trivial : HWResource::ASR;
      break;
    case Instruction::LShr:
      T = isTrivial ? HWResource::Trivial : HWResource::LSR;
      break;
    case Instruction::Shl:
      T = isTrivial ? HWResource::Trivial : HWResource::SHL;
      break;
    default: 
      llvm_unreachable("Instruction not support yet!");
  }
  // LHS
  Deps.push_back(getValDepInState(*I.getOperand(0), I.getParent(), isSigned));
  // RHS
  Deps.push_back(getValDepInState(*I.getOperand(1), I.getParent()));
  
  return getPostBind(I, Deps, 1, T);
}

//===----------------------------------------------------------------------===//
// Create atom

HWAPreBind *HWAtomInfo::bindToResource(HWAPostBind &PostBind,
                                              unsigned Instance) {
  FoldingSetNodeID ID;
  ID.AddInteger(atomPreBind);
  ID.AddPointer(&PostBind.getValue());

  void *IP = 0;
  HWAPreBind *A =
    static_cast<HWAPreBind*>(UniqiueHWAtoms.FindNodeOrInsertPos(ID, IP));

  assert(!A && "Why the instruction is bind?");

  A = new (HWAtomAllocator) HWAPreBind(ID.Intern(HWAtomAllocator),
                                       PostBind, Instance);

  updateAtomMap(PostBind.getValue(), A); 

  UniqiueHWAtoms.InsertNode(A, IP);
  UniqiueHWAtoms.RemoveNode(&PostBind);

  return A;
}

HWAPreBind *HWAtomInfo::getPreBind(Instruction &I, SmallVectorImpl<HWEdge*> &Deps,
                               size_t OpNum, enum HWResource::ResTypes OpClass,
                               unsigned latency, unsigned ResInst) {
  FoldingSetNodeID ID;
  ID.AddInteger(atomPreBind);
  ID.AddPointer(&I);

  void *IP = 0;
  HWAPreBind *A =
    static_cast<HWAPreBind*>(UniqiueHWAtoms.FindNodeOrInsertPos(ID, IP));

  if (!A) {
    A = new (HWAtomAllocator) HWAPreBind(ID.Intern(HWAtomAllocator),
      I, latency, Deps.begin(), Deps.end(), OpNum, OpClass, ResInst);
    UniqiueHWAtoms.InsertNode(A, IP);
  }
  return A;
}

HWAPostBind *HWAtomInfo::getPostBind(Instruction &I, SmallVectorImpl<HWEdge*> &Deps,
                                 size_t OpNum, unsigned latency,
                                 enum HWResource::ResTypes OpClass) {
  FoldingSetNodeID ID;
  ID.AddInteger(atomPostBind);
  ID.AddPointer(&I);

  void *IP = 0;
  HWAPostBind *A =
    static_cast<HWAPostBind*>(UniqiueHWAtoms.FindNodeOrInsertPos(ID, IP));

  if (!A) {
    A = new (HWAtomAllocator) HWAPostBind(ID.Intern(HWAtomAllocator),
      I, latency, Deps.begin(), Deps.end(), OpNum, OpClass);
    UniqiueHWAtoms.InsertNode(A, IP);
  }
  return A;
}

HWAVRoot *HWAtomInfo::getEntryRoot(BasicBlock *BB) {
  assert(BB && "BB can not be null!");
  FoldingSetNodeID ID;
  ID.AddInteger(atomVRoot);
  ID.AddPointer(BB);

  void *IP = 0;
  HWAVRoot *A =
    static_cast<HWAVRoot*>(UniqiueHWAtoms.FindNodeOrInsertPos(ID, IP));

  if (!A) {
    A = new (HWAtomAllocator) HWAVRoot(ID.Intern(HWAtomAllocator), *BB);
    UniqiueHWAtoms.InsertNode(A, IP);
  }
  return A;
}

HWADrvReg *HWAtomInfo::getDrvReg(HWAtom *Src, HWReg *Reg) {
  FoldingSetNodeID ID;
  ID.AddInteger(atomDrvReg);
  ID.AddPointer(Src);
  ID.AddPointer(Reg);

  void *IP = 0;
  HWADrvReg *A =
    static_cast<HWADrvReg*>(UniqiueHWAtoms.FindNodeOrInsertPos(ID, IP));

  if (!A) {
    A = new (HWAtomAllocator) HWADrvReg(ID.Intern(HWAtomAllocator), getValDepEdge(Src, Reg));
    UniqiueHWAtoms.InsertNode(A, IP);
  }
  return A;
}


HWMemDep *HWAtomInfo::getMemDepEdge(HWAOpInst *Src, HWAVRoot *Root,
                                    enum HWMemDep::MemDepTypes DepType,
                                    unsigned Diff) {
  return new (HWAtomAllocator) HWMemDep(Diff > 0 ? (HWAtom*)Root : (HWAtom*)Src,
                                        Src, DepType, Diff);
}

void HWAtomInfo::print(raw_ostream &O, const Module *M) const {
  for (StateMapType::const_iterator I = BBToStates.begin(), E = BBToStates.end();
      I != E; ++I) {
    I->second->print(O);
  }
}
