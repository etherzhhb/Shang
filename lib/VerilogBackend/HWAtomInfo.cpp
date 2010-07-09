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

#include "llvm/Analysis/LoopInfo.h"
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
  AU.addRequiredTransitive<ResourceConfig>();
  AU.setPreservesAll();
}

bool HWAtomInfo::runOnFunction(Function &F) {
  LI = &getAnalysis<LoopInfo>();
  RC = &getAnalysis<ResourceConfig>();

  for (Function::iterator I = F.begin(), E = F.end(); I != E; ++I) {
    // Setup the state.
    BasicBlock *BB = &*I;
    SetControlRoot(getEntryRoot(BB));
    DEBUG(dbgs() << "Building atom for BB: " << BB->getName() << '\n');
    for (BasicBlock::iterator BI = BB->begin(), BE = BB->end();
        BI != BE; ++BI) {
      visit(*BI);
    }
    BBToStates[BB] = new FSMState(getEntryRoot(BB),
                                   cast<HWAPostBind>(getControlRoot()));
  }

  DEBUG(print(dbgs(), 0));

  return false;
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
void HWAtomInfo::visitTerminatorInst(TerminatorInst &I) {
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
  // Remember the atom.
  ValueToHWAtoms.insert(std::make_pair(&I, Atom));

  // This is a control atom.
  SetControlRoot(Atom);
}


void HWAtomInfo::visitPHINode(PHINode &I) {
  // Create a physics register for phi node.
  HWReg *Reg = getRegNumForLiveVal(I);
  // Merge the export register to PHI node.
  for (unsigned i = 0, e = I.getNumIncomingValues(); i != e; ++i)
    setRegNum(*I.getIncomingValue(i), Reg);
}

void HWAtomInfo::visitSelectInst(SelectInst &I) {
  SmallVector<HWEdge*, 4> Deps;
  addOperandDeps(I, Deps);

  // FIXME: Read latency from configure file
  HWAtom *SelAtom = getPostBind(I, Deps, 1, HWResource::Trivial);

  ValueToHWAtoms.insert(std::make_pair(&I, SelAtom));
}

void HWAtomInfo::visitCastInst(CastInst &I) {
  SmallVector<HWEdge*, 1> Deps;
  Deps.push_back(getValDepInState(*I.getOperand(0), I.getParent()));
  // CastInst do not have any latency
  // FIXME: Create the AtomReAssign Pass and set the latency to 0
  HWAtom *CastAtom = getPostBind(I, Deps, 1, HWResource::Trivial);
  ValueToHWAtoms.insert(std::make_pair(&I, CastAtom));
}

void HWAtomInfo::visitLoadInst(LoadInst &I) {
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

  ValueToHWAtoms.insert(std::make_pair(&I, LoadAtom));
}

void HWAtomInfo::visitStoreInst(StoreInst &I) {
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

  ValueToHWAtoms.insert(std::make_pair(&I, StoreAtom));
}

void HWAtomInfo::visitGetElementPtrInst(GetElementPtrInst &I) {
  const Type *Ty = I.getOperand(0)->getType();
  assert(isa<SequentialType>(Ty) && "GEP type not support yet!");
  assert(I.getNumIndices() < 2 && "Too much indices in GEP!");

  SmallVector<HWEdge*, 2> Deps;
  addOperandDeps(I, Deps);

  HWAtom *GEPAtom = getPostBind(I, Deps, 1, HWResource::AddSub);

  ValueToHWAtoms.insert(std::make_pair(&I, GEPAtom));
}

void HWAtomInfo::visitICmpInst(ICmpInst &I) {
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
  HWAtom *CmpAtom = getPostBind(I, Deps, 1, T);

  ValueToHWAtoms.insert(std::make_pair(&I, CmpAtom));
}

void HWAtomInfo::visitBinaryOperator(Instruction &I) {
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
  
  HWAtom *BinOpAtom = getPostBind(I, Deps, 1, T);

  ValueToHWAtoms.insert(
    std::make_pair(&I, BinOpAtom));
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

void HWAtomInfo::print(raw_ostream &O, const Module *M) const {
  for (StateMapType::const_iterator I = BBToStates.begin(), E = BBToStates.end();
      I != E; ++I) {
    I->second->print(O);
  }
}
