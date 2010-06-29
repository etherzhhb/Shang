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
    BBToStates[BB] = new ExecStage(getEntryRoot(BB),
                                   cast<HWAOpInst>(getControlRoot()));
    // AtomsInCurStates not use any more
    AtomsInCurState.clear();
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
}

void HWAtomInfo::releaseMemory() {
  clear();
}

//===----------------------------------------------------------------------===//
// Construct atom from LLVM-IR
// What if the one of the operand is Constant?
void HWAtomInfo::visitTerminatorInst(TerminatorInst &I) {
  // State end depand on or others atoms
  SmallVector<HWAtom*, 1> Deps;
  addOperandDeps(I, Deps);
  // Do not add the operand twice
  if (!Deps.empty())
    AtomsInCurState.erase(Deps[0]);
  unsigned OpSize = Deps.size();
  // Only emit the terminator when all others operation emited.
  Deps.append(AtomsInCurState.begin(), AtomsInCurState.end());

  // Get the atom, Terminator do not have any latency
  // Do not count basicblocks as operands
  HWAOpInst *Atom = getOpInst(I, Deps, OpSize, 0, HWResource::Other);
  // Remember the atom.
  ValueToHWAtoms.insert(std::make_pair<const Instruction*, HWAtom*>(&I, Atom));

  // This is a control atom.
  SetControlRoot(Atom);
}


void HWAtomInfo::visitPHINode(PHINode &I) {
  SmallVector<HWAtom*, 0> Deps;
  // PHI node always have no latency
  HWAtom *Phi = getOpInst(I, Deps, 0, 0, HWResource::Other);
  ValueToHWAtoms.insert(std::make_pair<const Instruction*, HWAtom*>(&I, Phi));
}

void HWAtomInfo::visitSelectInst(SelectInst &I) {
  SmallVector<HWAtom*, 4> Deps;
  addOperandDeps(I, Deps);

  // FIXME: Read latency from configure file
  HWAtom *SelAtom = getOpInst(I, Deps, 1, HWResource::Other);
  // Register the result
  SelAtom = getRegister(I, SelAtom);

  ValueToHWAtoms.insert(std::make_pair<const Instruction*, HWAtom*>(&I, SelAtom));
}

void HWAtomInfo::visitCastInst(CastInst &I) {
  SmallVector<HWAtom*, 1> Deps;
  Deps.push_back(getAtomInState(*I.getOperand(0), I.getParent()));
  // CastInst do not have any latency
  // FIXME: Create the AtomReAssign Pass and set the latency to 0
  HWAtom *CastAtom = getOpInst(I, Deps, 1, HWResource::Other);
  // Register the result
  CastAtom = getRegister(I, CastAtom);
  ValueToHWAtoms.insert(std::make_pair<const Instruction*, HWAtom*>(&I, CastAtom));
}

void HWAtomInfo::visitLoadInst(LoadInst &I) {
  SmallVector<HWAtom*, 2> Deps;
  addOperandDeps(I, Deps);

  // Push the control root base on dependence analysis
  Deps.push_back(getControlRoot());

  HWResource *Res = RC->getResource(HWResource::MemoryBus);
  assert(Res && "Can find resource!");

  // Dirty Hack: allocate membus 1 to all load/store at this moment
  HWAtom *LoadAtom = getOpRes(I, Deps, *Res, Res->getLatency(), 1);
  // Set as new atom
  SetControlRoot(LoadAtom);
  // And register the result
  LoadAtom =  getRegister(I, LoadAtom);

  ValueToHWAtoms.insert(std::make_pair<const Instruction*, HWAtom*>(&I, LoadAtom));
}

void HWAtomInfo::visitStoreInst(StoreInst &I) {
  SmallVector<HWAtom*, 2> Deps;
  addOperandDeps(I, Deps);

  // Push the control root base on dependence analysis
  Deps.push_back(getControlRoot());

  HWResource *Res = RC->getResource(HWResource::MemoryBus);
  assert(Res && "Can find resource!");

  // Dirty Hack: allocate membus 1 to all load/store at this moment
  HWAtom *StoreAtom = getOpRes(I, Deps, *Res, Res->getLatency(), 1);
  // Set as new atom
  SetControlRoot(StoreAtom);

  ValueToHWAtoms.insert(std::make_pair<const Instruction*, HWAtom*>(&I, StoreAtom));
}

void HWAtomInfo::visitGetElementPtrInst(GetElementPtrInst &I) {
  const Type *Ty = I.getOperand(0)->getType();
  assert(isa<SequentialType>(Ty) && "GEP type not support yet!");
  assert(I.getNumIndices() < 2 && "Too much indices in GEP!");

  SmallVector<HWAtom*, 2> Deps;
  addOperandDeps(I, Deps);

  HWAtom *GEPAtom = 0;

  // Create the atom
  if (HWResource *Res = RC->getResource(HWResource::AddSub))
    GEPAtom = getOpRes(I, Deps, *Res, Res->getLatency());
  else
    // FIXME: Read latency from configure file
    GEPAtom = getOpInst(I, Deps, 1, HWResource::AddSub);

  // Register the result
  GEPAtom = getRegister(I, GEPAtom);

  ValueToHWAtoms.insert(std::make_pair<const Instruction*, HWAtom*>(&I, GEPAtom));
}

void HWAtomInfo::visitICmpInst(ICmpInst &I) {
  // Get the operand;
  SmallVector<HWAtom*, 2> Deps;
  // LHS
  Deps.push_back(getAtomInState(*I.getOperand(0), I.getParent()));
  // RHS
  Deps.push_back(getAtomInState(*I.getOperand(1), I.getParent()));

  enum HWResource::ResTypes T;
  // It is trivial if one of the operand is constant
  if (isa<Constant>(I.getOperand(0)) || isa<Constant>(I.getOperand(1)))
    T = HWResource::Other;
  else // We need to do a subtraction for the comparison.
    T = HWResource::AddSub;

  if (I.isSigned()) {
    Deps[0] = getSigned(Deps[0]);
    Deps[1] = getSigned(Deps[1]);
  }

  // FIXME: Read latency from configure file
  // 
  HWAtom *CmpAtom = getOpInst(I, Deps, 1, T);
  // Register it
  CmpAtom = getRegister(I, CmpAtom);

  ValueToHWAtoms.insert(std::make_pair<const Instruction*, HWAtom*>(&I, CmpAtom));
}

void HWAtomInfo::visitBinaryOperator(Instruction &I) {
  // Get the operand;
  SmallVector<HWAtom*, 2> Deps;
  // LHS
  Deps.push_back(getAtomInState(*I.getOperand(0), I.getParent()));
  // RHS
  Deps.push_back(getAtomInState(*I.getOperand(1), I.getParent()));
  bool isTrivial = isa<Constant>(I.getOperand(0)) || isa<Constant>(I.getOperand(1));
  enum HWResource::ResTypes T;
  // Select the resource
  switch (I.getOpcode()) {
    case Instruction::Add:
    case Instruction::Sub:
      T = HWResource::AddSub;
      break;
    case Instruction::Mul:
      T = HWResource::Mul;
      break;
    case Instruction::And:
    case Instruction::Or:
    case Instruction::Xor:
      T = HWResource::Other;
      break;
    case Instruction::AShr:
      // Add the signed prefix for lhs
      Deps[0] = getSigned(Deps[0]);
      T = isTrivial ? HWResource::Other : HWResource::ASR;
      break;
    case Instruction::LShr:
      T = isTrivial ? HWResource::Other : HWResource::LSR;
      break;
    case Instruction::Shl:
      T = isTrivial ? HWResource::Other : HWResource::SHL;
      break;
    default: 
      llvm_unreachable("Instruction not support yet!");
  }
  
  HWAtom *BinOpAtom = 0;
  if (HWResource *Res = RC->getResource(T))
    BinOpAtom = getOpRes(I, Deps, *Res, Res->getLatency());
  else
    // FIXME: Read latency from configure file
    BinOpAtom = getOpInst(I, Deps, 1, T);

  // Register it
  BinOpAtom = getRegister(I, BinOpAtom);

  ValueToHWAtoms.insert(
    std::make_pair<const Instruction*, HWAtom*>(&I, BinOpAtom));
}

//===----------------------------------------------------------------------===//
// Create atom

HWAtom *HWAtomInfo::getConstant(Value &V) {
  assert((isa<Constant>(V) || isa<Argument>(V)) && "Not a constant!");
  AtomMapType::iterator At = ValueToHWAtoms.find(&V);
  if (At != ValueToHWAtoms.end())
    return At->second;

  FoldingSetNodeID ID;
  HWAtom *A = new (HWAtomAllocator) HWAConst(ID.Intern(HWAtomAllocator), V);
  //UniqiueHWAtoms.InsertNode(A, IP);
  ValueToHWAtoms.insert(std::make_pair<const Value*, HWAtom*>(&V, A));
  return A;
}

HWARegister *HWAtomInfo::getRegister(Value &V, HWAtom *Using) {
  FoldingSetNodeID ID;
  ID.AddInteger(atomRegister);
  ID.AddPointer(&V);
  ID.AddPointer(Using);

  void *IP = 0;
  HWARegister *A =
    static_cast<HWARegister*>(UniqiueHWAtoms.FindNodeOrInsertPos(ID, IP));

  if (!A) {
    HWAtom **O = HWAtomAllocator.Allocate<HWAtom *>(1);
    O[0] = Using;
    A = new (HWAtomAllocator) HWARegister(ID.Intern(HWAtomAllocator), V , O);
    UniqiueHWAtoms.InsertNode(A, IP);
    // Add New Atom
    AtomsInCurState.insert(A);
  }
  return A;
}

HWAtom *HWAtomInfo::getSigned(HWAtom *Using) {
  // Do not add the signed prefix for a constant.
  if (isa<Constant>(Using->getValue()))
    return Using;
  
  FoldingSetNodeID ID;
  ID.AddInteger(atomSignedPrefix);
  ID.AddPointer(Using);

  void *IP = 0;
  HWASigned *A =
    static_cast<HWASigned*>(UniqiueHWAtoms.FindNodeOrInsertPos(ID, IP));

  if (!A) {
    HWAtom **O = HWAtomAllocator.Allocate<HWAtom *>(1);
    O[0] = Using;
    A = new (HWAtomAllocator) HWASigned(ID.Intern(HWAtomAllocator), O);
    UniqiueHWAtoms.InsertNode(A, IP);
    // Add New Atom
    AtomsInCurState.insert(A);
  }
  return A;
}

HWAOpRes *HWAtomInfo::getOpRes(Instruction &I, SmallVectorImpl<HWAtom*> &Deps,
                               size_t OpNum, HWResource &Res, unsigned latency,
                               unsigned ResInst) {
  FoldingSetNodeID ID;
  ID.AddInteger(atomOpRes);
  ID.AddPointer(&I);

  void *IP = 0;
  HWAOpRes *A =
    static_cast<HWAOpRes*>(UniqiueHWAtoms.FindNodeOrInsertPos(ID, IP));

  if (!A) {
    HWAtom **O = HWAtomAllocator.Allocate<HWAtom *>(Deps.size());
    std::uninitialized_copy(Deps.begin(), Deps.end(), O);
    A = new (HWAtomAllocator) HWAOpRes(ID.Intern(HWAtomAllocator),
      I, latency, O, Deps.size(), OpNum, Res, ResInst);
    UniqiueHWAtoms.InsertNode(A, IP);
    // Add New Atom
    AtomsInCurState.insert(A);
  }
  return A;
}

HWAOpInst *HWAtomInfo::getOpInst(Instruction &I, SmallVectorImpl<HWAtom*> &Deps,
                                 size_t OpNum, unsigned latency,
                                 enum HWResource::ResTypes OpClass) {
  FoldingSetNodeID ID;
  ID.AddInteger(atomOpInst);
  ID.AddPointer(&I);

  void *IP = 0;
  HWAOpInst *A =
    static_cast<HWAOpInst*>(UniqiueHWAtoms.FindNodeOrInsertPos(ID, IP));

  if (!A) {
    HWAtom **O = HWAtomAllocator.Allocate<HWAtom *>(Deps.size());
    std::uninitialized_copy(Deps.begin(), Deps.end(), O);
    A = new (HWAtomAllocator) HWAOpInst(ID.Intern(HWAtomAllocator),
      I, latency, O, Deps.size(), OpNum, OpClass);
    UniqiueHWAtoms.InsertNode(A, IP);
    // Add New Atom
    AtomsInCurState.insert(A);
  }
  return A;
}


HWAEntryRoot *HWAtomInfo::getEntryRoot(BasicBlock *BB) {
  assert(BB && "BB can not be null!");
  FoldingSetNodeID ID;
  ID.AddInteger(atomEntryRoot);
  ID.AddPointer(BB);

  void *IP = 0;
  HWAEntryRoot *A =
    static_cast<HWAEntryRoot*>(UniqiueHWAtoms.FindNodeOrInsertPos(ID, IP));

  if (!A) {
    A = new (HWAtomAllocator) HWAEntryRoot(ID.Intern(HWAtomAllocator), *BB);
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
