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
  AU.addRequired<ResourceConfig>();
  AU.setPreservesAll();
}

bool HWAtomInfo::runOnFunction(Function &F) {
  LI = &getAnalysis<LoopInfo>();

  for (Function::iterator I = F.begin(), E = F.end(); I != E; ++I) {
    // Setup the state.
    BasicBlock &BB = *I;
    dbgs() << "Building atom for BB: " << BB.getName() << '\n';
    updateStateTo(BB);
    for (BasicBlock::iterator BI = BB.begin(), BE = BB.end(); BI != BE; ++BI) {
      visit(*BI);
    }
  }

  print(dbgs(), 0);

  return false;
}

void HWAtomInfo::clear() {
  HWAtomAllocator.Reset();
  UniqiueHWAtoms.clear();
  InstToHWAtoms.clear();
  BBToStates.clear();
  RT.clear();
}

void HWAtomInfo::releaseMemory() {
  clear();
}


//===----------------------------------------------------------------------===//
// Construct atom
void HWAtomInfo::visitTerminatorInst(TerminatorInst &I) {
  // TODO: the emitNextLoop atom

  SmallVector<HWAtom*, 2> Deps;
  addOperandDeps(I, Deps);

  // Push the control root?
  Deps.push_back(getControlRoot());
  // Get the atom
  HWAStateEnd *Atom = getStateEnd(I, Deps);
  // Remember the atom.
  InstToHWAtoms.insert(std::make_pair<const Instruction*, HWAtom*>(&I, Atom));

  // This is a control atom.
  // SetControlRoot(RetAtom);
}


void HWAtomInfo::visitPHINode(PHINode &I) {
  // Create the register
  HWARegister *Reg = getRegister(I, getStateFor(*I.getParent()));
  InstToHWAtoms.insert(std::make_pair<const Instruction*, HWAtom*>(&I, Reg));
}

void HWAtomInfo::visitSelectInst(SelectInst &I) {
  SmallVector<HWAtom*, 4> Deps;
  addOperandDeps(I, Deps);
  HWResource *Res = RT.initResource("Mux");
  assert(Res && "Can find resource!");

  HWAtom *SelAtom = getOpRes(I, Deps, *Res);

  InstToHWAtoms.insert(std::make_pair<const Instruction*, HWAtom*>(&I, SelAtom));
}

void HWAtomInfo::visitCastInst(CastInst &I) {
  HWAtom *Using;
  if (Instruction *Op = dyn_cast<Instruction>(I.getOperand(0))) {
    assert(Op && "Expect casting a Instruction");
    Using = getAtomInState(I, I.getParent());
  } else // Just use a always ready atom
    Using = getCurState();

  HWAtom *CastAtom = getWireOp(I, Using);
  InstToHWAtoms.insert(std::make_pair<const Instruction*, HWAtom*>(&I, CastAtom));
}

void HWAtomInfo::visitLoadInst(LoadInst &I) {
  SmallVector<HWAtom*, 2> Deps;
  addOperandDeps(I, Deps);

  // Push the control root base on dependence analysis
  Deps.push_back(getControlRoot());

  HWResource *Res = RT.initResource("MemoryBus");
  assert(Res && "Can find resource!");

  // Dirty Hack: allocate membus 1 to all load/store at this moment
  HWAtom *LoadAtom = getOpPreAllRes(I, Deps, *Res, 1);
  // Set as new atom
  SetControlRoot(LoadAtom);
  // And register the result
  LoadAtom =  getRegister(I, LoadAtom);

  InstToHWAtoms.insert(std::make_pair<const Instruction*, HWAtom*>(&I, LoadAtom));
}

void HWAtomInfo::visitStoreInst(StoreInst &I) {
  SmallVector<HWAtom*, 2> Deps;
  addOperandDeps(I, Deps);

  // Push the control root base on dependence analysis
  Deps.push_back(getControlRoot());

  HWResource *Res = RT.initResource("MemoryBus");
  assert(Res && "Can find resource!");

  // Dirty Hack: allocate membus 1 to all load/store at this moment
  HWAtom *StoreAtom = getOpPreAllRes(I, Deps, *Res, 1);
  // Set as new atom
  SetControlRoot(StoreAtom);

  InstToHWAtoms.insert(std::make_pair<const Instruction*, HWAtom*>(&I, StoreAtom));
}

void HWAtomInfo::visitGetElementPtrInst(GetElementPtrInst &I) {
  const Type *Ty = I.getOperand(0)->getType();
  assert(isa<SequentialType>(Ty) && "GEP type not support yet!");
  assert(I.getNumIndices() < 2 && "Too much indices in GEP!");

  SmallVector<HWAtom*, 2> Deps;
  addOperandDeps(I, Deps);

  HWResource *Res = RT.initResource("Add");
  assert(Res && "Can find resource!");

  // Create the atom
  HWAtom *GEPAtom = getOpRes(I, Deps, *Res);
  // Register the result
  GEPAtom = getRegister(I, GEPAtom);

  InstToHWAtoms.insert(std::make_pair<const Instruction*, HWAtom*>(&I, GEPAtom));
}

void HWAtomInfo::visitICmpInst(ICmpInst &I) {
  // Get the operand;
  SmallVector<HWAtom*, 2> Deps;
  if (Instruction *LHI = dyn_cast<Instruction>(I.getOperand(0))) {
    HWAtom *LHS = getAtomInState(*LHI, I.getParent());
    if (I.isSigned())
      LHS = getSigned(LHS);
    Deps.push_back(LHS);
  }

  if (Instruction *RHI = dyn_cast<Instruction>(I.getOperand(1))) {
    HWAtom *RHS = getAtomInState(*RHI, I.getParent());
    if (I.isSigned())
      RHS = getSigned(RHS);
    Deps.push_back(RHS);
  }

  HWResource *Res = RT.initResource("ICmp");
  assert(Res && "Can find resource!");
  HWAtom *CmpAtom = getOpRes(I, Deps, *Res);
  // Register it
  CmpAtom = getRegister(I, CmpAtom);

  InstToHWAtoms.insert(std::make_pair<const Instruction*, HWAtom*>(&I, CmpAtom));
}

void HWAtomInfo::visitBinaryOperator(Instruction &I) {
  HWResource *Res = 0;

  // Get the operand;
  HWAtom *LHS = 0, *RHS = 0;

  if (Instruction *LHI = dyn_cast<Instruction>(I.getOperand(0)))
    LHS = getAtomInState(*LHI, I.getParent());

  if (Instruction *RHI = dyn_cast<Instruction>(I.getOperand(1)))
    RHS = getAtomInState(*RHI, I.getParent());
  
  std::string ResName("Unknown");
  // Select the resource
  switch (I.getOpcode()) {
    case Instruction::Add:
    case Instruction::Sub:
      ResName = "Add";
      break;
    case Instruction::Mul:
      ResName ="Mul";
      break;
    case Instruction::And:
      ResName ="And";
      break;
    case Instruction::Or:
      ResName ="Or";
      break;
    case Instruction::Xor:
      ResName ="Xor";
      break;
    case Instruction::Shl:
      ResName ="Shl";
      break;
    case Instruction::LShr:
      ResName ="LShr";
      break;
    case Instruction::AShr:
      // Add the signed prefix
      if (LHS)
        LHS = getSigned(LHS);
      
      ResName ="AShr";
      break;
    default: 
      llvm_unreachable("Instruction not support yet!");
  }
  Res = RT.initResource(ResName);
  assert(Res && "Can find resource!");
  SmallVector<HWAtom*, 2> Deps;
  if (LHS)  Deps.push_back(LHS);
  if (RHS)  Deps.push_back(RHS);
  
  HWAtom *BinOpAtom = getOpRes(I, Deps, *Res);
  // Register it
  BinOpAtom = getRegister(I, BinOpAtom);

  InstToHWAtoms.insert(
    std::make_pair<const Instruction*, HWAtom*>(&I, BinOpAtom));
}

//===----------------------------------------------------------------------===//
// Create atom
HWAStateBegin *HWAtomInfo::getStateBegin(BasicBlock &BB) {
  FoldingSetNodeID ID;
  ID.AddInteger(atomStateBegin);
  ID.AddPointer(&BB);

  void *IP = 0;
  HWAStateBegin *A =
    static_cast<HWAStateBegin*>(UniqiueHWAtoms.FindNodeOrInsertPos(ID, IP));

  if (!A) {
    A = new (HWAtomAllocator) HWAStateBegin(ID.Intern(HWAtomAllocator), BB);
    // Dont Add New Atom
    // getCurState()->addNewAtom(A);
  }
  return A;
}

HWAStateEnd *HWAtomInfo::getStateEnd(TerminatorInst &Term,
                                          SmallVectorImpl<HWAtom*> &Deps) {
  FoldingSetNodeID ID;
  ID.AddInteger(atomStateEnd);
  ID.AddPointer(&Term);
  ID.AddInteger(Deps.size());
  for (unsigned i = 0, e = Deps.size(); i != e; ++i)
    ID.AddPointer(Deps[i]);

  void *IP = 0;
  HWAStateEnd *A =
    static_cast<HWAStateEnd*>(UniqiueHWAtoms.FindNodeOrInsertPos(ID, IP));

  if (!A) {
    HWAtom **O = HWAtomAllocator.Allocate<HWAtom *>(Deps.size());
    std::uninitialized_copy(Deps.begin(), Deps.end(), O);
    A = new (HWAtomAllocator) HWAStateEnd(ID.Intern(HWAtomAllocator),
      Term, O, Deps.size());
    // Add New Atom
    getCurState()->addNewAtom(A);
  }
  return A;
}

HWARegister *HWAtomInfo::getRegister(Instruction &I, HWAtom *Using) {
  FoldingSetNodeID ID;
  ID.AddInteger(atomRegister);
  ID.AddPointer(&I);
  ID.AddPointer(Using);

  void *IP = 0;
  HWARegister *A =
    static_cast<HWARegister*>(UniqiueHWAtoms.FindNodeOrInsertPos(ID, IP));

  if (!A) {
    HWAtom **O = HWAtomAllocator.Allocate<HWAtom *>(1);
    O[0] = Using;
    A = new (HWAtomAllocator) HWARegister(ID.Intern(HWAtomAllocator), I , O);
    // Add New Atom
    getCurState()->addNewAtom(A);
  }
  return A;
}

HWASigned *HWAtomInfo::getSigned(HWAtom *Using) {
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
    // Add New Atom
    getCurState()->addNewAtom(A);
  }
  return A;
}


HWAWireOp *HWAtomInfo::getWireOp(Instruction &I, HWAtom *Using) {
  FoldingSetNodeID ID;
  ID.AddInteger(atomWireOp);
  ID.AddPointer(&I);
  ID.AddPointer(Using);

  void *IP = 0;
  HWAWireOp *A =
    static_cast<HWAWireOp*>(UniqiueHWAtoms.FindNodeOrInsertPos(ID, IP));

  if (!A) {
    HWAtom **O = HWAtomAllocator.Allocate<HWAtom *>(1);
    O[0] = Using;
    A = new (HWAtomAllocator) HWAWireOp(ID.Intern(HWAtomAllocator), I , O);
    // Add New Atom
    getCurState()->addNewAtom(A);
  }
  return A;
}

HWAOpPostAllRes *HWAtomInfo::getOpPostAllRes(Instruction &I,
                                             SmallVectorImpl<HWAtom*> &Deps,
                                             HWResource &Res) {
  FoldingSetNodeID ID;
  ID.AddInteger(atomOpPostAllRes);
  ID.AddPointer(&I);
  ID.AddInteger(Deps.size());
  ID.AddPointer(&Res);
  for (unsigned i = 0, e = Deps.size(); i != e; ++i)
    ID.AddPointer(Deps[i]);

  void *IP = 0;
  HWAOpPostAllRes *A =
    static_cast<HWAOpPostAllRes*>(UniqiueHWAtoms.FindNodeOrInsertPos(ID, IP));

  if (!A) {
    HWAtom **O = HWAtomAllocator.Allocate<HWAtom *>(Deps.size());
    std::uninitialized_copy(Deps.begin(), Deps.end(), O);
    A = new (HWAtomAllocator) HWAOpPostAllRes(ID.Intern(HWAtomAllocator),
      I, O, Deps.size(), Res);
    // Add New Atom
    getCurState()->addNewAtom(A);
  }
  return A;
}


HWAOpPreAllRes *HWAtomInfo::getOpPreAllRes(Instruction &I,
                                           SmallVectorImpl<HWAtom*> &Deps,
                                           HWResource &Res, unsigned ResInst) {
  FoldingSetNodeID ID;
  ID.AddInteger(atomOpPreAllRes);
  ID.AddPointer(&I);
  ID.AddInteger(Deps.size());
  for (unsigned i = 0, e = Deps.size(); i != e; ++i)
    ID.AddPointer(Deps[i]);

  void *IP = 0;
  HWAOpPreAllRes *A =
    static_cast<HWAOpPreAllRes*>(UniqiueHWAtoms.FindNodeOrInsertPos(ID, IP));

  if (!A) {
    HWAtom **O = HWAtomAllocator.Allocate<HWAtom *>(Deps.size());
    std::uninitialized_copy(Deps.begin(), Deps.end(), O);
    A = new (HWAtomAllocator) HWAOpPreAllRes(ID.Intern(HWAtomAllocator),
      I, O, Deps.size(), Res, ResInst);
    // Add New Atom
    getCurState()->addNewAtom(A);
  }
  return A;
}

void HWAtomInfo::print(raw_ostream &O, const Module *M) const {
  for (StateMapType::const_iterator I = BBToStates.begin(), E = BBToStates.end();
      I != E; ++I) {
    I->second->print(O);
  }
}

HWAtomInfo::HWAtomInfo()
: FunctionPass(&ID), ControlRoot(0), CurState(0),
LI(0), RT(*(new ResourceConfig()))  {
  llvm_unreachable("We can not create HWSAtomInfo like this!");
}
