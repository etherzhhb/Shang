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
#include "llvm/Analysis/LiveValues.h"

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
  AU.addRequired<LiveValues>();
  AU.addRequired<LoopInfo>();
  AU.addRequired<ResourceConfig>();
  AU.addRequired<MemDepInfo>();
  AU.setPreservesAll();
}

bool HWAtomInfo::runOnFunction(Function &F) {
  LV = &getAnalysis<LiveValues>();
  LI = &getAnalysis<LoopInfo>();
  RC = &getAnalysis<ResourceConfig>();
  MDA = &getAnalysis<MemDepInfo>();
  TD = getAnalysisIfAvailable<TargetData>();
  assert(TD && "Can not work without TD!");

  std::vector<HWAOpFU*> MemOps;

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
      // Add the Atom to state.
      // FIXME: Some atom will add them to parent atoms vector automatically,
      // but this is not a good idea.
      if (!A->getParent())
        State->addAtom(A);

      // Remember the atom.
      ValueToHWAtoms.insert(std::make_pair(&Inst, A));
      // Remember the MemOps, we will compute the dependencies about them later.
      if (isa<LoadInst>(Inst) || isa<StoreInst>(Inst))
        MemOps.push_back(cast<HWAOpFU>(A));
    }
    // preform memory dependencies analysis to add corresponding edges.
    addMemDepEdges(MemOps, *BB);

    bool selfLoop = haveSelfLoop(BB);
    

    HWAOpFU *Exit = cast<HWAOpFU>(getControlRoot());
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

HWADelay *HWAtomInfo::addLoopPredBackEdge(BasicBlock *BB) {
  assert(haveSelfLoop(BB) && "Loop SCC only exist in self loop!");
  
  FSMState *State = getStateFor(*BB);
  // Get the induction variable increment.
  //Instruction *IVInc = cast<Instruction>(IV->getIncomingValueForBlock(BB));
  //HWAOpInst *IVIncAtom = cast<HWAOpInst>(getAtomFor(*IVInc));
  // And get the predicate
  BranchInst *Br = cast<BranchInst>(BB->getTerminator());
  ICmpInst *ICmp = cast<ICmpInst>(Br->getCondition());
  HWAOpFU *Pred = cast<HWAOpFU>(getAtomFor(*ICmp));

  // The Next loop depend on the result of predicate.
  // Dirty Hack: The FSM have a delay of 1.
  HWADelay *Delay = getDelay(Pred, 1);
  HWMemDep *LoopDep = getMemDepEdge(Delay, true, HWMemDep::TrueDep, 1);
  //IVIncAtom->addDep(LoopDep);
  State->addDep(LoopDep);

  return Delay;
}

void HWAtomInfo::addMemDepEdges(std::vector<HWAOpFU*> &MemOps, BasicBlock &BB) {
  typedef std::vector<HWAOpFU*> OpInstVec;
  typedef MemDepInfo::DepInfo DepInfoType;
  for (OpInstVec::iterator SrcI = MemOps.begin(), SrcE = MemOps.end();
      SrcI != SrcE; ++SrcI) {
    HWAOpFU *Src = *SrcI;
    bool isSrcLoad = isa<LoadInst>(Src->getValue());

    for (OpInstVec::iterator DstI = MemOps.begin(), DstE = MemOps.end();
        DstI != DstE; ++DstI) {
      HWAOpFU *Dst = *DstI;
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
  // We may need to wait until the operation for return value finish.
  // FIXME: If we make return port a wire, then we do not need to delay
  // the operation.
  //if (!Deps.empty() && isa<ReturnInst>(I))
  //  if (HWAOpFU *OI = dyn_cast<HWAOpFU>(Deps[0]->getSrc())) {
  //    Deps[0]->setSrc(getDelay(OI, 1));
  //  }

  unsigned OpSize = Deps.size();

  std::set<HWEdge*> ExportEdges;
  // All node should finish before terminator run.
  FSMState *State = getStateFor(*I.getParent());
  BasicBlock *BB = I.getParent();

  // And export the live values.
  for (BasicBlock::iterator II = BB->begin(), IE = --BB->end(); II != IE; ++II) {
    Instruction &Inst = *II;
    HWAtom *A = getAtomFor(Inst);
    
    if (!LV->isKilledInBlock(&Inst, BB)) {
      Deps.push_back(getValDepEdge(A, false, HWValDep::Export));
      continue;
    }

    Value *V = &A->getValue();
    const Type *Ty =A->getValue().getType();
    if (Ty->isVoidTy() || A->use_empty()) {
      Deps.push_back(getCtrlDepEdge(A));
    } 
  }

  // Create delay atom for phi node.
  addPhiExportEdges(*BB, Deps);
  // handle the situation that a BB that only contains a "ret void".
  if (State->getNumUses() == 0)
    Deps.push_back(getCtrlDepEdge(State));

  // Emit all atom before exit.
  if (haveSelfLoop(BB))
    Deps.push_back(getCtrlDepEdge(addLoopPredBackEdge(BB)));

  assert(!Deps.empty() && "exit root not connect to anything.");
  HWFUnit *FU = RC->allocaTrivialFU(0, 0);
  HWAOpFU *Atom = getOpFU(I, Deps, OpSize, FU);
  // This is a control atom.
  SetControlRoot(Atom);

  return Atom;
}


HWAtom *HWAtomInfo::visitPHINode(PHINode &I) {
  return getLIReg(getStateFor(*I.getParent()), I);
}

HWAtom *HWAtomInfo::visitSelectInst(SelectInst &I) {
  SmallVector<HWEdge*, 4> Deps;
  addOperandDeps(I, Deps);

  // FIXME: Read latency from configure file
  HWFUnit *FU = RC->allocaTrivialFU(1, TD->getTypeSizeInBits(I.getType()));
  return getOpFU(I, Deps, FU);
}

HWAtom *HWAtomInfo::visitCastInst(CastInst &I) {
  SmallVector<HWEdge*, 1> Deps;
  Deps.push_back(getValDepInState(*I.getOperand(0), I.getParent()));
  // CastInst do not have any latency
  HWFUnit *FU = RC->allocaTrivialFU(0, TD->getTypeSizeInBits(I.getType()));
  return getOpFU(I, Deps, FU);
}

HWAtom *HWAtomInfo::visitLoadInst(LoadInst &I) {
  SmallVector<HWEdge*, 2> Deps;
  addOperandDeps(I, Deps);

  // Dirty Hack: allocate membus 1 to all load/store at this moment
  HWAtom *LoadAtom = getOpFU(I, Deps, RC->allocaMemBusFU(1));

  return LoadAtom;
}

HWAtom *HWAtomInfo::visitStoreInst(StoreInst &I) {
  SmallVector<HWEdge*, 2> Deps;
  addOperandDeps(I, Deps);

  // Dirty Hack: allocate membus 0 to all load/store at this moment
  HWAtom *StoreAtom = getOpFU(I, Deps, RC->allocaMemBusFU(1));
  // Set as new atom
  SetControlRoot(StoreAtom);

  return StoreAtom;
}

HWAtom *HWAtomInfo::visitGetElementPtrInst(GetElementPtrInst &I) {
  const Type *Ty = I.getOperand(0)->getType();
  assert(isa<SequentialType>(Ty) && "GEP type not support yet!");
  if (I.getNumIndices() > 1) {
    assert(I.hasAllZeroIndices() && "Too much indices in GEP!");
    SmallVector<HWEdge*, 8> Deps;
    addOperandDeps(I, Deps);

    HWFUnit *FU = RC->allocaTrivialFU(0, TD->getTypeSizeInBits(I.getType()));
    return getOpFU(I, Deps, FU);
  }

  SmallVector<HWEdge*, 2> Deps;
  addOperandDeps(I, Deps);
  return getOpFU(I, Deps, RC->allocaBinOpFU(HWResType::AddSub,
                                                TD->getPointerSizeInBits()));
}

HWAtom *HWAtomInfo::visitICmpInst(ICmpInst &I) {
  // Get the operand;
  SmallVector<HWEdge*, 2> Deps;
  // LHS
  Deps.push_back(getValDepInState(*I.getOperand(0), I.getParent(), I.isSigned()));
  // RHS
  Deps.push_back(getValDepInState(*I.getOperand(1), I.getParent(), I.isSigned()));

  // It is trivial if one of the operand is constant
  if (isa<Constant>(I.getOperand(0)) || isa<Constant>(I.getOperand(1))) {
    HWFUnit *FU = RC->allocaTrivialFU(1, TD->getTypeSizeInBits(I.getType()));
    return getOpFU(I, Deps, FU);
  } else {// We need to do a subtraction for the comparison.
    HWFUnit *FU = RC->allocaTrivialFU(1, TD->getTypeSizeInBits(I.getType()));
    return getOpFU(I, Deps, FU);
  }
}

HWAtom *HWAtomInfo::visitBinaryOperator(Instruction &I) {
  // Get the operand;
  SmallVector<HWEdge*, 2> Deps;
  bool isSigned = false;
  bool isOp1Const = isa<Constant>(I.getOperand(1));
  unsigned BitWidth = TD->getTypeSizeInBits(I.getType());
  HWFUnit *FU;
  // Select the resource
  switch (I.getOpcode()) {
    case Instruction::Add:
    case Instruction::Sub:
      //T = isTrivial ? HWResource::Trivial : HWResource::AddSub;
      FU = RC->allocaBinOpFU(HWResType::AddSub, BitWidth);
      break;
    case Instruction::Mul:
      FU = RC->allocaBinOpFU(HWResType::Mult, BitWidth);
      break;
    case Instruction::And:
    case Instruction::Or:
    case Instruction::Xor:
      // FIXME: Enable chaining.
      FU = RC->allocaTrivialFU(1, BitWidth);
      break;
    case Instruction::AShr:
      // Add the signed prefix for lhs
      isSigned = true;
      FU = isOp1Const ? RC->allocaTrivialFU(1, BitWidth)
                      : RC->allocaBinOpFU(HWResType::ASR, BitWidth);
      break;
    case Instruction::LShr:
      FU = isOp1Const ? RC->allocaTrivialFU(1, BitWidth)
                      : RC->allocaBinOpFU(HWResType::LSR, BitWidth);
      break;
    case Instruction::Shl:
      FU = isOp1Const ? RC->allocaTrivialFU(1, BitWidth)
                      : RC->allocaBinOpFU(HWResType::SHL, BitWidth);
      break;
    default: 
      llvm_unreachable("Instruction not support yet!");
  }
  // LHS
  Deps.push_back(getValDepInState(*I.getOperand(0), I.getParent(), isSigned));
  // RHS
  Deps.push_back(getValDepInState(*I.getOperand(1), I.getParent()));
  
  return getOpFU(I, Deps, FU);
}

//===----------------------------------------------------------------------===//
// Create atom

HWAOpFU *HWAtomInfo::bindToFU(HWAOpFU *PostBind, unsigned FUID)
{
  assert(FUID && "Instance can not be 0 !");
  PostBind->reAssignFUnit(RC->assignIDToFU(PostBind->getFUnit(), FUID));

  return PostBind;
}

HWAOpFU *HWAtomInfo::getOpFU(Instruction &I, SmallVectorImpl<HWEdge*> &Deps,
                             size_t OpNum, HWFUnit *FU) {
  FoldingSetNodeID FUID;
  FUID.AddInteger(atomOpFU);
  FUID.AddPointer(&I);

  void *IP = 0;
  HWAOpFU *A = static_cast<HWAOpFU*>(UniqiueHWAtoms.FindNodeOrInsertPos(FUID, IP));

  if (!A) {
    A = new (HWAtomAllocator) HWAOpFU(FUID.Intern(HWAtomAllocator), I, FU,
                                      Deps.begin(), Deps.end(), OpNum, ++InstIdx);
    UniqiueHWAtoms.InsertNode(A, IP);
  }
  return A;
}

FSMState *HWAtomInfo::getState(BasicBlock *BB) {
  assert(BB && "BB can not be null!");
  FoldingSetNodeID FUID;
  FUID.AddInteger(atomVRoot);
  FUID.AddPointer(BB);

  void *IP = 0;
  FSMState *A =
    static_cast<FSMState*>(UniqiueHWAtoms.FindNodeOrInsertPos(FUID, IP));

  if (!A) {
    A = new (HWAtomAllocator) FSMState(FUID.Intern(HWAtomAllocator), *BB,
                                       ++InstIdx);
    UniqiueHWAtoms.InsertNode(A, IP);
  }
  return A;
}

HWAWrReg *HWAtomInfo::getWrReg(HWAtom *Src) {
  FoldingSetNodeID FUID;
  FUID.AddInteger(atomWrReg);
  FUID.AddPointer(Src);
  HWRegister *R = getRegForValue(&Src->getValue());
  FUID.AddPointer(R);

  void *IP = 0;
  HWAWrReg *A =
    static_cast<HWAWrReg*>(UniqiueHWAtoms.FindNodeOrInsertPos(FUID, IP));

  if (!A) {
    A = new (HWAtomAllocator) HWAWrReg(FUID.Intern(HWAtomAllocator),
                                       *getValDepEdge(Src, false), R,
                                       Src->getFinSlot(), ++InstIdx);
    UniqiueHWAtoms.InsertNode(A, IP);
  }
  return A;
}

HWAWrReg *HWAtomInfo::getWrReg(HWAtom *Src, HWRegister *Reg,
                               unsigned short Slot) {
  FoldingSetNodeID FUID;
  FUID.AddInteger(atomWrReg);
  FUID.AddPointer(Src);
  FUID.AddPointer(Reg);

  void *IP = 0;
  HWAWrReg *A =
    static_cast<HWAWrReg*>(UniqiueHWAtoms.FindNodeOrInsertPos(FUID, IP));

  if (!A) {
    A = new (HWAtomAllocator) HWAWrReg(FUID.Intern(HWAtomAllocator),
                                       *getValDepEdge(Src, false), Reg,
                                       Slot, ++InstIdx);
    UniqiueHWAtoms.InsertNode(A, IP);
  }
  return A;
}

HWAWrReg *HWAtomInfo::getWrReg(HWEdge *SrcEdge, Value *V) {
  FoldingSetNodeID FUID;
  FUID.AddInteger(atomWrReg);
  FUID.AddPointer(SrcEdge->getSrc());
  HWRegister *R = getRegForValue(V);
  FUID.AddPointer(R);

  void *IP = 0;
  HWAWrReg *A =
    static_cast<HWAWrReg*>(UniqiueHWAtoms.FindNodeOrInsertPos(FUID, IP));

  if (!A) {
    A = new (HWAtomAllocator) HWAWrReg(FUID.Intern(HWAtomAllocator),
                                       *SrcEdge, R, 0, ++InstIdx);
    UniqiueHWAtoms.InsertNode(A, IP);
  }
  return A;
}

HWADelay *HWAtomInfo::getDelay(HWAtom *Src, unsigned Delay) {
  FoldingSetNodeID FUID;
  FUID.AddInteger(atomDelay);
  FUID.AddPointer(Src);
  FUID.AddInteger(Delay);

  void *IP = 0;
  HWADelay *A =
    static_cast<HWADelay*>(UniqiueHWAtoms.FindNodeOrInsertPos(FUID, IP));

  if (!A) {
    A = new (HWAtomAllocator) HWADelay(FUID.Intern(HWAtomAllocator),
                                       *getCtrlDepEdge(Src), Delay, ++InstIdx);
    UniqiueHWAtoms.InsertNode(A, IP);
  }
  return A;
}


HWALIReg *HWAtomInfo::getLIReg(HWAtom *Src, Value &V) {
  FoldingSetNodeID FUID;
  FUID.AddInteger(atomLIReg);
  FUID.AddPointer(Src);
  FUID.AddPointer(&V);

  void *IP = 0;
  HWALIReg *A =
    static_cast<HWALIReg*>(UniqiueHWAtoms.FindNodeOrInsertPos(FUID, IP));

  if (!A) {
    A = new (HWAtomAllocator) HWALIReg(FUID.Intern(HWAtomAllocator), V,
                                       getValDepEdge(Src, false, HWValDep::Import), 
                                       TD->getTypeSizeInBits(V.getType()),
                                       ++InstIdx);
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

void HWAtomInfo::addPhiExportEdges(BasicBlock &BB, SmallVectorImpl<HWEdge*> &Deps) {
  FSMState *State = getStateFor(BB);
  for (succ_iterator SI = succ_begin(&BB), SE = succ_end(&BB); SI != SE; ++SI){
      BasicBlock *SuccBB = *SI;
    for (BasicBlock::iterator II = SuccBB->begin(),
        IE = SuccBB->getFirstNonPHI(); II != IE; ++II) {
      PHINode *PN = cast<PHINode>(II);
      Value *IV = PN->getIncomingValueForBlock(&BB);
      //// No instruction value do not need to Export.
      //if (!Inst)
      //  continue;
      //// We do not need to Export if the value not define in the current BB.
      //if (Inst->getParent() != &BB)
      //  continue;
      //// We do not need to delay if source is PHI, because it is ready before
      //// entering this FSMState.
      //if (isa<PHINode>(Inst))
      //  continue;
      HWEdge *PHIEdge = getValDepInState(*IV, &BB);
      HWAWrReg *WR = getWrReg(PHIEdge, PN);
      PHIEdge = getValDepEdge(WR, false, HWValDep::PHI);
      Deps.push_back(PHIEdge);
      //// Create the PHI edge.
      //HWAOpFU *OpInst = cast<HWAOpFU>(getAtomFor(*Inst));
      //// Delay one cycle to wait the value finish.
      //HWADelay *Delay = getDelay(OpInst, 1);
      //HWValDep *PHIEdge = getValDepEdge(Delay, false, HWValDep::PHI);
      //Deps.push_back(PHIEdge);
      //// Remember this edge and its dest PHINode.
      //State->addPHIEdge(PN, PHIEdge);

      if (&BB == SuccBB) {// Self Loop?
        // The Next loop depend on the result of phi.
        HWMemDep *PHIDep = getMemDepEdge(WR, true, HWMemDep::TrueDep, 1);
        // Create the cycle for PHINode.
        getAtomFor(*PN)->addDep(PHIDep);
      }
    }
  }
}

HWEdge *HWAtomInfo::getValDepInState(Value &V, BasicBlock *BB, bool isSigned) {
  // Is this not a instruction?
  if (isa<Argument>(V))
    return getValDepEdge(getLIReg(getStateFor(*BB), V), isSigned);
  else if (Constant *C = dyn_cast<Constant>(&V))
    return getConstEdge(getStateFor(*BB), C);

  // Now it is an Instruction
  Instruction &Inst = cast<Instruction>(V);
  if (BB == Inst.getParent())
    // Create a wire dep for atoms in the same state.
    return getValDepEdge(getAtomFor(Inst), isSigned);
  else
    // Otherwise this edge is an import edge.
    return getValDepEdge(getLIReg(getStateFor(*BB), Inst), isSigned);
}

void HWAtomInfo::addOperandDeps(Instruction &I, SmallVectorImpl<HWEdge*> &Deps) {
  BasicBlock *ParentBB = I.getParent();
  for (ReturnInst::op_iterator OI = I.op_begin(), OE = I.op_end();
      OI != OE; ++OI)
    if(!isa<BasicBlock>(OI)) // Ignore the basic Block.
      Deps.push_back(getValDepInState(**OI, ParentBB));
}
