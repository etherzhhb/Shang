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

#include "esly/HWAtomInfo.h"
//#include "MemDepAnalysis.h"
#include "esly/HWAtomPasses.h"

#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/MachineLoopInfo.h"
#include "llvm/CodeGen/LiveVariables.h"
#include "llvm/Support/Allocator.h"
#include "llvm/Support/ErrorHandling.h"

#define DEBUG_TYPE "vbe-hw-atom-info"
#include "llvm/Support/Debug.h"

using namespace llvm;

//===----------------------------------------------------------------------===//

char HWAtomInfo::ID = 0;
static RegisterPass<HWAtomInfo> X("vbe-hw-atom-info",
                                  "vbe - Construct the Hardware atom respresent"
                                  " on llvm IR");

Pass *esyn::createHWAtonInfoPass() {
  return new HWAtomInfo();
}

void HWAtomInfo::getAnalysisUsage(AnalysisUsage &AU) const {
  MachineFunctionPass::getAnalysisUsage(AU);
  AU.addRequired<LiveVariables>();
  AU.addRequired<MachineLoopInfo>();
  // AU.addRequired<MemDepInfo>();
  AU.setPreservesAll();
}

bool HWAtomInfo::runOnMachineFunction(MachineFunction &MF) {
  LiveVars = &getAnalysis<LiveVariables>();
  VTarget = &(MF.getTarget().getSubtarget<VSubtarget>());
  MRI = &MF.getRegInfo();

  for (MachineFunction::iterator I = MF.begin(), E = MF.end();
       I != E; ++I) {
    const MachineBasicBlock &MBB = *I;

    FSMState *State = createState(&MBB);
    MachBBToStates.insert(std::make_pair(&MBB, State));

    for (MachineBasicBlock::const_iterator BI = MBB.begin(), BE = MBB.end();
        BI != BE; ++BI) {
      const MachineInstr &MInst = *BI;
      if (HWAtom *A = buildAtom(&MInst)) {
        State->addAtom(A);
        InstToHWAtoms.insert(std::make_pair(&MInst, A));
      }
    }

    State->viewGraph();
  }


  //LI = &getAnalysis<LoopInfo>();
  //RC = &getAnalysis<ResourceConfig>();
  //// MDA = &getAnalysis<MemDepInfo>();
  //TD = getAnalysisIfAvailable<TargetData>();
  //assert(TD && "Can not work without TD!");

  //std::vector<HWAOpFU*> MemOps;

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
  //      MemOps.push_back(cast<HWAOpFU>(A));
  //  }
  //  // preform memory dependencies analysis to add corresponding edges.
  //  addMemDepEdges(MemOps, *BB);

  //  bool selfLoop = haveSelfLoop(BB);
  //  

  //  HWAOpFU *Exit = cast<HWAOpFU>(getControlRoot());
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

HWADelay *HWAtomInfo::addLoopPredBackEdge(const MachineBasicBlock *MBB) {
  assert(haveSelfLoop(MBB) && "Loop SCC only exist in self loop!");
  
  FSMState *State = getStateFor(MBB);
  // And get the predicate
  //BranchInst *Br = cast<BranchInst>(MBB->getTerminator());
  //ICmpInst *ICmp = cast<ICmpInst>(Br->getCondition());
  //HWAOpFU *Pred = cast<HWAOpFU>(getAtomFor(*ICmp));

  //// The Next loop depend on the result of predicate.
  //// Dirty Hack: The FSM have a delay of 1.
  //HWADelay *Delay = getDelay(Pred, 1);
  //HWMemDep *LoopDep = getMemDepEdge(Delay, true, HWMemDep::TrueDep, 1);
  //State->addDep(LoopDep);

  // return Delay;
  return 0;
}

void HWAtomInfo::addMemDepEdges(std::vector<HWAOpFU*> &MemOps, BasicBlock &BB) {
  //typedef std::vector<HWAOpFU*> OpInstVec;
  //typedef MemDepInfo::DepInfo DepInfoType;
  //for (OpInstVec::iterator SrcI = MemOps.begin(), SrcE = MemOps.end();
  //    SrcI != SrcE; ++SrcI) {
  //  HWAOpFU *Src = *SrcI;
  //  bool isSrcLoad = isa<LoadInst>(Src->getValue());

  //  for (OpInstVec::iterator DstI = MemOps.begin(), DstE = MemOps.end();
  //      DstI != DstE; ++DstI) {
  //    HWAOpFU *Dst = *DstI;
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
// Construct atom from LLVM-IR
// What if the one of the operand is Constant?
//HWAtom *HWAtomInfo::visitTerminatorInst(TerminatorInst &I) {
//  // State end depand on or others atoms
//  SmallVector<HWEdge*, 16> Deps;
//  addOperandDeps(I, Deps);
//  // We may need to wait until the operation for return value finish.
//  // FIXME: If we make return port a wire, then we do not need to delay
//  // the operation.
//  //if (!Deps.empty() && isa<ReturnInst>(I))
//  //  if (HWAOpFU *OI = dyn_cast<HWAOpFU>(Deps[0]->getSrc())) {
//  //    Deps[0]->setSrc(getDelay(OI, 1));
//  //  }
//
//  unsigned OpSize = Deps.size();
//
//  std::set<HWEdge*> ExportEdges;
//  // All node should finish before terminator run.
//  FSMState *State = getStateFor(*I.getParent());
//  BasicBlock *BB = I.getParent();
//
//  // And export the live values.
//  for (BasicBlock::iterator II = BB->begin(), IE = --BB->end(); II != IE; ++II) {
//    Instruction &Inst = *II;
//    HWAtom *A = getAtomFor(Inst);
//    
//    if (usedOutSideBB(Inst, BB)) {
//      Deps.push_back(getValDepEdge(A, false, HWValDep::Export));
//      continue;
//    }
//
//    const Type *Ty =A->getValue().getType();
//    if (Ty->isVoidTy() || A->use_empty()) {
//      Deps.push_back(getCtrlDepEdge(A));
//    } 
//  }
//
//  // Also Export live in value, to simplify the Local LEA based register
//  // merging alogrithm.
//  for (FSMState::iterator AI = State->begin(), AE = State->end();
//       AI != AE; ++AI)
//    if (HWALIReg *LI = dyn_cast<HWALIReg>(*AI))
//      if (usedOutSideBB(LI->getValue(), BB))
//        Deps.push_back(getCtrlDepEdge(LI));
//
//  // Create delay atom for phi node.
//  addPhiExportEdges(*BB, Deps);
//  // handle the situation that a BB that only contains a "ret void".
//  if (State->getNumUses() == 0)
//    Deps.push_back(getCtrlDepEdge(State));
//
//  // Emit all atom before exit.
//  if (haveSelfLoop(BB))
//    Deps.push_back(getCtrlDepEdge(addLoopPredBackEdge(BB)));
//
//  assert(!Deps.empty() && "exit root not connect to anything.");
//  HWFUnit *FU = RC->allocaTrivialFU(0, 0);
//  HWAOpFU *Atom = getOpFU(I, Deps, OpSize, FU);
//  // This is a control atom.
//  SetControlRoot(Atom);
//
//  return Atom;
//}
//
//
//HWAtom *HWAtomInfo::visitPHINode(PHINode &I) {
//  return getLIReg(getStateFor(*I.getParent()), I);
//}
//
//HWAtom *HWAtomInfo::visitSelectInst(SelectInst &I) {
//  SmallVector<HWEdge*, 4> Deps;
//  addOperandDeps(I, Deps);
//
//  // FIXME: Read latency from configure file
//  HWFUnit *FU = RC->allocaTrivialFU(1, TD->getTypeSizeInBits(I.getType()));
//  return getOpFU(I, Deps, FU);
//}
//
//HWAtom *HWAtomInfo::visitCastInst(CastInst &I) {
//  SmallVector<HWEdge*, 1> Deps;
//  Deps.push_back(getValDepInState(*I.getOperand(0), I.getParent()));
//  // CastInst do not have any latency
//  HWFUnit *FU = RC->allocaTrivialFU(0, TD->getTypeSizeInBits(I.getType()));
//  return getOpFU(I, Deps, FU);
//}
//
//HWAtom *HWAtomInfo::visitLoadInst(LoadInst &I) {
//  SmallVector<HWEdge*, 2> Deps;
//  addOperandDeps(I, Deps);
//
//  // Dirty Hack: allocate membus 1 to all load/store at this moment
//  HWAtom *LoadAtom = getOpFU(I, Deps, RC->allocaMemBusFU(1));
//
//  return LoadAtom;
//}
//
//HWAtom *HWAtomInfo::visitStoreInst(StoreInst &I) {
//  SmallVector<HWEdge*, 2> Deps;
//  addOperandDeps(I, Deps);
//
//  // Dirty Hack: allocate membus 0 to all load/store at this moment
//  HWAtom *StoreAtom = getOpFU(I, Deps, RC->allocaMemBusFU(1));
//  // Set as new atom
//  SetControlRoot(StoreAtom);
//
//  return StoreAtom;
//}
//
//HWAtom *HWAtomInfo::visitGetElementPtrInst(GetElementPtrInst &I) {
//  const Type *Ty = I.getOperand(0)->getType();
//  assert(isa<SequentialType>(Ty) && "GEP type not support yet!");
//  if (I.getNumIndices() > 1) {
//    assert(I.hasAllZeroIndices() && "Too much indices in GEP!");
//    SmallVector<HWEdge*, 8> Deps;
//    addOperandDeps(I, Deps);
//
//    HWFUnit *FU = RC->allocaTrivialFU(0, TD->getTypeSizeInBits(I.getType()));
//    return getOpFU(I, Deps, FU);
//  }
//
//  SmallVector<HWEdge*, 2> Deps;
//  addOperandDeps(I, Deps);
//  return getOpFU(I, Deps, RC->allocaBinOpFU(HWResType::AddSub,
//                                                TD->getPointerSizeInBits()));
//}
//
//HWAtom *HWAtomInfo::visitICmpInst(ICmpInst &I) {
//  // Get the operand;
//  SmallVector<HWEdge*, 2> Deps;
//  // LHS
//  Deps.push_back(getValDepInState(*I.getOperand(0), I.getParent(), I.isSigned()));
//  // RHS
//  Deps.push_back(getValDepInState(*I.getOperand(1), I.getParent(), I.isSigned()));
//
//  // It is trivial if one of the operand is constant
//  if (isa<Constant>(I.getOperand(0)) || isa<Constant>(I.getOperand(1))) {
//    HWFUnit *FU = RC->allocaTrivialFU(1, TD->getTypeSizeInBits(I.getType()));
//    return getOpFU(I, Deps, FU);
//  } else {// We need to do a subtraction for the comparison.
//    HWFUnit *FU = RC->allocaTrivialFU(1, TD->getTypeSizeInBits(I.getType()));
//    return getOpFU(I, Deps, FU);
//  }
//}
//
//HWAtom *HWAtomInfo::visitBinaryOperator(Instruction &I) {
//  // Get the operand;
//  SmallVector<HWEdge*, 2> Deps;
//  bool isSigned = false;
//  bool isOp1Const = isa<Constant>(I.getOperand(1));
//  unsigned BitWidth = TD->getTypeSizeInBits(I.getType());
//  HWFUnit *FU;
//  // Select the resource
//  switch (I.getOpcode()) {
//    case Instruction::Add:
//    case Instruction::Sub:
//      //T = isTrivial ? HWResource::Trivial : HWResource::AddSub;
//      FU = RC->allocaBinOpFU(HWResType::AddSub, BitWidth);
//      break;
//    case Instruction::Mul:
//      FU = RC->allocaBinOpFU(HWResType::Mult, BitWidth);
//      break;
//    case Instruction::And:
//    case Instruction::Or:
//    case Instruction::Xor:
//      // FIXME: Enable chaining.
//      FU = RC->allocaTrivialFU(1, BitWidth);
//      break;
//    case Instruction::AShr:
//      // Add the signed prefix for lhs
//      isSigned = true;
//      FU = isOp1Const ? RC->allocaTrivialFU(1, BitWidth)
//                      : RC->allocaBinOpFU(HWResType::ASR, BitWidth);
//      break;
//    case Instruction::LShr:
//      FU = isOp1Const ? RC->allocaTrivialFU(1, BitWidth)
//                      : RC->allocaBinOpFU(HWResType::LSR, BitWidth);
//      break;
//    case Instruction::Shl:
//      FU = isOp1Const ? RC->allocaTrivialFU(1, BitWidth)
//                      : RC->allocaBinOpFU(HWResType::SHL, BitWidth);
//      break;
//    default: 
//      llvm_unreachable("Instruction not support yet!");
//  }
//  // LHS
//  Deps.push_back(getValDepInState(*I.getOperand(0), I.getParent(), isSigned));
//  // RHS
//  Deps.push_back(getValDepInState(*I.getOperand(1), I.getParent()));
//  
//  return getOpFU(I, Deps, FU);
//}

//===----------------------------------------------------------------------===//
// Create atom

FSMState *HWAtomInfo::createState(const MachineBasicBlock *MBB) {
  assert(!MachBBToStates.count(MBB) && "MI exist!");

  FSMState *A =  new (HWAtomAllocator) FSMState(MBB, ++InstIdx);
  return A;
}

HWADelay *HWAtomInfo::getDelay(HWAtom *Src, unsigned Delay) {
  //FoldingSetNodeID FUID;
  //FUID.AddInteger(atomDelay);
  //FUID.AddPointer(Src);
  //FUID.AddInteger(Delay);

  //void *IP = 0;
  //HWADelay *A =
  //  static_cast<HWADelay*>(UniqiueHWAtoms.FindNodeOrInsertPos(FUID, IP));

  //if (!A) {
  //  A = new (HWAtomAllocator) HWADelay(FUID.Intern(HWAtomAllocator),
  //                                     *getCtrlDepEdge(Src), Delay, ++InstIdx);
  //  UniqiueHWAtoms.InsertNode(A, IP);
  //}
  return 0;
}


HWMemDep *HWAtomInfo::getMemDepEdge(HWAtom *Src, bool isBackEdge,
                                    enum HWMemDep::MemDepTypes DepType,
                                    unsigned Diff) {
  return new (HWAtomAllocator) HWMemDep(Src, isBackEdge, DepType, Diff);
}

void HWAtomInfo::print(raw_ostream &O, const Module *M) const {}

void HWAtomInfo::addPhiExportEdges(BasicBlock &BB, SmallVectorImpl<HWEdge*> &Deps) {
  //for (succ_iterator SI = succ_begin(&BB), SE = succ_end(&BB); SI != SE; ++SI){
  //    BasicBlock *SuccBB = *SI;
  //  for (BasicBlock::iterator II = SuccBB->begin(),
  //      IE = SuccBB->getFirstNonPHI(); II != IE; ++II) {
  //    PHINode *PN = cast<PHINode>(II);
  //    Value *IV = PN->getIncomingValueForBlock(&BB);

  //    //  continue;
  //    HWEdge *PHIEdge = getValDepInState(*IV, &BB);
  //    HWAWrReg *WR = getWrReg(PHIEdge, PN);
  //    // The source of PHI nodes suppose live untill the state finished. 
  //    PHIEdge = getValDepEdge(WR, false, HWValDep::PHI);
  //    Deps.push_back(PHIEdge);

  //    if (&BB == SuccBB) {// Self Loop?
  //      // The Next loop depend on the result of phi.
  //      HWMemDep *PHIDep = getMemDepEdge(WR, true, HWMemDep::TrueDep, 1);
  //      // Create the cycle for PHINode.
  //      getAtomFor(*PN)->addDep(PHIDep);
  //    }
  //  }
  //}
}

void HWAtomInfo::addOperandDeps(const MachineInstr *MI,
                                SmallVectorImpl<HWEdge*> &Deps) {
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

    // We are building a dependence graph of a MBB only.
    if (MBB->isLiveIn(Reg)) continue;

    if (HWAtom *Dep = getAtomFor(MRI->getVRegDef(Reg)))
      Deps.push_back(getValDepEdge(Dep));
  }

  // If the atom depend on nothing, make it depend on the entry node.
  if (Deps.empty())
    Deps.push_back(getValDepEdge(getStateFor(MBB)));
}

HWAtom *HWAtomInfo::buildAtom(const MachineInstr *MI) {
  switch (HWAtom::getHWAtomType(MI)) {
  default: llvm_unreachable("Can not handle MInst!"); return 0;
  case HWAtom::atomVRoot:
    return getExitRoot(MI);
  case HWAtom::atomOpFU:
    return createOpFU(MI);
  case HWAtom::atomOhters:
    // Ignore the instruction.
    return 0;
  }
}

HWAOpFU *HWAtomInfo::createOpFU(const MachineInstr *MI, unsigned ID) {
  SmallVector<HWEdge*, 4> Deps;
  addOperandDeps(MI, Deps);

  HWResType::Types ResTy = HWResType::getHWResType(MI);
  HWResType *ResType = VTarget->getResType(ResTy);

  assert(!InstToHWAtoms.count(MI) && "MI exist!");

  // TODO: Remember the register that live out this MBB.
  // and the instruction that only produce a chain.

  HWAOpFU *A = new (HWAtomAllocator) HWAOpFU(MI, ResTy, Deps.begin(), Deps.end(),
                                             ResType->getLatency(), ID, ++InstIdx);
  return A;
}

HWAtom *HWAtomInfo::getExitRoot(const MachineInstr *MI) {
  SmallVector<HWEdge*, 4> Deps;
  addOperandDeps(MI, Deps);

  // Add the instruction that defining the live out registers
  // and a chain to the terminator.

  HWAOpFU *A = new (HWAtomAllocator) HWAOpFU(MI, HWResType::Trivial,
                                             Deps.begin(), Deps.end(),
                                             0, 0, ++InstIdx);
  return A;
}
