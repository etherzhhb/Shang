//===------ PreSchedRTLOpt.cpp - Pre-schedule RTL optimization  -*- C++ -*-===//
//
// Copyright: 2011 by SYSU EDA Group. all rights reserved.
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
// This file implement the PreSchedRTLOptPass, which perform low-level
// optimization before scheduling to avoid the scheduling algorithm allocating
// unnecessary time(c-step) for operations that will be eliminated by
// downstream tools.
//
//===----------------------------------------------------------------------===//

#include "MachineFunction2Datapath.h"

#include "vtm/Passes.h"

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineDominators.h"
#include "llvm/ADT/DenseSet.h"
#include "llvm/ADT/OwningPtr.h"
#include "llvm/ADT/PostOrderIterator.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Allocator.h"
#define DEBUG_TYPE "vtm-pre-schedule-rtl-opt"
#include "llvm/Support/Debug.h"

using namespace llvm;

namespace llvm {
class VASTMachineOperand : public VASTValue {
  const MachineOperand MO;
public:
  VASTMachineOperand(const MachineOperand &MO)
    : VASTValue(VASTNode::vastMachineOperand, VInstrInfo::getBitWidth(MO)),
      MO(MO) {}

  MachineOperand getMO() const {
    return MO;
  }

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const VASTMachineOperand *A) { return true; }
  static inline bool classof(const VASTNode *A) {
    return A->getASTType() == vastMachineOperand;
  }

  void printAsOperandImpl(raw_ostream &OS, unsigned UB, unsigned LB) const {
    OS << getMO() << '[' << UB << ',' << LB << ']';
  }
};
}

namespace {
struct PreSchedRTLOpt : public MachineFunctionPass,
                        public DatapathBuilderContext {
  static char ID;
  MachineRegisterInfo *MRI;
  MachineDominatorTree *DT;
  MachineBasicBlock *Entry;
  // VASTExpr management, managed by VASTModule.
  VASTModule Mod;
  OwningPtr<DatapathBuilder> Builder;
  typedef DenseMap<MachineOperand, VASTMachineOperand*,
                   VMachineOperandValueTrait> VASTMOMapTy;
  VASTMOMapTy VASTMOs;

  VASTImmediate *getOrCreateImmediate(uint64_t Value, int8_t BitWidth) {
    return Mod.getOrCreateImmediate(Value, BitWidth);
  }

  // Remember in which MachineBasicBlock the expression is first created, we can
  // simply write this expression in the MachineBasicBlock.
  typedef DenseMap<VASTExpr*, MachineBasicBlock*> ExprLocMapTy;

  MachineBasicBlock *getDefMBB(VASTValPtr V) const;
  MachineBasicBlock *calculateInsertMBB(VASTExpr *Expr) const;
  MachineInstr *calculateInsertPos(VASTExpr *Expr, MachineInstr *CurIP) const;

  void enterMBB(MachineBasicBlock &MBB);

  void leaveMBB(MachineBasicBlock &MBB);

  typedef std::map<VASTValPtr, unsigned> Val2RegMapTy;
  Val2RegMapTy Val2Reg;

  VASTValPtr createExpr(VASTExpr::Opcode Opc, ArrayRef<VASTValPtr> Ops,
                        unsigned UB, unsigned LB) {
    return Mod.createExpr(Opc, Ops, UB, LB);;
  }

  // TODO: Remember the outputs by wires?.
  VASTValPtr getOrCreateVASTMO(MachineOperand DefMO) {
    DefMO.clearParent();
    assert((!DefMO.isReg() || !DefMO.isDef())
           && "The define flag should had been clear!");
    VASTMachineOperand *&VASTMO = VASTMOs[DefMO];
    if (!VASTMO)
      VASTMO = new VASTMachineOperand(DefMO);

    return VASTMO;
  }

  VASTValPtr getAsOperand(MachineOperand &Op, bool GetAsInlineOperand = true);

  virtual bool shouldExprBeFlatten(VASTExpr *E) {
    return E->isInlinable();
  }

  PreSchedRTLOpt() : MachineFunctionPass(ID), MRI(0), DT(0), Entry(0), Mod("m"){
    initializeMachineDominatorTreePass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF);

  void getAnalysisUsage(AnalysisUsage &AU) const {
    MachineFunctionPass::getAnalysisUsage(AU);
    AU.addRequired<MachineDominatorTree>();
    AU.setPreservesCFG();
  }

  bool optimizeMBB(MachineBasicBlock &MBB);
  VASTValPtr buildDatapath(MachineInstr *MI);

  void rewriteDepForPHI(MachineInstr *PHI, MachineInstr *IncomingPos);
  // Rewrite the expression and all its dependences.
  unsigned rewriteExprTree(VASTExprPtr V, MachineInstr *InsertBefore);
  void rewriteExprTreeForMO(MachineOperand &MO, MachineInstr *InsertBefore,
                            bool isPredicate, bool isPHI);
  // Rewrite the single expression, return the register which hold the result of
  // the expression.
  unsigned rewriteExpr(VASTExprPtr Expr, MachineInstr *InsertBefore);

  // Functions to rewrite VASTExpr to MachineInstr.
  template<unsigned Opcode>
  unsigned rewriteUnary(VASTExpr *Expr, MachineInstr *IP);
  unsigned rewriteRXor(VASTExpr *Expr, MachineInstr *IP);
  unsigned rewriteSel(VASTExpr *Expr, MachineInstr *IP);
  unsigned rewriteAdd(VASTExpr *Expr, MachineInstr *IP);
  unsigned rewriteAssign(VASTExpr *Expr);
  template<unsigned Opcode, typename BitwidthFN>
  unsigned rewriteNAryExpr(VASTExpr *Expr, MachineInstr *IP, BitwidthFN F);
  template<unsigned Opcode>
  unsigned rewriteBinExpr(VASTExpr *Expr, MachineInstr *IP);
  template<VFUs::ICmpFUType ICmpTy>
  unsigned rewriteICmp(VASTExpr *Expr, MachineInstr *IP);
  unsigned rewriteNotOf(VASTExpr *Expr);
  void buildNot(unsigned DstReg, const MachineOperand &Op);

  unsigned lookupRegNum(VASTValPtr V) const {
    Val2RegMapTy::const_iterator at = Val2Reg.find(V);
    return at == Val2Reg.end() ? 0 : at->second;
  }

  MachineInstr *getRewrittenInstr(VASTValPtr V) const {
    if (unsigned RegNum = lookupRegNum(V))
      return MRI->getVRegDef(RegNum);

    return 0;
  }

  unsigned getRewrittenRegNum(VASTValPtr V) const {
    unsigned RegNum = lookupRegNum(V);
    if (RegNum && MRI->getVRegDef(RegNum))
      return RegNum;

    return 0;
  }

  MachineInstr *getInsertPos(const MachineOperand &MO) {
    // Insert at the begin of the function by default.
    typedef MachineBasicBlock::instr_iterator instr_it;
    instr_it IP = Entry->instr_begin();

    if (MO.isReg() && MO.getReg()) {
      MachineInstr *Def = MRI->getVRegDef(MO.getReg());
      assert(Def && "Register is not yet defined!");
      IP = Def;
      // Insert the invert operation right after the definition of source
      // operand, and make sure we skip all PHINodes.
      while((++IP)->isPHI())
        ;
    }

    return IP;
  }

  MachineOperand getAsOperand(VASTValPtr V);

  unsigned allocateRegNum(VASTValPtr V) {
    unsigned RegNum = lookupRegNum(V);
    assert((!RegNum || !MRI->getVRegDef(RegNum))
           && "Reg already allocated for expression!");
    if (RegNum == 0) {
      RegNum = MRI->createVirtualRegister(&VTM::DRRegClass);
      // Index the expression by the the newly created register number.
      if (V) Builder->indexVASTExpr(RegNum, V);
    }

    return RegNum;;
  }

  MachineOperand allocateRegMO(VASTValPtr V, unsigned BitWidth = 0) {
    if (BitWidth == 0) BitWidth = V->getBitWidth();
    return VInstrInfo::CreateReg(allocateRegNum(V), BitWidth, true);;
  }

  template<bool AllowDifference>
  unsigned rememberRegNumForExpr(VASTValPtr V, unsigned RegNo) {
    bool inserted;
    Val2RegMapTy::iterator at;
    assert(TargetRegisterInfo::isVirtualRegister(RegNo)
           && TargetRegisterInfo::virtReg2Index(RegNo) < MRI->getNumVirtRegs()
           && "Bad RegNo!");
    tie(at, inserted) = Val2Reg.insert(std::make_pair(V, RegNo));
    if (!inserted && at->second != RegNo) {
      assert(AllowDifference && "Expr is rewritten twice?");
      MRI->replaceRegWith(RegNo, at->second);
      RegNo = at->second;
    }

    return RegNo;
  }

  void releaseMemory() {
    Builder.reset();
    DeleteContainerSeconds(VASTMOs);
    Val2Reg.clear();
    Mod.reset();
  }
};
}

template<typename T>
inline static T *check(T *Ptr) {
  assert(Ptr && "Bad pointer!");
  return Ptr;
}

Pass *llvm::createPreSchedRTLOptPass() {
  return new PreSchedRTLOpt();
}

char PreSchedRTLOpt::ID = 0;

bool PreSchedRTLOpt::runOnMachineFunction(MachineFunction &F) {
  bool Changed = false;
  MRI = &F.getRegInfo();
  DT = &getAnalysis<MachineDominatorTree>();
  Builder.reset(new DatapathBuilder(*this, *MRI));

  Entry = F.begin();
  ReversePostOrderTraversal<MachineBasicBlock*> RPOT(Entry);
  typedef ReversePostOrderTraversal<MachineBasicBlock*>::rpo_iterator rpo_it;

  for (rpo_it I = RPOT.begin(), E = RPOT.end(); I != E; ++I)
    Changed = optimizeMBB(**I);

  // Verify the function.
  F.verify(this);
  return Changed;
}

bool PreSchedRTLOpt::optimizeMBB(MachineBasicBlock &MBB) {
  enterMBB(MBB);

  DEBUG(dbgs() << "Before Rtl Optimization:\n";
        MBB.dump(););

  MachineBasicBlock::instr_iterator I = MBB.instr_begin(), E = MBB.instr_end();

  // Skip the PHINodes.
  while (I->isPHI())
    ++I;

  typedef MachineBasicBlock::instr_iterator instr_iterator;
  instr_iterator InsertPos = I;
  bool LastMIIsTerminator = false;
  while (I != E) {
    // Do not insert the instruction between terminators.
    if (!LastMIIsTerminator) InsertPos = I;
    MachineInstr *MI = I++;
    LastMIIsTerminator |= MI->isTerminator();

    // Try to add the instruction into the Datapath net-list.
    if (VASTValPtr V = buildDatapath(MI)) {
      MI->eraseFromParent();
      continue;
    }

    if (MI->getOpcode() == VTM::VOpMove) {
      VASTValPtr Src = getAsOperand(MI->getOperand(1));
      unsigned RegNum = MI->getOperand(0).getReg();
      // Remember the register number mapping, the register maybe CSEd.
      if (RegNum == rememberRegNumForExpr<true>(Src, RegNum))
        Builder->indexVASTExpr(RegNum, Src);
      MI->eraseFromParent();
      continue;
    }

    const MCInstrDesc &TID = MI->getDesc();

    for (unsigned i = 0, e = MI->getNumOperands(); i != e; ++i) {
      MachineOperand &MO = MI->getOperand(i);
      // Only care about a use register.
      if (!MO.isReg() || MO.isDef() || MO.getReg() == 0)
        continue;

      bool IsPredicate = i < TID.getNumOperands()&&TID.OpInfo[i].isPredicate();
      // All operand of br instruction are predicates.
      IsPredicate |= VInstrInfo::isBrCndLike(TID.getOpcode());
      rewriteExprTreeForMO(MO, InsertPos, IsPredicate, false);
    }
  }

  // Rewrite the incoming value from this MBB.
  typedef MachineBasicBlock::succ_iterator succ_iterator;
  for (succ_iterator SI = MBB.succ_begin(), SE = MBB.succ_end(); SI != SE; ++SI) {
    MachineBasicBlock::instr_iterator PHI = (*SI)->instr_begin();
    for (instr_iterator PI = (*SI)->instr_begin(), PE = (*SI)->instr_end();
         PI != PE && PI->isPHI(); ++PI)
      rewriteDepForPHI(PHI++, InsertPos);
  }


  leaveMBB(MBB);
  return true;
}

void PreSchedRTLOpt::rewriteDepForPHI(MachineInstr *PHI,
                                      MachineInstr *IncomingPos) {
  for (unsigned i = 1, e = PHI->getNumOperands(); i != e; i += 2) {
    // Only rewrite the dependence from incoming BB.
    if (PHI->getOperand(i + 1).getMBB() != IncomingPos->getParent())
      continue;

    MachineOperand &MO = PHI->getOperand(i);

    rewriteExprTreeForMO(MO, IncomingPos, false, true);
  }
}

void PreSchedRTLOpt::rewriteExprTreeForMO(MachineOperand &MO, MachineInstr *IP,
                                          bool isPredicate, bool isPHI) {
  MO.setIsKill(false);
  VASTValPtr V = Builder->lookupExpr(MO.getReg());

  if (!V) {
    assert(MRI->getVRegDef(MO.getReg()));
    return;
  }

  if (!isPredicate) {
    // Fix the bit width.
    unsigned Bitwidth = V->getBitWidth();
    if (Bitwidth < VInstrInfo::getBitWidth(MO)) {
      assert(!isPHI && "The bitwidth of incoming value should not be changed!");
      VInstrInfo::setBitWidth(MO, Bitwidth);
    }
  } else if (VInstrInfo::isPredicateInverted(MO))
    V = V.invert();

  // Rewrite the instruction tree.
  if (VASTExprPtr Expr = dyn_cast<VASTExprPtr>(V)) {
    unsigned NewRegNo = rewriteExprTree(Expr, IP);
    if (NewRegNo != MO.getReg()) {
      assert(isPredicate && VInstrInfo::isPredicateInverted(MO)
             && "Unexpected register number changed!");
      MO.ChangeToRegister(NewRegNo, false);
      VInstrInfo::setBitWidth(MO, 1);
    }

    return;
  }

  if (MRI->getVRegDef(MO.getReg()))
    return;

  MachineOperand NewMO = getAsOperand(V);
  if (NewMO.isReg()) {
    MO.ChangeToRegister(NewMO.getReg(), false);
    return;
  }

  if (!isPHI && NewMO.isImm()) {
    MO.ChangeToImmediate(NewMO.getImm());
    return;
  }

  IP = Entry->instr_begin();
  // A copy instruction is needed.
  MachineOperand Dst = MO;
  Dst.setIsDef(true);
  DebugLoc dl;
  BuildMI(*IP->getParent(), IP, dl, VInstrInfo::getDesc(VTM::VOpMove))
    .addOperand(Dst).addOperand(NewMO)
    .addOperand(VInstrInfo::CreatePredicate())
    .addOperand(VInstrInfo::CreateTrace());
}

VASTValPtr PreSchedRTLOpt::buildDatapath(MachineInstr *MI) {
  if (!VInstrInfo::isDatapath(MI->getOpcode())) return 0;

  unsigned ResultReg = MI->getOperand(0).getReg();
  VASTValPtr V = Builder->buildDatapathExpr(MI);

  // Remember the register number mapping, the register maybe CSEd.
  unsigned FoldedReg = rememberRegNumForExpr<true>(V, ResultReg);
  // If ResultReg is not CSEd to other Regs, index the newly created Expr.
  if (FoldedReg == ResultReg)
    Builder->indexVASTExpr(FoldedReg, V);

  return V;
}

unsigned PreSchedRTLOpt::rewriteExprTree(VASTExprPtr Expr, MachineInstr *IP) {
  typedef VASTValue::dp_dep_it ChildIt;
  std::vector<std::pair<VASTExprPtr, ChildIt> > VisitStack;
  unsigned NewRegNo = getRewrittenRegNum(Expr);

  // Is the expression already rewritten?
  if (NewRegNo) return NewRegNo;

  VisitStack.push_back(std::make_pair(Expr, Expr->op_begin()));
  while (!VisitStack.empty()) {
    VASTExprPtr CurExpr = VisitStack.back().first;
    ChildIt It = VisitStack.back().second;

    // All depenece of this node is visited.
    if (It == CurExpr->op_end()) {
      NewRegNo = rewriteExpr(CurExpr, calculateInsertPos(CurExpr.get(), IP));

      VisitStack.pop_back();
      continue;
    }

    // Otherwise, remember the node and visit its children first.
    VASTExprPtr ChildNode = dyn_cast<VASTExprPtr>(It->get());
    ++VisitStack.back().second;

    // No need to visit the leaves and the rewritten expressions.
    if (!ChildNode || getRewrittenRegNum(ChildNode)) continue;

    VisitStack.push_back(std::make_pair(ChildNode, ChildNode->op_begin()));
  }

  return NewRegNo;
}

template<unsigned Opcode>
unsigned PreSchedRTLOpt::rewriteUnary(VASTExpr *Expr, MachineInstr *IP) {
  MachineOperand DefMO = allocateRegMO(Expr);
  BuildMI(*IP->getParent(), IP, DebugLoc(), VInstrInfo::getDesc(Opcode))
    .addOperand(DefMO)
    .addOperand(getAsOperand(Expr->getOperand(0)))
    .addOperand(VInstrInfo::CreatePredicate())
    .addOperand(VInstrInfo::CreateTrace());
  return DefMO.getReg();
}

template<unsigned Opcode>
unsigned PreSchedRTLOpt::rewriteBinExpr(VASTExpr *Expr, MachineInstr *IP) {
  MachineOperand DefMO = allocateRegMO(Expr);
  BuildMI(*IP->getParent(), IP, DebugLoc(), VInstrInfo::getDesc(Opcode))
    .addOperand(DefMO)
    .addOperand(getAsOperand(Expr->getOperand(0)))
    .addOperand(getAsOperand(Expr->getOperand(1)))
    .addOperand(VInstrInfo::CreatePredicate())
    .addOperand(VInstrInfo::CreateTrace());
  return DefMO.getReg();
}

unsigned PreSchedRTLOpt::rewriteSel(VASTExpr *Expr, MachineInstr *IP) {
  MachineOperand DefMO = allocateRegMO(Expr);
  assert(Expr->getOperand(0)->getASTType() != VASTNode::vastImmediate);
  BuildMI(*IP->getParent(), IP, DebugLoc(), VInstrInfo::getDesc(VTM::VOpSel))
    .addOperand(DefMO)
    .addOperand(getAsOperand(Expr->getOperand(0)))
    .addOperand(getAsOperand(Expr->getOperand(1)))
    .addOperand(getAsOperand(Expr->getOperand(2)))
    .addOperand(VInstrInfo::CreatePredicate())
    .addOperand(VInstrInfo::CreateTrace());
  return DefMO.getReg();
}

unsigned PreSchedRTLOpt::rewriteAdd(VASTExpr *Expr, MachineInstr *IP) {
  assert(Expr->NumOps > 1 && Expr->NumOps < 4 && "Bad operand number!");
  MachineOperand DefMO = allocateRegMO(Expr);
  MachineOperand LHS = getAsOperand(Expr->getOperand(0));
  MachineOperand RHS = getAsOperand(Expr->getOperand(1));
  MachineOperand Carry = VInstrInfo::CreateImm(0, 1);
  if (Expr->NumOps == 3) Carry = getAsOperand(Expr->getOperand(2));

  BuildMI(*IP->getParent(), IP, DebugLoc(), VInstrInfo::getDesc(VTM::VOpAdd_c))
    .addOperand(DefMO).addOperand(LHS).addOperand(RHS).addOperand(Carry)
    .addOperand(VInstrInfo::CreatePredicate())
    .addOperand(VInstrInfo::CreateTrace());
  return DefMO.getReg();
}

template<VFUs::ICmpFUType ICmpTy>
unsigned PreSchedRTLOpt::rewriteICmp(VASTExpr *Expr, MachineInstr *IP) {
  MachineOperand DefMO = allocateRegMO(Expr);
  BuildMI(*IP->getParent(), IP, DebugLoc(), VInstrInfo::getDesc(VTM::VOpICmp_c))
    .addOperand(DefMO)
    .addOperand(getAsOperand(Expr->getOperand(0)))
    .addOperand(getAsOperand(Expr->getOperand(1)))
    .addOperand(VInstrInfo::CreateImm(ICmpTy, 8))
    .addOperand(VInstrInfo::CreatePredicate())
    .addOperand(VInstrInfo::CreateTrace());
  return DefMO.getReg();
}


void PreSchedRTLOpt::buildNot(unsigned DstReg, const MachineOperand &Op) {
  MachineInstr *IP = getInsertPos(Op);
  BuildMI(*IP->getParent(), IP, DebugLoc(), VInstrInfo::getDesc(VTM::VOpNot))
    .addOperand(VInstrInfo::CreateReg(DstReg,VInstrInfo::getBitWidth(Op),true))
    .addOperand(Op).addOperand(VInstrInfo::CreatePredicate())
    .addOperand(VInstrInfo::CreateTrace());
}

unsigned PreSchedRTLOpt::rewriteNotOf(VASTExpr *Expr) {
  // Allocate register for the result of the invertion.
  MachineOperand DefMO = allocateRegMO(VASTExprPtr(Expr, true));
  buildNot(DefMO.getReg(), getAsOperand(Expr));
  return DefMO.getReg();
}

unsigned PreSchedRTLOpt::rewriteAssign(VASTExpr *Expr) {
  assert(Expr->isSubBitSlice() && "Unexpected trivial assign!");

  MachineOperand SrcMO = getAsOperand(Expr->getOperand(0));
  MachineOperand DefMO = allocateRegMO(Expr);
  // Do not create self-assign.
  //if (SrcMO.isReg() && SrcMO.getReg() == DefMO.getReg())
  //  return DefMO.getReg();
  
  MachineInstr *IP = getInsertPos(SrcMO);
  DebugLoc dl;

  BuildMI(*IP->getParent(), IP, dl, VInstrInfo::getDesc(VTM::VOpBitSlice))
    .addOperand(DefMO).addOperand(SrcMO)
    .addOperand(VInstrInfo::CreateImm(Expr->UB, 8))
    .addOperand(VInstrInfo::CreateImm(Expr->LB, 8))
    .addOperand(VInstrInfo::CreatePredicate())
    .addOperand(VInstrInfo::CreateTrace());
  return DefMO.getReg();
}

static unsigned GetBitCatWidth(const MachineOperand &LHS,
                               const MachineOperand &RHS) {
  return VInstrInfo::getBitWidth(LHS) + VInstrInfo::getBitWidth(RHS);
}

static unsigned GetSameWidth(const MachineOperand &LHS,
                               const MachineOperand &RHS) {
  unsigned BitWidth = VInstrInfo::getBitWidth(LHS);
  assert(BitWidth == VInstrInfo::getBitWidth(RHS) && "Bitwidth not match!");
  return BitWidth;
}


template<unsigned Opcode, typename BitwidthFN>
unsigned PreSchedRTLOpt::rewriteNAryExpr(VASTExpr *Expr, MachineInstr *IP,
                                         BitwidthFN F) {
  SmallVector<MachineOperand, 8> Ops;
  for (unsigned i = 0; i < Expr->NumOps; ++i)
    Ops.push_back(getAsOperand(Expr->getOperand(i)));

  while (Ops.size() > 2) {
    unsigned ResultPos = 0;
    unsigned OperandPos = 0;
    unsigned NumOperand = Ops.size();
    while (OperandPos + 1 < NumOperand) {
      MachineOperand LHS = Ops[OperandPos];
      MachineOperand RHS = Ops[OperandPos + 1];
      MachineOperand DefMO = allocateRegMO(0, F(LHS, RHS));
      OperandPos += 2;
      BuildMI(*IP->getParent(), IP, DebugLoc(), VInstrInfo::getDesc(Opcode))
        .addOperand(DefMO)
        .addOperand(LHS).addOperand(RHS)
        .addOperand(VInstrInfo::CreatePredicate())
        .addOperand(VInstrInfo::CreateTrace());
      DefMO.setIsDef(false);
      Ops[ResultPos++] = DefMO;
    }

    // Move the rest of the operand.
    while (OperandPos < NumOperand)
      Ops[ResultPos++] = Ops[OperandPos++];

    // Only preserve the results.
    Ops.resize(ResultPos);
  }

  MachineOperand DefMO = allocateRegMO(Expr);
  BuildMI(*IP->getParent(), IP, DebugLoc(), VInstrInfo::getDesc(Opcode))
    .addOperand(DefMO)
    .addOperand(Ops[0]).addOperand(Ops[1])
    .addOperand(VInstrInfo::CreatePredicate())
    .addOperand(VInstrInfo::CreateTrace());

  return DefMO.getReg();
}

unsigned PreSchedRTLOpt::rewriteExpr(VASTExprPtr E, MachineInstr *IP) {
  bool isInverted = E.isInverted();
  VASTExpr *Expr = E.get();

  unsigned RegNo = lookupRegNum(Expr);
  // The expression itself may had been rewritten, but the invert is not yet
  // rewritten.
  assert((!getRewrittenRegNum(Expr) || isInverted) && "E had already rewritten!");

  if (!getRewrittenRegNum(Expr)) {
    switch (Expr->getOpcode()) {
    default: llvm_unreachable("Unsupported expression!");
    case VASTExpr::dpAnd:
      RegNo = rewriteNAryExpr<VTM::VOpAnd>(Expr, IP, GetSameWidth);
      break;
    case VASTExpr::dpRAnd:
      RegNo = rewriteUnary<VTM::VOpRAnd>(Expr, IP);
      break;
    case VASTExpr::dpRXor:
      RegNo = rewriteUnary<VTM::VOpRXor>(Expr, IP);
      break;
    case VASTExpr::dpSel:
      RegNo = rewriteSel(Expr, IP);
      break;
    case VASTExpr::dpBitCat:
      RegNo = rewriteNAryExpr<VTM::VOpBitCat>(Expr, IP, GetBitCatWidth);
      break;
    case VASTExpr::dpBitRepeat:
      RegNo = rewriteBinExpr<VTM::VOpBitRepeat>(Expr, IP);
      break;
    case VASTExpr::dpAssign:
      RegNo = rewriteAssign(Expr);
      break;
    case VASTExpr::dpAdd:
      RegNo = rewriteAdd(Expr, IP);
      break;
    case VASTExpr::dpMul:
      RegNo = rewriteBinExpr<VTM::VOpMult_c>(Expr, IP);
      break;
    case VASTExpr::dpShl:
      RegNo = rewriteBinExpr<VTM::VOpSHL_c>(Expr, IP);
      break;
    case VASTExpr::dpSRA:
      RegNo = rewriteBinExpr<VTM::VOpSRA_c>(Expr, IP);
      break;
    case VASTExpr::dpSRL:
      RegNo = rewriteBinExpr<VTM::VOpSRL_c>(Expr, IP);
      break;
    case VASTExpr::dpUCmp:
      RegNo = rewriteICmp<VFUs::CmpUnsigned>(Expr, IP);
      break;
    case VASTExpr::dpSCmp:
      RegNo = rewriteICmp<VFUs::CmpSigned>(Expr, IP);
      break;
    }

    rememberRegNumForExpr<false>(Expr, RegNo);
  }

  if (isInverted) {
    RegNo = rewriteNotOf(Expr);
    rememberRegNumForExpr<false>(VASTValPtr(Expr, true), RegNo);
  }

  return RegNo;
}

VASTValPtr
PreSchedRTLOpt::getAsOperand(MachineOperand &Op, bool GetAsInlineOperand) {
  unsigned BitWidth = VInstrInfo::getBitWidth(Op);
  switch (Op.getType()) {
  case MachineOperand::MO_Register: {
    unsigned Reg = Op.getReg();
    if (!Reg) return 0;

    VASTValPtr V = Builder->lookupExpr(Reg);

    if (!V) {
      MachineInstr *DefMI = check(MRI->getVRegDef(Reg));
      assert(VInstrInfo::isControl(DefMI->getOpcode())
             && "Reg defined by data-path should had already been indexed!");
      MachineOperand DefMO = DefMI->getOperand(0);
      DefMO.setIsDef(false);
      V = getOrCreateVASTMO(DefMO);
    }

    // The operand may only use a sub bitslice of the signal.
    V = Builder->buildBitSliceExpr(V, BitWidth, 0);
    // Try to inline the operand.
    if (GetAsInlineOperand) V = V.getAsInlineOperand();
    return V;
                                    }
  case MachineOperand::MO_Immediate:
    return getOrCreateImmediate(Op.getImm(), BitWidth);
  default: break;
  }

  return getOrCreateVASTMO(Op);
}

MachineOperand PreSchedRTLOpt::getAsOperand(VASTValPtr V) {
  if (unsigned RegNum = getRewrittenRegNum(V))
    return VInstrInfo::CreateReg(RegNum, V->getBitWidth());

  if (VASTExprPtr E = dyn_cast<VASTExprPtr>(V)) {
    if (E->isZeroBasedBitSlice()) {
      MachineOperand MO = getAsOperand(E.getOperand(0));
      VInstrInfo::setBitWidth(MO, V->getBitWidth());
      return MO;
    }
  }

  if (VASTImmPtr Imm = dyn_cast<VASTImmPtr>(V))
    return VInstrInfo::CreateImm(Imm.getUnsignedValue(), Imm->getBitWidth());

  if (VASTMachineOperand *VASTMO = dyn_cast<VASTMachineOperand>(V))
    return VASTMO->getMO();

  // Try to invert the value and find again.
  VASTValPtr Inv = V.invert();
  MachineOperand MO = getAsOperand(Inv);

  // Build the not operation.
  unsigned RegNum = allocateRegNum(V);
  buildNot(RegNum, MO);

  return VInstrInfo::CreateReg(RegNum, V->getBitWidth());
}

MachineBasicBlock *PreSchedRTLOpt::getDefMBB(VASTValPtr V) const {
  if (isa<VASTImmediate>(V.get())) return Entry;

  if (VASTMachineOperand *VASTMO = dyn_cast<VASTMachineOperand>(V.get())) {
    const MachineOperand &MO = VASTMO->getMO();
    if (!MO.isReg() || !MO.getReg())
      return Entry;

    return check(MRI->getVRegDef(MO.getReg()))->getParent();
  }

  return check(getRewrittenInstr(V.get()))->getParent();
}

MachineBasicBlock *PreSchedRTLOpt::calculateInsertMBB(VASTExpr *Expr) const {
  MachineBasicBlock *MBB = Entry;

  // Find a MBB that dominated by all defining MBB of the operands.
  // In a dominator tree, these MBBs should distributes in the nodes of a path.
  for (unsigned i = 0; i < Expr->NumOps; ++i) {
    MachineBasicBlock *CurMBB = getDefMBB(Expr->getOperand(i));
    if (DT->dominates(MBB, CurMBB)) {
      MBB = CurMBB;
      continue;
    }

    assert(DT->dominates(CurMBB, MBB)
           && "Operands not in a path of the dominator tree!");
  }


  return MBB;
}

MachineInstr *PreSchedRTLOpt::calculateInsertPos(VASTExpr *Expr,
                                                 MachineInstr *CurIP) const {
  MachineBasicBlock *MBB = calculateInsertMBB(Expr);
  if (MBB == CurIP->getParent()) return CurIP;

  return MBB->getFirstInstrTerminator();
}

void PreSchedRTLOpt::leaveMBB(MachineBasicBlock &MBB) {
  bool Termiator = false;
  typedef MachineBasicBlock::instr_iterator it;
  for (it I = it(MBB.getFirstNonPHI()), E = MBB.instr_end(); I != E; ++I) {
    assert(!I->isPHI());
    assert(!Termiator || I->isTerminator());
    Termiator |= I->isTerminator();
  }
}

void PreSchedRTLOpt::enterMBB(MachineBasicBlock &MBB) {
  // Fix the terminators so we can safely insert the instructions before the
  // first terminator of a MBB.
  fixTerminators(&MBB);
}
