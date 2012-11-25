//===---------- VInstrInfo.cpp - VTM Instruction Information -----*- C++ -*-===//
//
//                      The Shang HLS frameowrk                               //
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the VTM implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//


#include "VTargetMachine.h"

#include "vtm/VerilogBackendMCTargetDesc.h"
#include "vtm/VFInfo.h"
#include "vtm/VInstrInfo.h"
#include "vtm/Utilities.h"
#include "vtm/DetailLatencyInfo.h"

#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/Hashing.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/CommandLine.h"
#define DEBUG_TYPE "vtm-instr-info"
#include "llvm/Support/Debug.h"
#define GET_INSTRINFO_CTOR
#include "VerilogBackendGenInstrInfo.inc"

namespace llvm {
extern const MCInstrDesc VTMInsts[];
}

using namespace llvm;
//----------------------------------------------------------------------------//
// Halper function.
static MachineInstr *addOperandsToMI(MachineInstr *MI,
                                     ArrayRef<MachineOperand> Ops) {
  for (unsigned i = 0; i < Ops.size(); ++i)
    MI->addOperand(Ops[i]);

  return MI;
}

//----------------------------------------------------------------------------//
// VInstrInfo implementation.
VInstrInfo::VInstrInfo() : VTMGenInstrInfo(), RI() {}

const MCInstrDesc &VInstrInfo::getDesc(unsigned Opcode)  {
  return VTMInsts[Opcode];
}

const MachineOperand *VInstrInfo::getPredOperand(const MachineInstr *MI) {
  if (MI->getOpcode() <= TargetOpcode::COPY) return 0;

  unsigned Idx = MI->getDesc().NumOperands - 2;
  assert(MI->getDesc().OpInfo[Idx].isPredicate() && "Cannot get PredOperand!");
  return &MI->getOperand(Idx);
}

MachineOperand *VInstrInfo::getPredOperand(MachineInstr *MI) {
  return const_cast<MachineOperand*>(getPredOperand((const MachineInstr*)MI));
}

bool VInstrInfo::isReallyTriviallyReMaterializable(const MachineInstr *MI,
                                                   AliasAnalysis *AA) const {
  const MCInstrDesc &TID = MI->getDesc();
  return !TID.isBarrier() && hasTrivialFU(TID.getOpcode()) &&
         // Dont rematerialize control ops if the block already scheduled.
         (isDatapath(TID.getOpcode()) ||
          MI->getParent()->back().getOpcode() != VTM::EndState);
}

bool VInstrInfo::isPredicable(MachineInstr *MI) const {
  return MI->getOpcode() > VTM::COPY && MI->getOpcode() != VTM::VOpRet;
}

bool VInstrInfo::isPredicated(const MachineInstr *MI) const {
  // Pseudo machine instruction are never predicated.
  if (!isPredicable(const_cast<MachineInstr*>(MI))) return false;

  if (const MachineOperand *Pred = getPredOperand(MI))
    return (Pred->isReg() && Pred->getReg() != 0);

  return false;
}

void VInstrInfo::ChangeCopyToMove(MachineInstr *CopyMI) {
  CopyMI->setDesc(getDesc(VTM::VOpMove));
  CopyMI->addOperand(VInstrInfo::CreatePredicate());
  CopyMI->addOperand(VInstrInfo::CreateTrace());
}

bool VInstrInfo::shouldAvoidSinking(MachineInstr *MI) const {
  return MI->getOpcode() == VTM::VOpMoveArg || isDatapath(MI->getOpcode());
}

MachineInstr *VInstrInfo::commuteInstruction(MachineInstr *MI, bool NewMI)const{
  const MCInstrDesc &TID = MI->getDesc();
  bool HasDef = TID.getNumDefs();
  assert(HasDef && MI->getOperand(0).isReg() && "Bad instruction to commute!");
  // Build the operand list and swap the operands to cummuted.
  SmallVector<MachineOperand, 6> MOs;
  typedef MachineInstr::mop_iterator it;
  for (it I = MI->operands_begin(), E = MI->operands_end(); I != E; ++I)
    MOs.push_back(*I);

  std::swap(MOs[1], MOs[2]);

  // Build a empty MI or clear the operands in the original MI, and re-insert
  // the operands to MI.
  if (NewMI) {
    MachineFunction &MF = *MI->getParent()->getParent();
    MI = BuildMI(MF, MI->getDebugLoc(), TID);
  } else
    while (MI->getNumOperands())
      MI->RemoveOperand(MI->getNumOperands() - 1);

  return addOperandsToMI(MI, MOs);
}

bool VInstrInfo::isUnpredicatedTerminator(const MachineInstr *MI) const{
  const MCInstrDesc &TID = MI->getDesc();
  if (!TID.isTerminator()) return false;

  return !isPredicated(MI);
}

bool VInstrInfo::isUnConditionalBranch(MachineInstr *MI) {
  if (!VInstrInfo::isBrCndLike(MI->getOpcode())) return false;

  MachineOperand &Cnd = MI->getOperand(0);
  return (Cnd.isReg() && Cnd.getReg() == 0) || Cnd.isImm();
}

bool VInstrInfo::AnalyzeBranch(MachineBasicBlock &MBB, MachineBasicBlock *&TBB,
                               MachineBasicBlock *&FBB,
                               SmallVectorImpl<MachineOperand> &Cond,
                               bool AllowModify /* = false */) const {
  if (MBB.empty()) return false;

  // Do not mess with the scheduled code.
  if (MBB.back().getOpcode() == VTM::EndState)
    return true;

  /// 1. If this block ends with no branches (it just falls through to its succ)
  ///    just return false, leaving TBB/FBB null.
  if (MBB.getFirstTerminator() == MBB.end()) return false;

  SmallVector<MachineInstr*, 4> Terms;
  for (MachineBasicBlock::iterator I = MBB.getFirstTerminator(), E = MBB.end();
       I != E; ++I) {
    MachineInstr *Inst = I;
    if (!Inst->getDesc().isTerminator()) continue;

    if (VInstrInfo::isBrCndLike(Inst->getOpcode())) {
      if (!isUnpredicatedTerminator(Inst)) return true;

      Terms.push_back(Inst);
    } else // Do not mess up with non-brcnd like terminators, i.e. return
      return true;
  }

  // Mixing branches and return?
  if (Terms.empty() || Terms.size() > 2) return true;

  MachineInstr *FstTerm = Terms[0];

  /// 2. If this block ends with only an unconditional branch, it sets TBB to be
  ///    the destination block.
  if (isUnConditionalBranch(FstTerm)) {
    TBB = FstTerm->getOperand(1).getMBB();
    assert(Terms.size() == 1 && "Expect single fall through edge!");
    return false;
  }

  Cond.push_back(FstTerm->getOperand(0));
  Cond.back().setIsKill(false);
  TBB = FstTerm->getOperand(1).getMBB();
  /// 3. If this block ends with a conditional branch and it falls through to a
  ///    successor block, it sets TBB to be the branch destination block and a
  ///    list of operands that evaluate the condition. These operands can be
  ///    passed to other TargetInstrInfo methods to create new branches.
  if (Terms.size() == 1) return false;

  /// 4. If this block ends with a conditional branch followed by an
  ///    unconditional branch, it returns the 'true' destination in TBB, the
  ///    'false' destination in FBB, and a list of operands that evaluate the
  ///    condition.  These operands can be passed to other TargetInstrInfo
  ///    methods to create new branches.
  MachineInstr *SndTerm = Terms[1];

  // Avoid strange branches.
  MachineOperand &SndPred = SndTerm->getOperand(0);
  if (SndPred.isReg()) {
    unsigned Reg = SndTerm->getOperand(0).getReg();
    if (Reg != 0 && Reg != Cond.front().getReg())
      return true;
  }

  FBB = SndTerm->getOperand(1).getMBB();
  return false;
}

bool VInstrInfo::extractJumpTable(MachineBasicBlock &BB, JT &Table, bool BrOnly)
{
  for (MachineBasicBlock::iterator I = BB.getFirstTerminator(), E = BB.end();
       I != E; ++I) {
    // We can only handle conditional jump.
    if (!VInstrInfo::isBrCndLike(I->getOpcode())) {
      if (BrOnly) return true;

      const MachineOperand *Pred = getPredOperand(I);
      MachineBasicBlock *TargetBB = 0;
      bool inserted = Table.insert(std::make_pair(TargetBB, *Pred)).second;
      assert(inserted && "BB with multiple entry in jump table?");
      continue;
    }

    // Do not mess up with the predicated terminator at the moment.
    if (const MachineOperand *Pred = getPredOperand(I))
      if (Pred->isReg() && Pred->getReg() != 0)
        return true;

    MachineBasicBlock *TargetBB = I->getOperand(1).getMBB();
    MachineOperand Cnd = I->getOperand(0);
    bool inserted = Table.insert(std::make_pair(TargetBB, Cnd)).second;
    assert(inserted && "BB with multiple entry in jump table?");
  }

  return false;
}

unsigned VInstrInfo::RemoveBranch(MachineBasicBlock &MBB) const {
  // Do not mess with the scheduled code.
  if (MBB.back().getOpcode() == VTM::EndState)
    return 0;

  // Just a fall through edge, return false and leaving TBB/FBB null.
  if (MBB.getFirstTerminator() == MBB.end()) return true;

  // Collect the branches and remove them.
  SmallVector<MachineInstr*, 4> Terms;
  for (MachineBasicBlock::iterator I = MBB.getFirstTerminator(), E = MBB.end();
       I != E; ++I) {
    MachineInstr *Inst = I;
    assert(Inst->getDesc().isTerminator() && "Broken terminator!");

    // Also remove return operation.
    Terms.push_back(Inst);
  }

  unsigned RemovedBranches = 0;
  while (!Terms.empty()) {
    Terms.pop_back_val()->removeFromParent();
    ++RemovedBranches;
  }

  return RemovedBranches;
}

void VInstrInfo::ReversePredicateCondition(MachineOperand &Cond) {
  assert(Cond.isReg() && "Broken predicate condition!");
  Cond.setTargetFlags(Cond.getTargetFlags() ^ VInstrInfo::PredInvertFlag);
}

bool
VInstrInfo::ReverseBranchCondition(SmallVectorImpl<MachineOperand> &Cond) const{
  // Invert the invert condition flag.
  for(unsigned i = 0, e = Cond.size(); i < e; ++i)
    ReversePredicateCondition(Cond[i]);
  return false;
}

unsigned VInstrInfo::InsertBranch(MachineBasicBlock &MBB,
                                  MachineBasicBlock *TBB,
                                  MachineBasicBlock *FBB,
                                  const SmallVectorImpl<MachineOperand> &Cond,
                                  DebugLoc DL) const {
  assert((Cond.size() <= 1) && "Too much conditions!");
  bool isUnconditional = Cond.empty();
  MachineOperand PredOp = isUnconditional ?
                          VInstrInfo::CreatePredicate() : Cond[0];
  PredOp.setIsKill(false);

  if (FBB == 0) {
    // Insert barrier branch for unconditional branch.
    unsigned Opc = isUnconditional ? VTM::VOpToStateb : VTM::VOpToState;
    BuildMI(&MBB, DL, get(Opc)).addOperand(PredOp).addMBB(TBB)
      .addOperand(VInstrInfo::CreatePredicate())
      .addOperand(VInstrInfo::CreateTrace());
    return 1;
  }

  // Two-way conditional branch.
  assert(PredOp.isReg() && PredOp.getReg() != 0
         && "Uncondtional predicate with true BB and false BB?");
  // Branch to true BB, with the no-barrier version.
  BuildMI(&MBB, DL, get(VTM::VOpToState)).addOperand(PredOp).addMBB(TBB)
    .addOperand(VInstrInfo::CreatePredicate())
    .addOperand(VInstrInfo::CreateTrace());
  // Branch to the false BB.
  ReversePredicateCondition(PredOp);
  BuildMI(&MBB, DL, get(VTM::VOpToStateb)).addOperand(PredOp).addMBB(FBB)
    .addOperand(VInstrInfo::CreatePredicate())
    .addOperand(VInstrInfo::CreateTrace());
   return 2;
}

void VInstrInfo::insertJumpTable(MachineBasicBlock &BB, JT &Table, DebugLoc dl){
  assert(BB.getFirstTerminator() == BB.end() && "Cannot insert jump table!");

  // Dirty hack: We may not evaluate the predicate to always true at the moment.
  if (Table.size() == 1) {
    BuildMI(&BB, dl, getDesc(VTM::VOpToStateb))
      .addOperand(VInstrInfo::CreatePredicate()).addMBB(*BB.succ_begin())
      .addOperand(VInstrInfo::CreatePredicate())
      .addOperand(VInstrInfo::CreateTrace());
    return;
  }

  // Insert the return operation
  JT::iterator RetPredAt = Table.find(0);
  if (RetPredAt != Table.end()) {
    BuildMI(&BB, dl, getDesc(VTM::VOpRet))
      .addOperand(RetPredAt->second).addOperand(VInstrInfo::CreateTrace());
    Table.erase(RetPredAt);
  }

  assert(Table.size() == BB.succ_size()&&"Table size and succ_size not match!");
  for (JT::iterator I = Table.begin(), E = Table.end(); I != E; ++I) {
    I->second.setIsKill(false);
    BuildMI(&BB, dl, getDesc(VTM::VOpToStateb))
      .addOperand(I->second).addMBB(I->first)
      .addOperand(VInstrInfo::CreatePredicate())
      .addOperand(VInstrInfo::CreateTrace());
  }
}

bool VInstrInfo::DefinesPredicate(MachineInstr *MI,
                                  std::vector<MachineOperand> &Pred) const {
  //MachineRegisterInfo &MRI = MI->getParent()->getParent()->getRegInfo();
  //for (unsigned i = 0, e = MI->getNumOperands(); i < e; ++i) {
  //  MachineOperand &MO =MI->getOperand(i);
  //  if (!MO.isReg() || !MO.isDef()) continue;

  //  if (MRI.getRegClass(MO.getReg()) == VTM::PredRRegClass)
  //    Pred.push_back(MO);
  //}

  //return !Pred.empty();
  return false;
}

bool VInstrInfo::isProfitableToIfCvt(MachineBasicBlock &TMBB,
                                     unsigned NumTCycles,
                                     unsigned ExtraTCycles,
                                     MachineBasicBlock &FMBB,
                                     unsigned NumFCycles,
                                     unsigned ExtraFCycles,
                                     float Probability, float Confidence) const{
  return true; // DirtyHack: Everything is profitable.
}

bool VInstrInfo::isProfitableToIfCvt(MachineBasicBlock &MBB, unsigned NumCyles,
                                     unsigned ExtraPredCycles,
                                     float Probability, float Confidence) const{
  return true;
}

bool VInstrInfo::isProfitableToDupForIfCvt(MachineBasicBlock &MBB,
                                           unsigned NumCyles,
                                           float Probability,
                                           float Confidence) const {
  return false; // DirtyHack: Duplicate the BB will break SSA from.
}

MachineInstr *VInstrInfo::PredicatePseudoInstruction(MachineInstr *MI,
                                                     const SmallVectorImpl<MachineOperand> &Pred) {

  // Implicit define do not need to predicate at all.
  if (MI->isImplicitDef()) return MI;

  if (MI->getOpcode() != VTM::COPY) return 0;

  if (MI->isImplicitDef()) return MI;

  SmallVector<MachineOperand, 2> Ops;
  while (MI->getNumOperands()) {
    unsigned LastOp = MI->getNumOperands() - 1;
    Ops.push_back(MI->getOperand(LastOp));
    MI->RemoveOperand(LastOp);
  }

  MachineBasicBlock::iterator InsertPos = MI;
  InsertPos = VInstrInfo::BuildConditionnalMove(*MI->getParent(), InsertPos,
                                                Ops[1], Pred, Ops[0]);
  MI->eraseFromParent();

  return InsertPos;
}

bool VInstrInfo::PredicateInstruction(MachineInstr *MI,
                                    const SmallVectorImpl<MachineOperand> &Pred)
                                    const {
  assert(Pred.size() == 1 && "Too much conditions!");
  // Can only have 1 predicate at the moment.
  if (!isPredicable(MI) || isPredicated(MI)) return false;

  // Can we get the predicate operand?
  MachineOperand *PredOp = getPredOperand(MI);
  if (PredOp == 0) return false;

  const MachineOperand &NewPred = Pred[0];
  PredOp->setReg(NewPred.getReg());
  PredOp->setTargetFlags(NewPred.getTargetFlags());
  return true;
}

void VInstrInfo::MergeBranches(MachineBasicBlock *PredFBB,
                               SmallVectorImpl<MachineOperand> &Pred,
                               MachineBasicBlock *&CndTBB,
                               MachineBasicBlock *&CndFBB,
                               SmallVectorImpl<MachineOperand> &Cnd,
                               const TargetInstrInfo *TII) {
  assert(Pred.size() <= 1 && "Too many predicate operand!");
  // Nothing to merge, became:
  // br Cnd, CndTBB, CndFBB.
  if (PredFBB == 0) {
    assert(Pred.empty() && "Expected unconditional branch to PredTBB!");
  } else if (Cnd.empty()) {
    // Unconditional branch to CndTBB, becomes:
    // br Pred, CndTBB, PredFBB.
    assert(CndFBB == 0 && "Expected unconditional branch!");
    Cnd.push_back(Pred.front());
    CndFBB = PredFBB;
  } else {
    // Now we have:
    // if (!Pred) br PredFBB,
    // else if (Pred & Cnd) br CndTBB
    // else if (Pred & !Cnd) br CndFBB
    // But PredFBB may equals to CndTBB, otherwise PredFBB equals to CndFBB.
    // Make sure PredFBB == CndFBB so we have:
    // br (Pred & Cnd), CndTBB, CndFBB(PredFBB)
    // Because (Pred & !Cnd) | (!Pred) = !(Pred & Cnd)
    if (PredFBB != CndFBB) {
      TII->ReverseBranchCondition(Cnd);
      std::swap(CndTBB, CndFBB);
    }
    assert(PredFBB == CndFBB && "We have 3 difference BB?");

    Cnd.push_back(Pred.front());
  }

  // Always branch to the same BB.
  if (CndTBB == CndFBB) {
    Cnd.clear();
    CndFBB = 0;
  }
}

bool VInstrInfo::isAlwaysTruePred(const MachineOperand &MO){
  assert(MO.isReg() && "Unexpected MO type!");
  if (MO.getReg() == 0) {
    assert(!VInstrInfo::isPredicateInverted(MO) && "Illegal always false!");
    return true;
  }

  return false;
}

MachineInstr *VInstrInfo::getBundleHead(MachineInstr *MI) {
  assert(MI->isBundled() && "Not a bundle!");
  MachineInstr *Head = getBundleStart(MI);
  assert((Head->getOpcode() == VTM::CtrlStart ||
    Head->getOpcode() == VTM::Datapath) && "Broken bundle found!");
  return Head;
}

bool VInstrInfo::isCtrlBundle(MachineInstr *MI) {
  return getBundleHead(MI)->getOpcode() == VTM::CtrlStart;
}

bool VInstrInfo::isDatapathBundle(MachineInstr *MI) {
  return getBundleHead(MI)->getOpcode() == VTM::Datapath;
}

MachineBasicBlock::instr_iterator VInstrInfo::getCtrlBundleEnd(MachineInstr *MI){
  assert(MI->getOpcode() == VTM::CtrlStart && "Bad MI!");
  MachineBasicBlock::instr_iterator I = MI;
  do {
    ++I;
    assert(I != I->getParent()->instr_end() && "Broken bundle found!");
  } while (I->getOpcode() != VTM::CtrlEnd);

  return I;
}

MachineOperand VInstrInfo::CreatePredicate(unsigned Reg) {
  // Read reg0 means always execute.
  MachineOperand MO = MachineOperand::CreateReg(Reg, false);
  VInstrInfo::setBitWidth(MO, 1);
  return MO;
}

MachineOperand VInstrInfo::CreateTrace() {
  MachineOperand MO = MachineOperand::CreateImm(0);
  MO.setTargetFlags(4);
  return MO;
}

bool VInstrInfo::isPredicateMutex(const MachineInstr *LHS,
                                  const MachineInstr *RHS) {
  const MachineOperand *LHSPred = VInstrInfo::getPredOperand(LHS);
  const MachineOperand *RHSPred = VInstrInfo::getPredOperand(RHS);

  // The same predicate, or there is always true predicate,
  // not mutual exclusive.
  if (LHSPred == 0 || RHSPred == 0 ||
      VInstrInfo::isAlwaysTruePred(*LHSPred) ||
      VInstrInfo::isAlwaysTruePred(*RHSPred))
    return false;

  // Compare the trace,
  uint8_t LHSBBNum = LHSPred[1].getTargetFlags(),
          RHSBBNum = RHSPred[1].getTargetFlags();

  // Inside the same local BB.
  if (LHSBBNum == RHSBBNum) return false;

  int64_t LHSPredBits = LHSPred[1].getImm(), RHSPredBits = RHSPred[1].getImm();

  // If the two predicates are mutual exclusive if they cannot reach each other.
  return !(LHSPredBits & (UINT64_C(1) << RHSBBNum))
         && !(RHSPredBits & (UINT64_C(1) << LHSBBNum));
}

MachineOperand VInstrInfo::CreateReg(unsigned RegNum, unsigned BitWidth,
                                     bool IsDef /* = false */) {
  MachineOperand MO = MachineOperand::CreateReg(RegNum, IsDef);
  VInstrInfo::setBitWidth(MO, BitWidth);
  return MO;
}

MachineOperand VInstrInfo::CreateImm(int64_t Val, unsigned BitWidth) {
  MachineOperand MO = MachineOperand::CreateImm(Val);
  VInstrInfo::setBitWidth(MO, BitWidth);
  return MO;
}

bool VInstrInfo::isAllOnes(const MachineOperand &MO) {
  return MO.isImm() && isAllOnes64(MO.getImm(), getBitWidth(MO));
}

bool VInstrInfo::isAllZeros(const MachineOperand &MO) {
  return MO.isImm() && isAllZeros64(MO.getImm(), getBitWidth(MO));
}

static uint64_t getMachineOperandHashValue(const MachineOperand &MO) {
  switch (MO.getType()) {
  default: break;
  case MachineOperand::MO_Register:
    if (MO.isDef() && TargetRegisterInfo::isVirtualRegister(MO.getReg()))
      return 0;  // Skip virtual register defs.
    return hash_combine(MO.getType(), MO.getReg(), MO.getTargetFlags());
  case MachineOperand::MO_Immediate:
    return hash_combine(MO.getType(), MO.getImm(), MO.getTargetFlags());
  case MachineOperand::MO_FrameIndex:
  case MachineOperand::MO_ConstantPoolIndex:
  case MachineOperand::MO_JumpTableIndex:
    return hash_combine(MO.getType(), MO.getIndex(), MO.getTargetFlags());
  case MachineOperand::MO_MachineBasicBlock:
    return hash_combine(MO.getType(), MO.getMBB(), MO.getTargetFlags());
  case MachineOperand::MO_GlobalAddress:
    return hash_combine(MO.getType(), MO.getGlobal(), MO.getTargetFlags());
  case MachineOperand::MO_BlockAddress:
    return hash_combine(MO.getType(), MO.getBlockAddress(), MO.getTargetFlags());
  case MachineOperand::MO_MCSymbol:
    return hash_combine(MO.getType(), MO.getMCSymbol(), MO.getTargetFlags());
  case MachineOperand::MO_ExternalSymbol:
    return hash_combine(MO.getType(), MO.getSymbolName(), MO.getTargetFlags());
  }

  llvm_unreachable("Bad machine operand type!");
  return 0;
}

unsigned VMachineOperandValueTrait::getHashValue(MachineOperand Op) {
  return getMachineOperandHashValue(Op);
}

unsigned
VMachineInstrExpressionTrait::getHashValue(const MachineInstr* const &MI) {
  // Build up a buffer of hash code components.
  //
  // FIXME: This is a total hack. We should have a hash_value overload for
  // MachineOperand, but currently that doesn't work because there are many
  // different ideas of "equality" and thus different sets of information that
  // contribute to the hash code. This one happens to want to take a specific
  // subset. And it's still not clear that this routine uses the *correct*
  // subset of information when computing the hash code. The goal is to use the
  // same inputs for the hash code here that MachineInstr::isIdenticalTo uses to
  // test for equality when passed the 'IgnoreVRegDefs' filter flag. It would
  // be very useful to factor the selection of relevant inputs out of the two
  // functions and into a common routine, but it's not clear how that can be
  // done.
  SmallVector<size_t, 8> HashComponents;
  HashComponents.reserve(MI->getNumOperands() + 1);
  HashComponents.push_back(MI->getOpcode());

  const MCInstrDesc &TID = MI->getDesc();

  for (unsigned i = 0, e = MI->getNumOperands(); i != e; ++i) {
    const MachineOperand &MO = MI->getOperand(i);
    // Ignore the virtual register definition.
    if (MO.isReg() && MO.isDef()
        && TargetRegisterInfo::isVirtualRegister(MO.getReg()))
      continue;

    // Ignore the SlotNumber.
    if (i < TID.getNumOperands() && TID.OpInfo[i].isPredicate() && MO.isImm())
      continue;

    HashComponents.push_back(getMachineOperandHashValue(MO));
  }
  return hash_combine_range(HashComponents.begin(), HashComponents.end());
}

static MachineOperand RemoveInvertFlag(MachineOperand MO, MachineRegisterInfo *MRI,
                                       MachineBasicBlock &MBB,
                                       MachineBasicBlock::iterator IP,
                                       const TargetInstrInfo *TII) {
  assert(!VInstrInfo::isAlwaysTruePred(MO) && "Unexpected always false!");
  MachineOperand Op(MO);

  if (VInstrInfo::isPredicateInverted(MO)) {
    Op.clearParent();
    // Remove the invert flag.
    VInstrInfo::setBitWidth(Op, 1);
    // Build the not instruction.
    unsigned DstReg = MRI->createVirtualRegister(&VTM::DRRegClass);
    MachineOperand Dst = MachineOperand::CreateReg(DstReg, true);
    VInstrInfo::setBitWidth(Dst, 1);
    BuildMI(MBB, IP, DebugLoc(), TII->get(VTM::VOpNot))
      .addOperand(Dst).addOperand(Op)
      .addOperand(VInstrInfo::CreatePredicate())
      .addOperand(VInstrInfo::CreateTrace());
    Dst.setIsDef(false);
    return Dst;
  }

  return Op;
}

MachineOperand VInstrInfo::MergePred(MachineOperand OldCnd,
                                     MachineOperand NewCnd,
                                     MachineBasicBlock &MBB,
                                     MachineBasicBlock::iterator IP,
                                     MachineRegisterInfo *MRI,
                                     const TargetInstrInfo *TII,
                                     unsigned MergeOpC) {
  // Perform simple optimization: A & 1 => A
  if (isAlwaysTruePred(OldCnd)) return NewCnd;
  if (isAlwaysTruePred(NewCnd)) return OldCnd;

  OldCnd.clearParent();
  MRI->clearKillFlags(OldCnd.getReg());
  OldCnd = RemoveInvertFlag(OldCnd, MRI, MBB, IP, TII);

  NewCnd.clearParent();
  MRI->clearKillFlags(NewCnd.getReg());
  NewCnd = RemoveInvertFlag(NewCnd, MRI, MBB, IP, TII);

  unsigned DstReg = MRI->createVirtualRegister(&VTM::DRRegClass);
  MachineOperand Dst = MachineOperand::CreateReg(DstReg, true);
  VInstrInfo::setBitWidth(Dst, 1);

  BuildMI(MBB, IP, DebugLoc(), TII->get(MergeOpC))
    .addOperand(Dst).addOperand(NewCnd).addOperand(OldCnd)
    .addOperand(VInstrInfo::CreatePredicate())
    .addOperand(VInstrInfo::CreateTrace());
  Dst.setIsDef(false);
  return Dst;
}


MachineInstr &
VInstrInfo::BuildConditionnalMove(MachineBasicBlock &MBB,
                                  MachineBasicBlock::iterator IP,
                                  MachineOperand &Res,
                                  const SmallVectorImpl<MachineOperand> &Pred,
                                  MachineOperand IfTrueVal) {
  if (!Res.getReg()) {
    MachineRegisterInfo &MRI = MBB.getParent()->getRegInfo();
    const TargetRegisterClass *RC = MRI.getRegClass(IfTrueVal.getReg());
    Res.setReg(MRI.createVirtualRegister(RC));
  }

  MachineOperand ResDef(Res);
  ResDef.setIsDef();

  return *BuildMI(MBB, IP, DebugLoc(), getDesc(VTM::VOpMove))
            .addOperand(ResDef).addOperand(IfTrueVal).addOperand(Pred[0])
            .addOperand(VInstrInfo::CreateTrace());
}

MachineInstr *VInstrInfo::getEdgeCndAndInsertPos(MachineBasicBlock *From,
                                                 MachineBasicBlock *To,
                                                 MachineOperand &Pred) {
  VInstrInfo::JT SrcJT;
  bool success = !VInstrInfo::extractJumpTable(*From, SrcJT, false);
  assert(success && "Broken machine code?");
  // TODO: Handle critical edges.

  // Insert the PHI copy.
  VInstrInfo::JT::iterator at = SrcJT.find(To);
  assert(at != SrcJT.end() && "Broken CFG?");
  Pred = at->second;
  return From->getFirstInstrTerminator();
}

MachineInstr *VInstrInfo::getInsertPosBeforTerminator(MachineBasicBlock *MBB) {
  MachineBasicBlock::iterator IP = MBB->getFirstTerminator();
  // Insert the imp_def before the PHI incoming copies.
  while (IP != MBB->begin() && llvm::prior(IP)->getOpcode() == VTM::VOpMvPhi)
    --IP;

  return IP;
}

bool VInstrInfo::isCopyLike(unsigned Opcode) {
  return Opcode == VTM::COPY
         || Opcode == VTM::PHI
         || Opcode == VTM::VOpMove
         || Opcode == VTM::VOpMoveArg
         || Opcode == VTM::VOpDstMux
         || Opcode == VTM::VOpReadReturn
         || Opcode == VTM::VOpReadFU
         || Opcode == VTM::VOpMvPhi
         || Opcode == VTM::VOpDefPhi;
}

bool VInstrInfo::isBrCndLike(unsigned Opcode) {
  return Opcode == VTM::VOpToState
         || Opcode == VTM::VOpToStateb
         || Opcode == VTM::VOpToState_nt;
}

bool VInstrInfo::isWriteUntilFinish(unsigned OpC) {
  const MCInstrDesc &TID = getDesc(OpC);
  return (TID.TSFlags & (WriteUntilFinishMask << WriteUntilFinishShiftAmount))
         || VInstrInfo::isCopyLike(OpC);
}

bool VInstrInfo::isDatapath(unsigned OpC) {
  // All pseudo instructions are control operations.
  return //OpC > TargetOpcode::COPY Not need because the bit is clean by default
         getDesc(OpC).TSFlags & (DatapathMask << DatapathShiftAmount);
}

VFUs::FUTypes VInstrInfo::getFUType(unsigned OpC) {
  return (VFUs::FUTypes)
    ((getDesc(OpC).TSFlags >> ResTypeShiftAmount) & ResTypeMask);
}

bool VInstrInfo::isReadAtEmit(unsigned OpC) {
  return (getDesc(OpC).TSFlags & (ReadAtEmitMask << ReadAtEmitShiftAmount))
         || isCopyLike(OpC);
}

template<int Idx, class FUClass>
static float LookupLatency(const MachineInstr *MI){
  unsigned SizeInBits = VInstrInfo::getBitWidth(MI->getOperand(Idx));

  return getFUDesc<FUClass>()->lookupLatency(SizeInBits);
}

FuncUnitId VInstrInfo::getPreboundFUId(const MachineInstr *MI) {
  // Dirty Hack: Bind all memory access to channel 0 at this moment.
  switch(MI->getOpcode()) {
  case VTM::VOpDisableFU:
    return FuncUnitId(uint16_t(MI->getOperand(1).getImm()));
  case VTM::VOpReadFU:
    return FuncUnitId(uint16_t(MI->getOperand(2).getImm()));
  case VTM::VOpMemTrans:
    return FuncUnitId(VFUs::MemoryBus, 0);
  case VTM::VOpBRAMTrans: {
    unsigned Id = MI->getOperand(5).getImm();
    return FuncUnitId(VFUs::BRam, Id);
  }
  case VTM::VOpInternalCall: {
    unsigned Id = MI->getOperand(1).getTargetFlags();
    return FuncUnitId(VFUs::CalleeFN, Id);
  }
  case VTM::VOpDstMux: {
    unsigned Id = MI->getOperand(2).getImm();
    return FuncUnitId(VFUs::Mux, Id);
  }
  default:
    return FuncUnitId();
  }
}

bool VInstrInfo::mayLoad(const MachineInstr *MI) {
  switch (MI->getOpcode()) {
  default: return false;
    // There is a "isLoad" flag in memory access operation.
  case VTM::VOpMemTrans:
  case VTM::VOpBRAMTrans: return !MI->getOperand(3).getImm();
  }
}

bool VInstrInfo::mayStore(const MachineInstr *MI) {
  switch (MI->getOpcode()) {
  default: return false;
    // There is a "isLoad" flag in memory access operation.
  case VTM::VOpMemTrans:
  case VTM::VOpBRAMTrans:  return MI->getOperand(3).getImm();
  }
}

BitWidthAnnotator::BitWidthAnnotator(MachineInstr &MI)
  : MO(&MI.getOperand(MI.getNumOperands() - 2)) {
  assert(hasBitWidthInfo() && "Bitwidth not available!");
  BitWidths = MO->getImm();
}

void BitWidthAnnotator::updateBitWidth() {
  assert(MO && hasBitWidthInfo() && "Cannot update bit width!");
  MO->setImm(BitWidths);
}

bool BitWidthAnnotator::hasBitWidthInfo() const {
  assert(MO && "MachineOperand not available!");
  return MO->isImm();
}

void BitWidthAnnotator::changeToDefaultPred() {
  MachineInstr *Parent = MO->getParent();
  unsigned MOIdx = MO - &Parent->getOperand(0);
  unsigned PredIdx = Parent->getDesc().NumOperands - 2;
  if (MOIdx != PredIdx) {
    MachineOperand PredMO[2] = {*(MO + 1), *MO};
    Parent->RemoveOperand(MOIdx);
    Parent->RemoveOperand(MOIdx);

    SmallVector<MachineOperand, 8> Ops;
    while(--MOIdx != 0) {
      Ops.push_back(Parent->getOperand(MOIdx));
      Parent->RemoveOperand(MOIdx);
    }

    // Insert the predicate operand to the operand list.
    Ops.insert(Ops.begin() + (Ops.size() - PredIdx) + 1,
               PredMO, array_endof(PredMO));

    while (!Ops.empty())
      Parent->addOperand(Ops.pop_back_val());
    MO = &Parent->getOperand(PredIdx);
  }

  MO->ChangeToRegister(0, false);
  MO->setTargetFlags(1);
}

template<unsigned IdxStart, unsigned IdxEnd>
static unsigned countNumRegOperands(const MachineInstr *MI) {
  unsigned NumRegs = 0;
  for (unsigned i = IdxStart; i < IdxEnd; ++i)
    if (MI->getOperand(i).isReg() && MI->getOperand(i).getReg())
      ++NumRegs;

  return NumRegs;
}

unsigned VInstrInfo::countNumRegUses(const MachineInstr *MI) {
  unsigned NumRegs = 0;
  for (unsigned i = 0, e = MI->getNumOperands(); i < e; ++i)
    if (MI->getOperand(i).isReg() && MI->getOperand(i).getReg() &&
        !MI->getOperand(i).isDef())
      ++NumRegs;

  return NumRegs;
}

// Get the latency of a machineinstr in cycle ratio.
float VInstrInfo::getDetialLatency(const MachineInstr *MI) {
  unsigned OpC = MI->getOpcode();

  switch (OpC) {
    // TODO: Bitrepeat.
  case VTM::VOpICmp_c:
  case VTM::VOpICmp:
    return LookupLatency<1, VFUICmp>(MI);
  // Retrieve the FU bit width from its operand bit width
  case VTM::VOpAdd_c:
  case VTM::VOpAdd:
    return LookupLatency<1, VFUAddSub>(MI);

  case VTM::VOpMultLoHi_c:
  case VTM::VOpMult_c:
  case VTM::VOpMultLoHi:
  case VTM::VOpMult:
    return LookupLatency<0, VFUMult>(MI);

  case VTM::VOpSRA_c:
  case VTM::VOpSRL_c:
  case VTM::VOpSHL_c:
  case VTM::VOpSRA:
  case VTM::VOpSRL:
  case VTM::VOpSHL:
    return LookupLatency<0, VFUShift>(MI);

  case VTM::VOpMemTrans:    return getFUDesc<VFUMemBus>()->getLatency();

  // Can be fitted into LUT.
  case VTM::VOpSel:
    return LookupLatency<0, VFUSel>(MI);

  // Ignore the trivial logic operation latency at the moment.
  case VTM::VOpLUT:
  case VTM::VOpAnd:
  case VTM::VOpOr:
  case VTM::VOpXor:
  case VTM::VOpNot:         return VFUs::LutLatency;

  case VTM::VOpRXor:
  case VTM::VOpROr:
  case VTM::VOpRAnd:
    return LookupLatency<1, VFUReduction>(MI);

  case VTM::VOpBRAMTrans:   return getFUDesc<VFUBRAM>()->getLatency();

  case VTM::VOpInternalCall:  return 1.0f;

  default:                  break;
  }

  return 0.0f;
}


