//===---------- VInstrInfo.cpp - VTM Instruction Information -----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
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

#include "vtm/VFInfo.h"
#include "vtm/VInstrInfo.h"
#include "vtm/VTM.h"
#include "vtm/MicroState.h"

#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/Debug.h"

#include "VGenInstrInfo.inc"
using namespace llvm;

static const unsigned MoveOpcodes[] = {
    0, //DRRegClassID = 0,
    0, //PHIRRegClassID = 1,
    VTM::VOpMove_ra, //RADDRegClassID = 2,
    VTM::VOpMove_rm, //RMULRegClassID = 3,
    VTM::VOpMove_rs, //RSHTRegClassID = 4,
    VTM::VOpMove_rw, //WireRegClassID = 5
};

const MachineOperand *VInstrInfo::getPredOperand(const MachineInstr *MI) {
  if (MI->getOpcode() <= VTM::COPY) return 0;

  unsigned Idx = MI->getDesc().NumOperands - 1;
  assert(MI->getDesc().OpInfo[Idx].isPredicate() && "Cannot get PredOperand!");
  return &MI->getOperand(Idx);
}

MachineOperand *VInstrInfo::getPredOperand(MachineInstr *MI) {
  return const_cast<MachineOperand*>(getPredOperand((const MachineInstr*)MI));
}

VInstrInfo::VInstrInfo(const TargetData &TD, const TargetLowering &TLI)
  : TargetInstrInfoImpl(VTMInsts, array_lengthof(VTMInsts)), RI(*this, TD, TLI){
}


bool VInstrInfo::isReallyTriviallyReMaterializable(const MachineInstr *MI,
                                                   AliasAnalysis *AA) const {
  VIDesc Desc(MI->getDesc());
  return !Desc->isBarrier() && Desc.hasTrivialFU();
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

bool VInstrInfo::isUnpredicatedTerminator(const MachineInstr *MI) const{
  const TargetInstrDesc &TID = MI->getDesc();
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
    }
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
  FBB = SndTerm->getOperand(1).getMBB();
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
    if (!Inst->getDesc().isTerminator()) continue;

    if (VInstrInfo::isBrCndLike(Inst->getOpcode()))
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
                          ucOperand::CreatePredicate() : Cond[0];
  PredOp.setIsKill(false);

  if (FBB == 0) {
    // Insert barrier branch for unconditional branch.
    unsigned Opc = isUnconditional ? VTM::VOpToStateb : VTM::VOpToState;
    BuildMI(&MBB, DL, get(Opc)).addOperand(PredOp).addMBB(TBB)
      .addOperand(ucOperand::CreatePredicate());
    return 1;
  }

  // Two-way conditional branch.
  assert(PredOp.isReg() && PredOp.getReg() != 0
         && "Uncondtional predicate with true BB and false BB?");
  // Branch to true BB, with the no-barrier version.
  BuildMI(&MBB, DL, get(VTM::VOpToState)).addOperand(PredOp).addMBB(TBB)
    .addOperand(ucOperand::CreatePredicate());
  // Branch to the false BB.
  ReversePredicateCondition(PredOp);
  BuildMI(&MBB, DL, get(VTM::VOpToStateb)).addOperand(PredOp).addMBB(FBB)
    .addOperand(ucOperand::CreatePredicate());
   return 2;
}

bool VInstrInfo::DefinesPredicate(MachineInstr *MI,
                                  std::vector<MachineOperand> &Pred) const {
  //MachineRegisterInfo &MRI = MI->getParent()->getParent()->getRegInfo();
  //for (unsigned i = 0, e = MI->getNumOperands(); i < e; ++i) {
  //  MachineOperand &MO =MI->getOperand(i);
  //  if (!MO.isReg() || !MO.isDef()) continue;

  //  if (MRI.getRegClass(MO.getReg()) == VTM::PredRRegisterClass)
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
  return true; // DirtyHack: Everything is profitable.
}

MachineInstr *VInstrInfo::PredicatePseudoInstruction(MachineInstr *MI,
                                                     const TargetInstrInfo *TII,
                                                     const SmallVectorImpl<MachineOperand> &Pred) {
  if (MI->getOpcode() != VTM::COPY) return 0;

  SmallVector<MachineOperand, 2> Ops;
  while (MI->getNumOperands()) {
    unsigned LastOp = MI->getNumOperands() - 1;
    Ops.push_back(MI->getOperand(LastOp));
    MI->RemoveOperand(LastOp);
  }

  MachineBasicBlock::iterator InsertPos = MI;
  InsertPos = VInstrInfo::BuildConditionnalMove(*MI->getParent(), InsertPos,
                                                Ops[1], Pred, Ops[0], TII);
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

unsigned VInstrInfo::createPHIIncomingReg(unsigned DestReg,
                                          MachineRegisterInfo *MRI) const {
  const TargetRegisterClass *PHIRC = VTM::PHIRRegisterClass;
  return MRI->createVirtualRegister(PHIRC);
}

typedef MachineBasicBlock::iterator mbb_it;

MachineInstr *VInstrInfo::insertPHIImpDef(MachineBasicBlock &MBB,
                                          mbb_it InsertPos,
                                          MachineInstr *PN) const {
  return TargetInstrInfo::insertPHIImpDef(MBB, InsertPos, PN);
}

MachineInstr *VInstrInfo::insertPHIIcomingCopy(MachineBasicBlock &MBB,
                                               mbb_it InsertPos,
                                               MachineInstr *PN,
                                               unsigned IncomingReg) const {
  ucOperand &DefOp = cast<ucOperand>(PN->getOperand(0));
  ucState Ctrl(InsertPos);
  assert(Ctrl->getOpcode() == VTM::Control && "Unexpected instruction type!");
  // Simply build the copy in the first control slot.
  MachineInstrBuilder Builder(InsertPos);
  Builder.addOperand(ucOperand::CreateOpcode(VTM::VOpDefPhi, Ctrl.getSlot()));
  Builder.addOperand(ucOperand::CreatePredicate());
  ucOperand Dst = MachineOperand::CreateReg(DefOp.getReg(), true);
  Dst.setBitWidth(DefOp.getBitWidth());
  Builder.addOperand(Dst);
  ucOperand Src = MachineOperand::CreateReg(IncomingReg, false);
  Src.setBitWidth(DefOp.getBitWidth());
  if (VRegisterInfo::IsWire(IncomingReg, &MBB.getParent()->getRegInfo()))
    Src.setIsWire();
  Builder.addOperand(Src);
  return &*Builder;
}

static MachineInstr *addOperandsToMI(MachineInstr *MI,
                                     SmallVectorImpl<MachineOperand> &Ops) {
  for (SmallVectorImpl<MachineOperand>::iterator I = Ops.begin(), E = Ops.end();
    I != E; ++I)
    MI->addOperand(*I);

  return MI;
}

MachineInstr *VInstrInfo::insertPHICopySrc(MachineBasicBlock &MBB,
                                           mbb_it InsertPos, MachineInstr *PN,
                                           unsigned IncomingReg,
                                           unsigned SrcReg, unsigned SrcSubReg)
                                           const {
  ucOperand &DefOp = cast<ucOperand>(PN->getOperand(0));
  // Get the last slot.
  while ((--InsertPos)->getOpcode() == VTM::IMPLICIT_DEF)
    ;

  VFInfo *VFI = MBB.getParent()->getInfo<VFInfo>();
  unsigned Slot = VFI->lookupPHISlot(PN);
  unsigned StartSlot = VFI->getStartSlotFor(&MBB);
  unsigned EndSlot = VFI->getEndSlotFor(&MBB);
  // If the phi scheduled into this MBB, insert the copy to the right control
  // slot.
  //if (Slot > StartSlot && Slot <= EndSlot) {
  if (Slot <= StartSlot || Slot > EndSlot)
    // Else we are issuing the copy at the end of the BB.
    Slot = EndSlot;

  unsigned II = VFI->getIIFor(&MBB);
  unsigned ModuloSlot = (Slot - StartSlot) % II + StartSlot;
  // If modulo slot is 0, insert the copy in the last control slot.
  // Otherwise, iterate over the BB to find the match slot.
  if (ModuloSlot != StartSlot) {
    while(ucState(InsertPos).getSlot() != ModuloSlot) {
      --InsertPos; // Skip the current control slot.
      --InsertPos; // Skip the current datapath slot.
    }
  }

  ucState Ctrl(InsertPos);
  assert(Ctrl->getOpcode() == VTM::Control && "Unexpected instruction type!");
  SmallVector<MachineOperand, 8> Ops;
  MachineRegisterInfo &MRI = MBB.getParent()->getRegInfo();

  Ops.push_back(ucOperand::CreateOpcode(VTM::VOpMvPhi, Slot));
  Ops.push_back(ucOperand::CreatePredicate());
  MachineOperand &Pred = Ops.back();
  ucOperand Dst = MachineOperand::CreateReg(IncomingReg, true);
  Dst.setBitWidth(DefOp.getBitWidth());
  if (VRegisterInfo::IsWire(DefOp.getReg(), &MRI))
    Dst.setIsWire();
  Ops.push_back(Dst);

  MachineRegisterInfo::def_iterator DI = MRI.def_begin(SrcReg);

  DEBUG(dbgs() << "Copying " << TargetRegisterInfo::virtReg2Index(SrcReg)
               << " in\n";
    ucState(*DI).dump();
  );

  ucOperand Src = MachineOperand::CreateReg(SrcReg, false);
  Src.setSubReg(SrcSubReg);
  Src.setBitWidth(DefOp.getBitWidth());
  if (VRegisterInfo::IsWire(SrcReg, &MRI))
    Src.setIsWire();

  // Try to forward the source value.
  if (DI->getOpcode() != VTM::PHI) {
    assert (++MRI.def_begin(SrcReg) == MRI.def_end() && "Not in SSA From!");
    ucOp WriteOp = ucOp::getParent(DI);
    // We need to forward the wire copy if the source register is written
    // in the same slot, otherwise we will read a out of date value.
    if (WriteOp->getParent() == &*Ctrl && WriteOp->getPredSlot() == Slot) {
      // Copy the predicate operand.
      MachineOperand &WritePred = WriteOp.getPredicate();
      Pred.ChangeToRegister(WritePred.getReg(), false);
      Pred.setTargetFlags(WritePred.getTargetFlags());

      // FIXME: Handle others case.
      if (isCopyLike(WriteOp->getOpcode())) {
        MachineOperand ForwardedVal = WriteOp.getOperand(1);
        Ops.push_back(ForwardedVal);
        return addOperandsToMI(InsertPos, Ops);
      }
    }

    Ops.push_back(Src);
    // Try to forward the imp-uses.
    if (Src.isWire())
      for (ucOp::op_iterator I = WriteOp.op_begin(), E = WriteOp.op_end(); I != E;
        ++I) {
          MachineOperand &Op = *I;
          if (Op.isReg() && Op.isImplicit()) Ops.push_back(Op);
      }

    return addOperandsToMI(InsertPos, Ops);
  }

  // Trivial case, just copy src to dst.
  Ops.push_back(Src);
  return addOperandsToMI(InsertPos, Ops);
}

MachineInstr &VInstrInfo::BuildSelect(MachineBasicBlock *MBB,
                                      MachineOperand &Result,
                                      MachineOperand Pred,
                                      MachineOperand IfTrueVal,
                                      MachineOperand IfFalseVal) {
  // create the result register if necessary.
  if (!Result.getReg()) {
    MachineRegisterInfo &MRI = MBB->getParent()->getRegInfo();
    const TargetRegisterClass *RC = MRI.getRegClass(IfTrueVal.getReg());
    assert(MRI.getRegClass(IfFalseVal.getReg()) == RC
      && "Register class dose not match!");
    Result.setReg(MRI.createVirtualRegister(RC));
  }

  MachineOperand ResDef(Result);
  ResDef.setIsDef();

  // Build and insert the select instruction at the end of the BB.
  return *BuildMI(*MBB, MBB->getFirstTerminator(), DebugLoc(),
                  VTMInsts[VTM::VOpSel])
            .addOperand(ResDef).addOperand(Pred)
            .addOperand(IfTrueVal).addOperand(IfFalseVal)
            .addOperand(ucOperand::CreatePredicate());
}

MachineInstr &VInstrInfo::BuildSelect(MachineBasicBlock *MBB, MachineOperand &Result,
                                      const SmallVectorImpl<MachineOperand> &Pred,
                                      MachineOperand IfTrueVal,
                                      MachineOperand IfFalseVal){
  // Build and insert the select instruction at the end of the BB.
  assert(Pred.size() == 1 && "Cannot select value!");
  return BuildSelect(MBB, Result, Pred[0], IfTrueVal, IfFalseVal);
}

MachineInstr &
VInstrInfo::BuildConditionnalMove(MachineBasicBlock &MBB,
                                  MachineBasicBlock::iterator IP,
                                  MachineOperand &Res,
                                  const SmallVectorImpl<MachineOperand> &Pred,
                                  MachineOperand IfTrueVal,
                                  const TargetInstrInfo *TII) {
  MachineRegisterInfo &MRI = MBB.getParent()->getRegInfo();
  if (!Res.getReg()) {
    MachineRegisterInfo &MRI = MBB.getParent()->getRegInfo();
    const TargetRegisterClass *RC = MRI.getRegClass(IfTrueVal.getReg());
    Res.setReg(MRI.createVirtualRegister(RC));
  }

  MachineOperand ResDef(Res);
  ResDef.setIsDef();

  unsigned Opcode = MoveOpcodes[MRI.getRegClass(IfTrueVal.getReg())->getID()];
  assert(Opcode && "Unsupported move!");

  return *BuildMI(MBB, IP, DebugLoc(), TII->get(Opcode))
            .addOperand(ResDef).addOperand(IfTrueVal).addOperand(Pred[0]);
}

// Add Source to PHINode, if PHINod only have 1 source value, replace the PHI by
// a copy, adjust and return true
static bool AddSrcValToPHI(MachineOperand SrcVal, MachineBasicBlock *SrcBB,
  MachineInstr *PN, MachineRegisterInfo &MRI) {
    if (PN->getNumOperands() != 1) {
      PN->addOperand(SrcVal);
      PN->addOperand(MachineOperand::CreateMBB(SrcBB));
      return false;
    }

    // A redundant PHI have only 1 incoming value after SrcVal added.
    MRI.replaceRegWith(PN->getOperand(0).getReg(), SrcVal.getReg());
    PN->eraseFromParent();
    return true;
}

void
VInstrInfo::mergePHISrc(MachineBasicBlock *Succ, MachineBasicBlock *FromBB,
                        MachineBasicBlock *ToBB, MachineRegisterInfo &MRI,
                        const SmallVectorImpl<MachineOperand> &FromBBCnd) {
  SmallVector<std::pair<MachineOperand, MachineBasicBlock*>, 2> SrcVals;
  SmallVector<MachineInstr*, 8> PHIs;

  // Fix up any PHI nodes in the successor.
  for (MachineBasicBlock::iterator MI = Succ->begin(), ME = Succ->end();
    MI != ME && MI->isPHI(); ++MI)
    PHIs.push_back(MI);

  while (!PHIs.empty()) {
    MachineInstr *MI = PHIs.pop_back_val();
    unsigned Idx = 1;
    while (Idx < MI->getNumOperands()) {
      MachineBasicBlock *SrcBB = MI->getOperand(Idx + 1).getMBB();
      if (SrcBB != FromBB && SrcBB != ToBB ) {
        Idx += 2;
        continue;
      }
      // Take the operand away.
      SrcVals.push_back(std::make_pair(MI->getOperand(Idx), SrcBB));
      MI->RemoveOperand(Idx);
      MI->RemoveOperand(Idx);
    }

    // If only 1 value comes from BB, re-add it to the PHI.
    if (SrcVals.size() == 1) {
      AddSrcValToPHI(SrcVals.pop_back_val().first, ToBB, MI, MRI);
      continue;
    }

    assert(SrcVals.size() == 2 && "Too many edges!");

    // Read the same register?
    if (SrcVals[0].first.getReg() == SrcVals[1].first.getReg()) {
      SrcVals.pop_back();
      AddSrcValToPHI(SrcVals.pop_back_val().first, ToBB, MI, MRI);
      continue;
    }

    // Make sure value from FromBB in SrcVals[1].
    if (SrcVals.back().second != FromBB)
      std::swap(SrcVals[0], SrcVals[1]);

    assert(SrcVals.back().second == FromBB
      && "Cannot build select for value!");
    assert(!FromBBCnd.empty()
      && "Do not know how to select without condition!");
    // Merge the value with select instruction.
    MachineOperand Result = MachineOperand::CreateReg(0, false);
    Result.setTargetFlags(MI->getOperand(0).getTargetFlags());
    MachineOperand FromBBIncomingVal = SrcVals.pop_back_val().first;
    MachineOperand ToBBIncomingVal = SrcVals.pop_back_val().first;
    VInstrInfo::BuildSelect(ToBB, Result, FromBBCnd,
                            FromBBIncomingVal, // Value from FromBB
                            ToBBIncomingVal // Value from ToBB
                            );
    AddSrcValToPHI(Result, ToBB, MI, MRI);
  }
}

bool VInstrInfo::isCopyLike(unsigned Opcode, bool IncludeMoveImm) {
  return Opcode == VTM::COPY
         || Opcode == VTM::VOpMove_ra
         || (Opcode == VTM::VOpMove_ri && IncludeMoveImm)
         || Opcode == VTM::VOpMove_rm
         || Opcode == VTM::VOpMove_rs
         || Opcode == VTM::VOpMove_rw;
}

bool VInstrInfo::isBrCndLike(unsigned Opcode) {
  return Opcode == VTM::VOpToState
         || Opcode == VTM::VOpToStateb;
}

bool VInstrInfo::isWireOp(const TargetInstrDesc &TID) {
  VIDesc VID(TID);

  // When a instruction has datapath and only has trivial FU, they will not
  // share the FU with others instructions, and its result will only be written
  // once, this means we can simply put the value to a wire and do not need to
  // register the value.
  return VID->getOpcode() != VTM::VOpReadReturn
         && VID.hasDatapath() && VID.hasTrivialFU();
}

unsigned VInstrInfo::computeLatency(const MachineInstr *SrcInstr,
                                    const MachineInstr *DstInstr) {
  assert(DstInstr && "DstInstr should not be null!");
  if (SrcInstr == DstInstr) return 0;

  if (SrcInstr == 0) {
    //// Set latency of Control operation and entry root to 1, so we can prevent
    //// scheduling control operation to the first slot.
    //// Do not worry about PHI Nodes, their will be eliminated at the register
    //// allocation pass.
    if (DstInstr->getOpcode() == VTM::PHI)
      return 0;

    return 1;
  }

  VIDesc SrcTID = *SrcInstr;
  // Compute the latency correspond to detail slot.
  unsigned latency = SrcTID.getLatency() * 2;

  VIDesc DstTID = *DstInstr;

  if (DstTID.isReadAtEmit()) {
    // If the edge is reg->reg, increase the latency by 1, because the result
    // is ready after the clock edge.
    if (SrcTID.isWriteUntilFinish())
      return latency + 1;

    // Else if the edge is wire->reg, decrease the latency by 1, becaue the
    // result is ready before the clock edge and let the register can
    // capture the result.
    if(latency > 0)
      latency -= 1;

    return latency;
  }

  // When DstInst dose not read at emit, it hopefully a datapath operation.
  // If the Src and Dst have difference slot type, it needs 1 extra slot to
  // adjust the slot type.
  if (DstTID.hasDatapath() && !SrcTID.hasDatapath())
    return latency + 1;

  return latency;
}

FuncUnitId VIDesc::getPrebindFUId()  const {
  // Dirty Hack: Bind all memory access to channel 0 at this moment.
  switch(getTID().Opcode) {
  case VTM::VOpMemTrans:
    return FuncUnitId(VFUs::MemoryBus, 0);
  case VTM::VOpBRam: {
    unsigned Id = get().getOperand(5).getImm();
    return FuncUnitId(VFUs::BRam, Id);
  }
  case VTM::VOpInternalCall: {
    unsigned Id = get().getOperand(1).getTargetFlags();
    return FuncUnitId(VFUs::CalleeFN, Id);
  }
  default:
    return FuncUnitId();
  }
}


BitWidthAnnotator::BitWidthAnnotator(MachineInstr &MI)
  : MO(&MI.getOperand(MI.getNumOperands() - 1)) {
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
  unsigned PredIdx = Parent->getDesc().NumOperands - 1;
  if (MOIdx != PredIdx) {
    MachineOperand PredMO = *MO;
    Parent->RemoveOperand(MOIdx);

    SmallVector<MachineOperand, 8> Ops;
    while(--MOIdx != 0) {
      Ops.push_back(Parent->getOperand(MOIdx));
      Parent->RemoveOperand(MOIdx);
    }

    // Insert the predicate operand to the operand list.
    Ops.insert(Ops.begin() + (Ops.size() - PredIdx) + 1, PredMO);

    while (!Ops.empty())
      Parent->addOperand(Ops.pop_back_val());
    MO = &Parent->getOperand(PredIdx);
  }

  MO->ChangeToRegister(0, false);
  MO->setTargetFlags(1);
}

bool VIDesc::mayLoad() const {
  switch (getTID().Opcode) {
  default: return false;
  // There is a "isLoad" flag in memory access operation.
  case VTM::VOpMemTrans: return !get().getOperand(3).getImm();
  }
}

bool VIDesc::mayStore() const {
  switch (getTID().Opcode) {
  default: return false;
    // There is a "isLoad" flag in memory access operation.
  case VTM::VOpMemTrans: return get().getOperand(3).getImm();
  }
}

bool VIDesc::canCopyBeFused() const {
  const MachineInstr &I = get();
  assert(I.isCopy() && "canCopyBeFused called on the wrong instruction!");
  if (I.getOperand(1).isImm()) return true;

  assert(I.getParent() && "Expected instruction embedded in machine function!");
  const MachineRegisterInfo &MRI = I.getParent()->getParent()->getRegInfo();
  unsigned DstReg = I.getOperand(0).getReg(),
           SrcReg = I.getOperand(1).getReg();

  // Later pass can not eliminate the non-trivial copy, so it should be fused.
  return MRI.getRegClass(DstReg) != MRI.getRegClass(SrcReg);
}

// Out of line virtual function to provide home for the class.
void VIDesc::anchor() {}
