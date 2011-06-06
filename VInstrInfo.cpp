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

#include "VGenInstrInfo.inc"
using namespace llvm;

static const unsigned MoveOpcodes[] = {
    0, //DRRegClassID = 0,
    0, //PHIRRegClassID = 1,
    VTM::VOpMove_rp, //PredRRegClassID = 2,
    VTM::VOpMove_ra, //RADDRegClassID = 3,
    VTM::VOpMove_rm, //RMULRegClassID = 4,
    VTM::VOpMove_rs, //RSHTRegClassID = 5,
    VTM::VOpMove_rw, //WireRegClassID = 6
};

static const MachineOperand *getPredOperand(const MachineInstr *MI) {
  if (MI->getOpcode() <= VTM::COPY) return 0;

  const MachineOperand &MO = MI->getOperand(MI->getNumOperands() - 1);
  return &MO;
}
VInstrInfo::VInstrInfo(const TargetData &TD, const TargetLowering &TLI)
  : TargetInstrInfoImpl(VTMInsts, array_lengthof(VTMInsts)), RI(*this, TD, TLI)
  {}


bool VInstrInfo::isReallyTriviallyReMaterializable(const MachineInstr *MI,
                                                   AliasAnalysis *AA) const {
  VIDesc Desc(MI->getDesc());
  return !Desc->isBarrier() && Desc.hasTrivialFU();
}

bool VInstrInfo::isPredicable(MachineInstr *MI) const {
  return MI->getOpcode() > VTM::COPY;
}

bool VInstrInfo::isPredicated(const MachineInstr *MI) const {
  // Pseudo machine instruction are never predicated.
  if (!isPredicable(const_cast<MachineInstr*>(MI))) return false;

  const MachineOperand *Pred = getPredOperand(MI);
  return (Pred->isReg() && Pred->getReg() != 0);
}

bool VInstrInfo::isUnpredicatedTerminator(const MachineInstr *MI) const{
  const TargetInstrDesc &TID = MI->getDesc();
  if (!TID.isTerminator()) return false;

  return !isPredicated(MI);
}

bool VInstrInfo::AnalyzeBranch(MachineBasicBlock &MBB, MachineBasicBlock *&TBB,
                               MachineBasicBlock *&FBB,
                               SmallVectorImpl<MachineOperand> &Cond,
                               bool AllowModify /* = false */) const {
  // Do not mess with the scheduled code.
  if (MBB.back().getOpcode() == VTM::EndState)
    return true;

  // Just a fall through edge, return false and leaving TBB/FBB null.
  if (MBB.getFirstTerminator() == MBB.end()) return true;

  SmallVector<MachineInstr*, 4> Terms;
  for (MachineBasicBlock::iterator I = MBB.getFirstTerminator(), E = MBB.end();
       I != E; ++I) {
    MachineInstr *Inst = I;
    if (!Inst->getDesc().isTerminator()) continue;

    if (I->getOpcode() == VTM::VOpToState)
      Terms.push_back(Inst);
  }

  // Mixing branches and return?
  if (Terms.size() != MBB.succ_size() || Terms.empty()) return true;

  // So many terminators!
  if (Terms.size() > 2) return true;

  MachineInstr *FstTerm = Terms[0];

  if (isUnpredicatedTerminator(FstTerm)) {
    TBB = FstTerm->getOperand(1).getMBB();
    assert(Terms.size() == 1 && "Expect single fall through edge!");
    return false;
  }

  assert(Terms.size() > 1 && "Unexpected terminator count!");

  TBB = FstTerm->getOperand(1).getMBB();
  MachineInstr *SndTerm = Terms[1];
  FBB = SndTerm->getOperand(1).getMBB();
  Cond.push_back(*getPredOperand(FstTerm));
  // Also add the invert condition operand flag.
  Cond.push_back(MachineOperand::CreateImm(0));
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

      if (I->getOpcode() == VTM::VOpToState)
        Terms.push_back(Inst);
  }

  unsigned RemovedBranches = 0;
  while (!Terms.empty()) {
    Terms.pop_back_val()->removeFromParent();
    ++RemovedBranches;
  }

  return RemovedBranches;
}

bool
VInstrInfo::ReverseBranchCondition(SmallVectorImpl<MachineOperand> &Cond) const {
  assert(Cond.size() == 2 && "Wrong condition count!");
  // Invert the invert condition flag.
  Cond[1] = MachineOperand::CreateImm(!Cond[1].getImm());
  return true;
}

unsigned VInstrInfo::InsertBranch(MachineBasicBlock &MBB,
                                  MachineBasicBlock *TBB,
                                  MachineBasicBlock *FBB,
                                  const SmallVectorImpl<MachineOperand> &Cond,
                                  DebugLoc DL) const {
  assert((Cond.size() == 0 || Cond.size() == 2) && "Too much conditions!");
  MachineOperand PredOp = Cond.empty() ?
                          ucOperand::CreatePredicate() : Cond[0];
  unsigned InvertCnd = Cond.empty() ? 0 : Cond[1].getImm();

  if (FBB == 0) {
    BuildMI(&MBB, DL, get(VTM::VOpToState))
      .addImm(InvertCnd).addMBB(TBB).addOperand(PredOp);
    return 1;
  }

  // Two-way conditional branch.
  assert(PredOp.isReg() && PredOp.getReg() != 0
         && "Uncondtional predicate with true BB and false BB?");
  // Branch to true BB.
  BuildMI(&MBB, DL, get(VTM::VOpToState))
      .addImm(InvertCnd).addMBB(TBB).addOperand(PredOp);
  // Branch the false BB.
  BuildMI(&MBB, DL, get(VTM::VOpToState))
      .addImm(!InvertCnd).addMBB(TBB).addOperand(PredOp);
   return 2;
}

bool VInstrInfo::DefinesPredicate(MachineInstr *MI,
                                  std::vector<MachineOperand> &Pred) const {
  MachineRegisterInfo &MRI = MI->getParent()->getParent()->getRegInfo();
}

bool
VInstrInfo::PredicateInstruction(MachineInstr *MI,
                                 const SmallVectorImpl<MachineOperand> &Pred)
                                 const {
  return false;
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
  Builder.addOperand(Src);
  return &*Builder;
}

MachineInstr *VInstrInfo::insertPHICopySrc(MachineBasicBlock &MBB,
                                           mbb_it InsertPos, MachineInstr *PN,
                                           unsigned IncomingReg,
                                           unsigned SrcReg, unsigned SrcSubReg)
                                           const {
  ucOperand &DefOp = cast<ucOperand>(PN->getOperand(0));
  // Get the last slot.
  InsertPos = prior(InsertPos);

  VFInfo *VFI = MBB.getParent()->getInfo<VFInfo>();
  unsigned Slot = VFI->lookupPHISlot(PN);
  unsigned StartSlot = VFI->getStartSlotFor(&MBB);
  // If the phi scheduled into this MBB, insert the copy to the right control
  // slot.
  if (Slot > StartSlot && Slot <= VFI->getEndSlotFor(&MBB)) {
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
  }

  ucState Ctrl(InsertPos);
  assert(Ctrl->getOpcode() == VTM::Control && "Unexpected instruction type!");

  MachineInstrBuilder Builder(InsertPos);
  Builder.addOperand(ucOperand::CreateOpcode(VTM::VOpMvPhi, Slot));
  Builder.addOperand(ucOperand::CreatePredicate());
  ucOperand Dst = MachineOperand::CreateReg(IncomingReg, true);
  Dst.setBitWidth(DefOp.getBitWidth());
  Builder.addOperand(Dst);

  MachineRegisterInfo &MRI = MBB.getParent()->getRegInfo();
  MachineRegisterInfo::def_iterator DI = MRI.def_begin(SrcReg);
  assert (++MRI.def_begin(SrcReg) == MRI.def_end() && "Not in SSA From!");
  ucOp WriteOp = ucOp::getParent(DI);
  // We need to forward the wire copy if the source register is written
  // in the same slot, otherwise we will read a out of date value.
  if (WriteOp->getParent() == &*Ctrl) {
    if (WriteOp->getOpcode() == VTM::COPY)
      Builder.addOperand(MachineOperand(WriteOp.getOperand(1)));
    // FIXME: Handle others case.
  }

  ucOperand Src = MachineOperand::CreateReg(SrcReg, false);
  Src.setSubReg(SrcSubReg);
  Src.setBitWidth(DefOp.getBitWidth());
  Builder.addOperand(Src);
  return &*Builder;
}

MachineInstr &VInstrInfo::BuildSelect(MachineBasicBlock *MBB, MachineOperand &Result,
                                      const SmallVectorImpl<MachineOperand> &Pred,
                                      MachineOperand IfTrueVal,
                                      MachineOperand IfFalseVal,
                                      const TargetInstrInfo *TII){
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
    assert(Pred.size() == 1 && "Cannot select value!");
    return *BuildMI(MBB, DebugLoc(), TII->get(VTM::VOpSel))
              .addOperand(ResDef).addOperand(Pred[0])
              .addOperand(IfTrueVal).addOperand(IfFalseVal)
              .addOperand(ucOperand::CreatePredicate());
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
  assert(Opcode && "Unsupport move!");

  return *BuildMI(MBB, IP, DebugLoc(), TII->get(Opcode))
            .addOperand(ResDef).addOperand(IfTrueVal).addOperand(Pred[0]);
}

FuncUnitId VIDesc::getPrebindFUId()  const {
  // Dirty Hack: Bind all memory access to channel 0 at this moment.
  if (getTID().Opcode == VTM::VOpMemTrans)
    return FuncUnitId(VFUs::MemoryBus, 0);

  if (getTID().Opcode == VTM::VOpBRam) {
    unsigned Id = get().getOperand(5).getImm();
    return FuncUnitId(VFUs::BRam, Id);
  }

  return FuncUnitId();
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
  MO->ChangeToRegister(0, false);
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
