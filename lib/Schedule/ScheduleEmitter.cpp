//===----- ScheduleEmitter.cpp - Emit the schedule  -------------*- C++ -*-===//
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
// This file implement the Schedule emitter class, which build the state
// instructions from the scheduled schedule unit.
//
//===----------------------------------------------------------------------===//

#include "VSUnit.h"

#include "vtm/VerilogBackendMCTargetDesc.h"
#include "vtm/VFInfo.h"
#include "vtm/VInstrInfo.h"

#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBundle.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/MachineSSAUpdater.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/raw_ostream.h"
#define DEBUG_TYPE "vtm-schedule-emitter"
#include "llvm/Support/Debug.h"

using namespace llvm;
STATISTIC(DanglingDatapath, "Number of dangling data-path operations");

namespace {
class OpSlot {
  int SlotNum;
  OpSlot(int S) : SlotNum(S) {}
  enum SlotType { Control, Datapath };
public:
  OpSlot() : SlotNum(0) {}
  OpSlot(int Slot, bool isCtrl) {
    SlotType T = isCtrl ? Control : Datapath;
    SlotNum = (Slot << 0x1) | (0x1 & T);
  }

  SlotType getSlotType() const {
    return (SlotType)(SlotNum & 0x1);
  }

  bool isControl() const {
    return getSlotType() == OpSlot::Control;
  }

  bool isDatapath() const {
    return getSlotType() == OpSlot::Datapath;
  }

  int getSlot() const { return SlotNum / 2; }

  inline bool operator==(OpSlot S) const {
    return SlotNum == S.SlotNum;
  }

  inline bool operator!=(OpSlot S) const {
    return SlotNum != S.SlotNum;
  }

  inline bool operator<(OpSlot S) const {
    return SlotNum < S.SlotNum;
  }

  inline bool operator<=(OpSlot S) const {
    return SlotNum <= S.SlotNum;
  }

  inline bool operator>(OpSlot S) const {
    return SlotNum > S.SlotNum;
  }
  inline bool operator>=(OpSlot S) const {
    return SlotNum >= S.SlotNum;
  }

  inline OpSlot &setSlot(unsigned RHS) {
    unsigned T = SlotNum & 0x1;
    SlotNum = (RHS << 0x1) | T;
    return *this;
  }

  void setType(bool isCtrl) {
    SlotType T = isCtrl ? Control : Datapath;
    SlotNum = (getSlot() << 0x1) | (0x1 & T);
  }

  inline OpSlot operator+(int RHS) const {
    return OpSlot(getSlot() + RHS, isControl());
  }

  inline OpSlot operator-(int RHS) const {
    return OpSlot(getSlot() - RHS, isControl());
  }

  inline OpSlot &operator+=(int RHS) {
    SlotNum += RHS * 2;
    return *this;
  }

  inline OpSlot &operator++() { return operator+=(1); }

  inline OpSlot operator++(int) {
    OpSlot Temp = *this;
    SlotNum += 2;
    return Temp;
  }

  OpSlot getNextSlot() const { return OpSlot(SlotNum + 1); }
  OpSlot getPrevSlot() const { return OpSlot(SlotNum - 1); }

  int getDetailStep() const { return SlotNum; }

  static OpSlot detailStepCeil(int S, bool isDatapath) {
    //OpSlot s(S);

    //// If the type not match, get the next slot.
    //if (s.isControl() != isCtrl)
    //  return s.getNextSlot();

    //return s;
    bool SIsDataPath = S & 0x1;
    bool TypeNotMatch = SIsDataPath != isDatapath;
    return OpSlot(S + TypeNotMatch);
  }

  static OpSlot detailStepFloor(int S, bool isDatapath) {
    //OpSlot s(S);

    //// If the type not match, get the next slot.
    //if (s.isControl() != isCtrl)
    //  return s.getPrevSlot();

    //return s;
    bool SIsDataPath = S & 0x1;
    bool TypeNotMatch = SIsDataPath != isDatapath;
    return OpSlot(S - TypeNotMatch);
  }
};

// Helper class to build Micro state.
struct MicroStateBuilder {
  MicroStateBuilder(const MicroStateBuilder&);     // DO NOT IMPLEMENT
  void operator=(const MicroStateBuilder&); // DO NOT IMPLEMENT

  VSchedGraph &State;
  MachineBasicBlock &MBB;
  const unsigned ScheduleStartSlot, ScheduleLoopOpSlot, ScheduleEndSlot, II,
                 StartSlot;
  const bool isMBBPipelined;
  typedef MachineInstr *InsertPosTy;
  InsertPosTy InsertPos;

  const TargetInstrInfo &TII;
  MachineRegisterInfo &MRI;
  VFInfo &VFI;

  SmallVector<VSUnit*, 8> SUnitsToEmit;

  struct WireDef {
    unsigned WireNum;
    MachineOperand Pred;
    MachineOperand Op;
    OpSlot DefSlot;
    OpSlot CopySlot;
    OpSlot LoopBoundary;

    WireDef(unsigned wireNum, MachineOperand pred, MachineOperand op,
            OpSlot defSlot, OpSlot copySlot, OpSlot loopBoundary)
      : WireNum(wireNum), Pred(pred), Op(op), DefSlot(defSlot),
      CopySlot(copySlot), LoopBoundary(loopBoundary) {}

    // Do not define a register twice by copying it self.
    bool shouldBeCopied() const {
      return (!Op.isReg() || WireNum != Op.getReg());
    }

    MachineOperand getOperand() const { return Op; }

    MachineOperand createOperand() const {
      return VInstrInfo::CreateReg(WireNum, VInstrInfo::getBitWidth(Op));
    }
  };

  inline WireDef createWireDef(unsigned WireNum, MachineOperand MO,
                               MachineOperand Pred, OpSlot defSlot,
                               OpSlot copySlot/*, bool isPHI = false*/){
    assert(copySlot.isControl() && "Can only copy at control!");
    // Compute the loop boundary, the last slot before starting a new loop,
    // which is at the same time with the first slot of next iteration.
    unsigned LoopBoundarySlot = copySlot.getSlot();
    if (!isMBBPipelined)
      LoopBoundarySlot = ScheduleLoopOpSlot;
    else {
      // For a PHI node, we do not need to insert new PHI for those read simply
      // wrap around to preserve SSA form. But we need a copy to pipeline the
      // value to preserve the dependence.
      // if (isPHI) LoopBoundarySlot += II;
      // else {
        // Otherwise, we need to insert a PHI node to preserve SSA form, by
        // avoiding use before define, which occur if the read is wrap around.
        LoopBoundarySlot -= ScheduleStartSlot;
        LoopBoundarySlot = RoundUpToAlignment(LoopBoundarySlot, II);
        LoopBoundarySlot += ScheduleStartSlot;
      //}
    }
    
    assert((LoopBoundarySlot > ScheduleStartSlot || defSlot == copySlot)
           && LoopBoundarySlot >= unsigned(copySlot.getSlot())
           && "LoopBoundary should bigger than start slot and copyt slot!");
    return WireDef(WireNum, Pred, MO, defSlot, copySlot, OpSlot(LoopBoundarySlot, true));
  }
  
  typedef std::vector<WireDef*> DefVector;
  
  typedef SmallVector<InsertPosTy, 32> IPVector;
  IPVector CtrlIPs, DataPathIPs;

  // register number -> wire define.
  typedef std::map<unsigned, WireDef> SWDMapTy;
  SWDMapTy StateWireDefs;

  MicroStateBuilder(VSchedGraph &S, MachineBasicBlock *MBB, unsigned StartSlot)
  : State(S), MBB(*MBB), ScheduleStartSlot(S.getStartSlot(MBB)),
    ScheduleLoopOpSlot(S.getLoopOpSlot(MBB)), ScheduleEndSlot(S.getEndSlot(MBB)),
    II(S.getII(MBB)), StartSlot(StartSlot), isMBBPipelined(S.isPipelined(MBB)),
    InsertPos(MBB->end()),
    TII(*MBB->getParent()->getTarget().getInstrInfo()),
    MRI(MBB->getParent()->getRegInfo()),
    VFI(*MBB->getParent()->getInfo<VFInfo>())
  {
    // Build the instructions for mirco-states.
    unsigned EndSlot = StartSlot + II;
    MachineInstr *Start =
      BuildMI(*MBB, InsertPos, DebugLoc(), TII.get(VTM::CtrlStart))
        .addImm(StartSlot).addImm(0).addImm(0);
    MachineInstr *End =
      BuildMI(*MBB, InsertPos, DebugLoc(), TII.get(VTM::CtrlEnd))
       .addImm(StartSlot).addImm(0).addImm(0);
    // Control Ops are inserted between Ctrl-Starts and Ctrl-Ends
    CtrlIPs.push_back(End);

    for (unsigned i = StartSlot + 1, e = EndSlot; i <= e; ++i) {
      // Build the header for datapath from in slot.
      BuildMI(*MBB, InsertPos, DebugLoc(), TII.get(VTM::Datapath))
        .addImm(i - 1).addImm(0).addImm(0);
      Start = BuildMI(*MBB, InsertPos, DebugLoc(), TII.get(VTM::CtrlStart))
                .addImm(i).addImm(0).addImm(0);
      End = BuildMI(*MBB, InsertPos, DebugLoc(), TII.get(VTM::CtrlEnd))
              .addImm(i).addImm(0).addImm(0);
      // Datapath Ops are inserted between Ctrl-Ends and Ctrl-Starts
      DataPathIPs.push_back(Start);
      CtrlIPs.push_back(End);
    }

    // Build Datapath bundle for dangling data-paths.
    BuildMI(*MBB, InsertPos, DebugLoc(), TII.get(VTM::Datapath))
      .addImm(EndSlot).addImm(0).addImm(0);
  }

  unsigned getModuloSlot(OpSlot S) const {
    unsigned Slot = S.getSlot();
    bool IsControl = S.isControl();

    assert(Slot >= ScheduleStartSlot && "Bad slot!");
    unsigned Idx = Slot -  ScheduleStartSlot;
    if (isMBBPipelined) {
      Idx %= II;
      // Move the entry of non-first stage to the last slot, so
      // Stage 0: Entry,    state1,        ... state(II - 1),
      // Stage 1: stateII,  state(II + 1), ... state(2II - 1),
      // Stage 2: state2II,  state(2II + 1), ... state(3II - 1),
      // become:
      // Stage 0: Entry,    state1,        ... state(II - 1),   stateII,
      // Stage 1:           state(II + 1), ... state(2II - 1),  state2II,
      // Stage 2:           state(2II + 1), ... state(3II - 1),
      if (IsControl && Idx == 0 && Slot >= ScheduleLoopOpSlot)
        Idx = II;
    }

    return Idx;
  }

  bool isReadWrapAround(OpSlot ReadSlot, WireDef &WD) const {
    return WD.LoopBoundary < ReadSlot;
  }

  InsertPosTy getStateCtrlAt(OpSlot CtrlSlot) {
    unsigned Idx = getModuloSlot(CtrlSlot);
    // Retrieve the instruction at specific slot. 
    MachineInstr *Ret = CtrlIPs[Idx];
    assert(Ret && "Unexpected NULL instruction!");
    return Ret;
  }

  InsertPosTy getStateDatapathAt(OpSlot DatapathSlot) {
    unsigned Idx = getModuloSlot(DatapathSlot);
    // Retrieve the instruction at specific slot.
    MachineInstr *Ret = DataPathIPs[Idx];
    assert(Ret && "Unexpected NULL instruction!");
    return Ret;
  }

  InsertPosTy getMIAt(OpSlot Slot) {
    if (Slot.isControl()) return getStateCtrlAt(Slot);
    else                  return getStateDatapathAt(Slot);
  }

  MachineBasicBlock::iterator getInsertPos() { return InsertPos; }

  // Builder interface.
  void emitSUnit(VSUnit *A) { SUnitsToEmit.push_back(A); }
  bool emitQueueEmpty() const { return SUnitsToEmit.empty(); }

  // Main state building function.
  MachineInstr *buildMicroState(unsigned Slot);

  // Fuse instructions in a bundle.
  void fuseInstr(MachineInstr &Inst, OpSlot SchedSlot, FuncUnitId FUId);

  // Build the machine operand that use at a specified slot.
  MachineOperand getRegUseOperand(MachineOperand MO, OpSlot ReadSlot) {
    unsigned Reg = MO.getReg();
    // Else this is a use.
    SWDMapTy::iterator at = StateWireDefs.find(Reg);
    // Using register from previous state.
    if (at == StateWireDefs.end()) {
      // Do not need to worry about if the new loop overwrite the the loop
      // invariants.
      return MO;
    }

    WireDef &WDef = at->second;

    // We need the value after it is written to register.
    if (WDef.CopySlot < ReadSlot) {
      //assert(((!IsCtrl && ReadSlot == EmitSlot + 1)
      //        || (IsCtrl && ReadSlot == EmitSlot))
      //        && "Assumption of Slots broken!");
      MachineOperand Ret = getRegUseOperand(WDef, ReadSlot, MO);
      if (!MO.isImplicit())
        VInstrInfo::setBitWidth(Ret, VInstrInfo::getBitWidth(MO));
      return Ret;
    }

    assert(WDef.DefSlot <= ReadSlot && "Dependencies broken!");
    // No need a implicit use, because the implicit operand is used explicitly
    // at the same slot.
    // Dirty Hack: Just return something meaningless.
    if (MO.isImplicit()) return MachineOperand::CreateReg(0, false);

    MachineOperand Ret = WDef.createOperand();
    VInstrInfo::setBitWidth(Ret, VInstrInfo::getBitWidth(MO));
    return Ret;
  }

  // Build the machine operand that read the wire definition at a specified slot.
  MachineOperand getRegUseOperand(WireDef &WD, OpSlot ReadSlot, MachineOperand MO);
 
  void emitPHICopy(MachineInstr *PN, unsigned Slot) {
    for (unsigned i = 1; i != PN->getNumOperands(); i += 2) {
      if (PN->getOperand(i + 1).getMBB() != &MBB) continue;

      MachineOperand &SrcMO = PN->getOperand(i);

      OpSlot CopySlot(Slot, true);
      unsigned DstReg = MRI.createVirtualRegister(&VTM::DRRegClass);

      BuildMI(MBB, getStateCtrlAt(CopySlot), DebugLoc(), TII.get(VTM::VOpMove))
        .addOperand(VInstrInfo::CreateReg(DstReg, VInstrInfo::getBitWidth(SrcMO),
                                         true))
        .addOperand(getRegUseOperand(SrcMO, CopySlot))
        .addOperand(VInstrInfo::CreatePredicate())
        .addImm(translateToSlotRegNum(Slot));

      SrcMO.ChangeToRegister(DstReg, false);
    }
  }

  void emitPHIDef(MachineInstr *PN) {
    // FIXME: Place the PHI define at the right slot to avoid the live interval
    // of registers in the PHI overlap. 
    unsigned InsertSlot = /*Slot ? Slot :*/ ScheduleStartSlot;
    MachineOperand &MO = PN->getOperand(0);
    unsigned PHIReg = MO.getReg();
    assert(MRI.getRegClass(PHIReg) == &VTM::DRRegClass
           && "Bad register class for PHI!");
    unsigned NewReg = MRI.createVirtualRegister(&VTM::DRRegClass);

    DebugLoc dl;
    InsertPosTy IP = getStateCtrlAt(OpSlot(InsertSlot, true));
    unsigned BitWidth = VInstrInfo::getBitWidth(MO);
    BuildMI(MBB, IP, dl, TII.get(VTM::VOpDefPhi))
      .addOperand(MO).addOperand(VInstrInfo::CreateReg(NewReg, BitWidth, false))
      .addOperand(VInstrInfo::CreatePredicate())
      .addImm(translateToSlotRegNum(InsertSlot));

    // Update the MO of the Original PHI.
    MO.ChangeToRegister(NewReg, true);
  }

  // Translate the global slot number to unique slot register number.
  unsigned translateToSlotRegNum(unsigned ScheduleSlot) {
    return ScheduleSlot - ScheduleStartSlot + StartSlot;
  }

  void setInstrSlotNum(MachineInstr * MI, unsigned ScheduleSlot) {
    unsigned Slot = translateToSlotRegNum(ScheduleSlot);
    VInstrInfo::setInstrSlotNum(MI, Slot);
  }

  // Build PHI nodes to preserve anti-dependence for pipelined BB.
  unsigned createPHI(unsigned RegNo, unsigned SizeInBits, unsigned WriteSlot,
                     bool WrapOnly) {
    SmallVector<MachineInstr*, 4> InsertedPHIs;

    // PHI node needed.
    // TODO: Move to constructor?
    MachineSSAUpdater SSAUpdate(*MBB.getParent(), &InsertedPHIs);
    SSAUpdate.Initialize(RegNo);
    SSAUpdate.AddAvailableValue(&MBB, RegNo);

    // 1. add an initialize value.
    for (MachineBasicBlock::pred_iterator I = MBB.pred_begin(),
         E = MBB.pred_end();I != E; ++I) {
      MachineBasicBlock *PredBB = *I;
      if (PredBB == &MBB) continue;

      // The register to hold initialize value.
      unsigned InitReg = MRI.createVirtualRegister(&VTM::DRRegClass);
      MachineOperand InitOp = MachineOperand::CreateReg(InitReg, true);
      VInstrInfo::setBitWidth(InitOp, SizeInBits);

      MachineBasicBlock::iterator IP = PredBB->getFirstTerminator();
      // Insert the imp_def before the PHI incoming copies.
      while (llvm::prior(IP)->getOpcode() == VTM::VOpMvPhi)
        --IP;

      BuildMI(*PredBB, IP, DebugLoc(), TII.get(VTM::IMPLICIT_DEF))
        .addOperand(InitOp);

      SSAUpdate.AddAvailableValue(PredBB, InitReg);
    }

    unsigned NewReg = SSAUpdate.GetValueInMiddleOfBlock(&MBB);

    // Update the bitwidth for newly inserted PHIs, insert it into the
    // First ucSate.
    while (!InsertedPHIs.empty()) {
      MachineInstr *PN = InsertedPHIs.pop_back_val();
      MachineOperand &Op = PN->getOperand(0);
      VInstrInfo::setBitWidth(Op, SizeInBits);

      for (unsigned i = 1; i != PN->getNumOperands(); i += 2) {
        MachineOperand &SrcOp = PN->getOperand(i);
        VInstrInfo::setBitWidth(SrcOp, SizeInBits);
      }

      if (!WrapOnly) {
        // If the PHI need copy, set the its register class to DR.
        MRI.setRegClass(Op.getReg(), &VTM::DRRegClass);
        // Emit instructions to preserve the dependence about PHINodes.
        emitPHIDef(PN);
        emitPHICopy(PN, WriteSlot);
      }
    }

    return NewReg;
  }

  // Increase the slot counter and emit all pending schedule units.
  unsigned advanceToSlot(unsigned CurSlot, unsigned TargetSlot) {
    assert(TargetSlot > CurSlot && "Bad target slot!");
    buildMicroState(CurSlot);
    SUnitsToEmit.clear();
    // Advance current slot.
    ++CurSlot;

    // Some states may not emit any atoms, but it may read the result from
    // previous atoms.
    // Note that SUnitsToEmit is empty now, so we do not emitting any new
    // atoms.
    while (CurSlot < TargetSlot && CurSlot < ScheduleEndSlot)
      buildMicroState(CurSlot++);

    return CurSlot;
  }
};
}

//===----------------------------------------------------------------------===//
typedef std::pair<MachineInstr*, OpSlot> InSUInstInfo;
static inline bool sort_intra_latency(const InSUInstInfo &LHS,
                                      const InSUInstInfo &RHS) {
  return LHS.second < RHS.second;
}

MachineInstr* MicroStateBuilder::buildMicroState(unsigned Slot) {
  for (SmallVectorImpl<VSUnit*>::iterator I = SUnitsToEmit.begin(),
       E = SUnitsToEmit.end(); I !=E; ++I) {
    VSUnit *A = *I;

    SmallVector<InSUInstInfo, 8> Insts;
    OpSlot SchedSlot(A->getSlot(), A->isControl());

    for (unsigned i = 0, e = A->num_instrs(); i < e; ++i) {
      MachineInstr *Inst = A->getPtrAt(i);
      // Ignore the entry node marker (null) and implicit define.
      if (Inst && !Inst->isImplicitDef()) {
        if (Inst->isPHI())
          emitPHIDef(Inst);
        else {
          OpSlot S = SchedSlot +  A->getLatencyAt(i);
          S = OpSlot::detailStepCeil(S.getDetailStep(),
                                     VInstrInfo::isDatapath(Inst->getOpcode()));
          Insts.push_back(std::make_pair(Inst, S));
        }
      }
    }

    // Sort the instructions, so we can emit them in order.
    std::sort(Insts.begin(), Insts.end(), sort_intra_latency);

    bool IsDangling = A->isDangling();

    typedef SmallVector<InSUInstInfo, 8>::iterator it;
    for (it I = Insts.begin(), E = Insts.end(); I != E; ++I) {
      MachineInstr *MI = I->first;

      // Simply place the dangling node at the end.
      if (IsDangling){
        ++DanglingDatapath;
        unsigned RegNo = MI->getOperand(0).getReg();
        unsigned Opcode = MI->getOpcode();
        MRI.setRegClass(RegNo, VRegisterInfo::getRepRegisterClass(Opcode));
        VInstrInfo::setInstrSlotNum(MI, 0);
        MI->removeFromParent();
        MBB.push_back(MI);
        continue;
      }

      OpSlot S = I->second;
      fuseInstr(*MI, S, VInstrInfo::getPreboundFUId(MI));
    }
  }

  return 0;
}

void MicroStateBuilder::fuseInstr(MachineInstr &Inst, OpSlot SchedSlot,
                                  FuncUnitId FUId) {
  bool IsCtrl = VInstrInfo::isControl(Inst.getOpcode());
  bool IsCtrlSlot = SchedSlot.isControl();
  assert(IsCtrlSlot == IsCtrl && "Wrong slot type.");
  bool isCopyLike = VInstrInfo::isCopyLike(Inst.getOpcode());
  bool isWriteUntilFinish = VInstrInfo::isWriteUntilFinish(Inst.getOpcode());
  // Compute the slots.
  OpSlot ReadSlot = SchedSlot;

  unsigned StepDelay = State.getStepsToFinish(&Inst);
  unsigned FinSlot = SchedSlot.getSlot() + StepDelay;
  OpSlot CopySlot(FinSlot, true);
  // We can not write the value to a register at the same moment we emit it.
  // Unless we read at emit.
  // FIXME: Introduce "Write at emit."
  if (CopySlot < SchedSlot) ++CopySlot;
  // Write to register operation need to wait one more slot if the result is
  // written at the moment (clock event) that the atom finish.
  //if (VInstrInfo::isWriteUntilFinish(Inst.getOpcode())) ++CopySlot;

  unsigned Opc = Inst.getOpcode();
  // SchedSlot is supposed to strictly smaller than CopySlot, if this not hold
  // then slots is wrapped.
  int WrappedDistance = getModuloSlot(CopySlot) - getModuloSlot(SchedSlot);
  int Distance = CopySlot.getSlot() - SchedSlot.getSlot();
  bool WrappedAround = (WrappedDistance != Distance);
  assert((!WrappedAround || isMBBPipelined)
         && "Live intervals are only wrapped in pipelined block d!");
  // FIX the opcode of terminators.
  if (Inst.isTerminator()) {
    if (VInstrInfo::isBrCndLike(Opc)) Inst.setDesc(TII.get(VTM::VOpToState_nt));
    else                              Inst.setDesc(TII.get(VTM::VOpRet_nt));
  }

  // Handle the predicate operand.
  MachineOperand Pred = *VInstrInfo::getPredOperand(&Inst);
  assert(Pred.isReg() && "Cannot handle predicate operand!");
  
  // Do not copy instruction that is write until finish, which is already taken
  // care by VOpPipelineStage.
  bool NeedCopy = !isWriteUntilFinish;

  // The value defined by this instruction.
  DefVector Defs;
  // Adjust the operand by the timing.
  for (unsigned i = 0 , e = Inst.getNumOperands(); i != e; ++i) {
    MachineOperand &MO = Inst.getOperand(i);

    // Ignore the non-register operand (also ignore reg0 which means nothing).
    if (!MO.isReg() || !MO.getReg())
      continue;

    const unsigned RegNo = MO.getReg();

    // Remember the defines.
    // DiryHack: Do not emit write define for copy since copy is write at
    // control block.

    if (MO.isDef()) {
      if (MRI.use_empty(RegNo)) {
        // Need to fix the register class even the register define is dead.
        MRI.setRegClass(RegNo, VRegisterInfo::getRepRegisterClass(Opc));
        continue;
      }

      unsigned BitWidth = VInstrInfo::getBitWidth(MO);
      // Do not emit write to register unless it not killed in the current state.
      // FIXME: Emit the wire only if the value is not read in a function unit port.
      // if (!NewDef->isSymbol()) {
      // Define wire for operations.
      MachineOperand NewOp = MO;
      unsigned WireNum = NewOp.getReg();

      // Define wire for trivial operation, otherwise, the result of function
      // unit should be wire, and there must be a copy follow up.
      if (!VRegisterInfo::IsWire(RegNo, &MRI) && NeedCopy) {
        assert(CopySlot != SchedSlot
               && "Copy should already set the right RC up!");
        WireNum =
          MRI.createVirtualRegister(VRegisterInfo::getRepRegisterClass(Opc));
        NewOp = VInstrInfo::CreateReg(WireNum, BitWidth, true);
      }

      // If the wire define and the copy wrap around?
      if (WrappedAround)
        WireNum = createPHI(WireNum, BitWidth, SchedSlot.getSlot(), true);

      WireDef WDef = createWireDef(WireNum, MO, Pred, SchedSlot, CopySlot);

      SWDMapTy::iterator mapIt;
      bool inserted;
      tie(mapIt, inserted) = StateWireDefs.insert(std::make_pair(RegNo, WDef));

      assert(inserted && "Instructions not in SSA form!");
      WireDef *NewDef = &mapIt->second;
      // Remember to emit this wire define if necessary.
      Defs.push_back(NewDef);

      // Update the operand.
      MO.ChangeToRegister(NewOp.getReg(), NewOp.isDef(), NewOp.isImplicit());
      MO.setTargetFlags(NewOp.getTargetFlags());
      // }
    } else if (!MO.isDef()) {
      MachineOperand NewOp = getRegUseOperand(MO, ReadSlot);
      // Update the operand.
      MO.ChangeToRegister(NewOp.getReg(), NewOp.isDef(), NewOp.isImplicit());
      MO.setTargetFlags(NewOp.getTargetFlags());
    }
  }

  // Set the scheduled slot of the instruction. Set the schedule slot of
  // data-path operations to 0, because the slot of data-path dose not make sense,
  // but preventing us from merging identical data-path operations.
  unsigned InstrSlot = IsCtrl ? SchedSlot.getSlot() : 0;
  // Also set the slot of datapath if we need.
  DEBUG_WITH_TYPE("vtm-debug-datapath-slot", InstrSlot = SchedSlot.getSlot());

  setInstrSlotNum(&Inst, InstrSlot);
  // Move the instruction to the right place.
  InsertPosTy IP = getMIAt(SchedSlot);
  Inst.removeFromParent();
  MBB.insert((MachineBasicBlock::iterator)IP, &Inst);

  if (!NeedCopy || Defs.empty()) {
    return;
  }

  // Emit the exported registers at current slot.
  IP = getStateCtrlAt(CopySlot);
  while (!Defs.empty()) {
    WireDef *WD = Defs.back();
    Defs.pop_back();

    MachineOperand MO = WD->getOperand();

    // FIXME: Do we need this?
    // This operand will delete with its origin instruction.
    // Eliminate the dead register.
    //if (MRI.use_empty(MO.getReg()))
    //  continue;

    if (WD->shouldBeCopied() && !isCopyLike) {
      unsigned Slot = CopySlot.getSlot();
      // Export the register.
      MachineInstrBuilder Builder = BuildMI(MBB, IP, DebugLoc(),
                                            TII.get(VTM::VOpReadFU));
      MachineOperand Src = WD->createOperand();
      // Do not define MO if we have a implicit use.
      MO.setIsDef();
      Builder.addOperand(MO);

      Builder.addOperand(Src);

      // Also remember the function unit id.
      Builder.addImm(FUId.getData());

      // Get the predicate operand at current slot.
      Builder.addOperand(getRegUseOperand(WD->Pred, CopySlot));

      // Add the operand to hold the schedule.
      Builder.addImm(0);
      // Set the slot number.
      setInstrSlotNum(Builder, Slot);
    }
  }
}

MachineOperand MicroStateBuilder::getRegUseOperand(WireDef &WD, OpSlot ReadSlot,
                                                   MachineOperand MO) {
  bool isImplicit = MO.isImplicit();
  unsigned RegNo = WD.getOperand().getReg();
  unsigned PredReg = WD.Pred.getReg();
  unsigned SizeInBits = VInstrInfo::getBitWidth(WD.Op);

  // Move the value to a new register otherwise the it will be overwritten.
  // If read before write in machine code, insert a phi node.
  while (isReadWrapAround(ReadSlot, WD)) {
    // Because the result of wireops will be copied to register at loop boundary
    // only extend the live interval of its operand to the first loop boundary.
    if (isImplicit && WD.LoopBoundary > WD.DefSlot + II)
      break;

    // Emit the PHI at loop boundary, but do not emit the copy if the wire is
    // defined at loop boundary, otherwise we will get the value from previous
    // iteration which is not we want.
    RegNo = createPHI(RegNo, SizeInBits, WD.LoopBoundary.getSlot(),
                      WD.LoopBoundary == WD.DefSlot);
    MO = VInstrInfo::CreateReg(RegNo, SizeInBits, false);
    WD.Op = MO;

    if (PredReg) {
      PredReg = createPHI(PredReg, 1, WD.LoopBoundary.getSlot(), false);
      WD.Pred = VInstrInfo::CreatePredicate(PredReg);
    }

    // Update the register.
    WD.CopySlot = WD.LoopBoundary;
    WD.LoopBoundary += II;
    assert(WD.CopySlot <= ReadSlot && "Broken PHI Slot!");
  }

  // Return the up to date machine operand
  MO = WD.Op;
  MO.setIsUse();
  MO.setImplicit(isImplicit);
  return MO;
}

typedef std::vector<unsigned> B2SMapTy;
static unsigned calculateMinSlotsFromEntry(VSUnit *BBEntry,
                                           const B2SMapTy &ExitSlots) {
  unsigned SlotsFromEntry = BBEntry->getSlot();

  typedef VSUnit::dep_iterator dep_it;
  for (dep_it I = cp_begin(BBEntry), E = cp_end(BBEntry); I != E; ++I) {
    VSUnit *PredTerminator = *I;
    //DEBUG(dbgs() << "MBB#" << PredTerminator->getParentBB()->getNumber()
    //             << "->MBB#" << BBEntry->getParentBB()->getNumber() << " slack: "
    //             << (BBEntry->getSlot() - PredTerminator->getSlot()) << '\n');

    MachineBasicBlock *PredBB = PredTerminator->getParentBB();
    unsigned PredDistance = ExitSlots[PredBB->getNumber()];
    // Consider the latency of the control dependency edge.
    assert(PredDistance && "Not visiting the BBs in topological order?");
    SlotsFromEntry = std::min<unsigned>(SlotsFromEntry, PredDistance);
  }

  return SlotsFromEntry;
}

static int calulateMinInterBBSlack(VSUnit *BBEntry, VSchedGraph &G,
                                   const B2SMapTy &Map,
                                   unsigned MinSlotsForEntry) {
  unsigned EntrySlot = BBEntry->getSlot();
  MachineBasicBlock *MBB = BBEntry->getParentBB();
  typedef VSUnit::use_iterator use_it;
  typedef VSUnit::dep_iterator dep_it;
  // The minimal slack from the predecessors of the BB to the entry of the BB.
  int MinInterBBSlack = 0;

  for (use_it I = cuse_begin(BBEntry), E = cuse_end(BBEntry); I != E; ++I) {
    VSUnit *U = *I;
    if (U->getParentBB() != MBB) continue;

    unsigned USlot = U->getSlot();
    for (dep_it DI = cp_begin(U), DE = cp_end(U); DI != DE; ++DI) {
      if (!DI.isCrossBB()) continue;

      MachineBasicBlock *SrcBB = DI->getParentBB();
      assert(SrcBB != MBB && "Not cross BB depenency!");
      VSUnit *SrcTerminator = G.lookUpTerminator(SrcBB);
      unsigned SrcExitSlot = Map[SrcBB->getNumber()];
      assert(SrcExitSlot && "Unexpected cross BB back-edge!");
      // Calculate the latency distributed between the SrcBB and the SnkBB.
      assert(USlot >= EntrySlot && SrcTerminator->getSlot() >= DI->getSlot()
              && "Bad schedule!");
      int InterBBLatency = DI.getLatency() - (USlot - EntrySlot)
                            - (SrcTerminator->getSlot() - DI->getSlot());
      // Calculate the minimal latency between the SrcBB and SnkBB.
      int ActualInterBBlatency = MinSlotsForEntry - SrcExitSlot;
      DEBUG(dbgs().indent(4) << "Found InterBBLatency and ActualInterBBlatency"
                              << InterBBLatency << ' '
                              << ActualInterBBlatency << '\n');
      // Calculate the slack.
      int Slack = ActualInterBBlatency - InterBBLatency;
      MinInterBBSlack = std::min<int>(MinInterBBSlack, Slack);
    }
  }

  return MinInterBBSlack;
}

void VSchedGraph::insertDelayBlock(MachineBasicBlock *From,
                                   MachineBasicBlock *To, unsigned Latency) {
  MachineInstr *BRInstr = 0;
  typedef MachineBasicBlock::reverse_instr_iterator reverse_iterator;

  for (reverse_iterator I = From->instr_rbegin(), E = From->instr_rend();
       I != E && I->isTerminator(); ++I) {
    if (!VInstrInfo::isBrCndLike(I->getOpcode())) continue;

    if (I->getOperand(1).getMBB() != To) continue;

    BRInstr = &*I;
    break;
  }

  assert(BRInstr && "The corresponding branch instruction not found!");

  MachineFunction *MF = From->getParent();
  MachineBasicBlock *DelayBlock = MF->CreateMachineBasicBlock();
  MF->push_back(DelayBlock);

  // Redirect the target block of the branch instruction to the newly created
  // block, and modify the CFG.
  BRInstr->getOperand(1).setMBB(DelayBlock);
  From->replaceSuccessor(To, DelayBlock);
  DelayBlock->addSuccessor(To);

  // Also fix the PHIs in the original sink block.
  typedef MachineBasicBlock::instr_iterator iterator;
  for (iterator I = To->instr_begin(), E = To->instr_end();
       I != E && I->isPHI(); ++I) {
    for (unsigned i = 1, e = I->getNumOperands(); i != e; i += 2)
      if (I->getOperand(i + 1).getMBB() == From) {
        I->getOperand(i + 1).setMBB(DelayBlock);
        break;
      }
  }

  // Build the entry of the delay block.
  VSUnit *EntrySU = createVSUnit(DelayBlock);
  EntrySU->scheduledTo(EntrySlot);
  EntrySU->setIsDangling(false);

  // And the terminator.
  MachineInstr *DelayBR
    = BuildMI(DelayBlock, DebugLoc(), VInstrInfo::getDesc(VTM::VOpToState))
       .addOperand(VInstrInfo::CreatePredicate()).addMBB(To)
       .addOperand(VInstrInfo::CreatePredicate())
       .addOperand(VInstrInfo::CreateTrace());

  VSUnit *U = createTerminator(DelayBlock);
  mapMI2SU(DelayBR, U, 0);
  U->scheduledTo(EntrySlot + Latency);
  U->setIsDangling(false);

  addDummyLatencyEntry(DelayBR);

  // TODO: Fix the dependencies edges.
}

bool VSchedGraph::insertDelayBlocks() {
  bool AnyLatencyFixed = false;

  B2SMapTy ExitSlots(num_bbs(), 0);
  std::set<VSUnit*> Visited;

  std::vector<std::pair<VSUnit*, VSUnit::dep_iterator> > WorkStack;
  VSUnit *ExitRoot = getExitRoot();
  WorkStack.push_back(std::make_pair(ExitRoot, cp_begin(ExitRoot)));

  // Visit the blocks in topological order, and ignore the pseudo exit node.
  for (;;) {
    VSUnit *U = WorkStack.back().first;
    VSUnit::dep_iterator ChildIt = WorkStack.back().second;

    if (ChildIt == cp_end(U)) {
      WorkStack.pop_back();
      // No need to visit the exit root.
      if (WorkStack.empty()) break;

      // All dependencies visited, now visit the current BBEntry.
      assert(U->isBBEntry() && "Unexpected non-entry node!");
      int CurDistance = calculateMinSlotsFromEntry(U, ExitSlots);

      int MinInterBBSlack
        = calulateMinInterBBSlack(U, *this, ExitSlots, CurDistance);

      DEBUG(dbgs() << "Minimal Slack: " << MinInterBBSlack << '\n');

      if (MinInterBBSlack < 0) {
        typedef VSUnit::dep_iterator dep_it;
        for (dep_it I = cp_begin(U), E = cp_end(U); I != E; ++I) {
          VSUnit *PredTerminator = *I;
          //DEBUG(dbgs() << "MBB#" << PredTerminator->getParentBB()->getNumber()
          //             << "->MBB#" << BBEntry->getParentBB()->getNumber() << " slack: "
          //             << (BBEntry->getSlot() - PredTerminator->getSlot()) << '\n');

          MachineBasicBlock *PredBB = PredTerminator->getParentBB();
          int PredDistance = ExitSlots[PredBB->getNumber()];
          int ExtraLatency = CurDistance - PredDistance - MinInterBBSlack;
          if (ExtraLatency <= 0) continue;

          insertDelayBlock(PredBB, U->getParentBB(), ExtraLatency);
        }
      }

      MachineBasicBlock *MBB = U->getParentBB();
      unsigned ExitSlot = CurDistance + getTotalSlot(MBB) - MinInterBBSlack;
      assert(ExitSlot && "Bad schedule!");
      ExitSlots[MBB->getNumber()] = ExitSlot;

      continue;
    }

    VSUnit *ChildNode = *ChildIt;
    ++WorkStack.back().second;

    assert(ChildNode->isTerminator() && "Unexpected non-terminator node!");
    if (!Visited.insert(ChildNode).second) continue;

    // Jump to the entry of the parent block of the terminator.
    ChildNode = lookupSUnit(ChildNode->getParentBB());
    WorkStack.push_back(std::make_pair(ChildNode, cp_begin(ChildNode)));
  }

  return AnyLatencyFixed;
}

void VSchedGraph::insertDisableFU(MachineInstr *MI, VSUnit *U) {
  MachineRegisterInfo &MRI = DLInfo.MRI;
  const TargetRegisterClass *RC
    = VRegisterInfo::getRepRegisterClass(MI->getOpcode());

  MachineBasicBlock *MBB = MI->getParent();
  DebugLoc dl = MI->getDebugLoc();
  FuncUnitId Id = VInstrInfo::getPreboundFUId(MI);
  MachineBasicBlock::instr_iterator IP = MI;
  ++IP;

  // Insert pseudo copy to model write until finish.
  // FIXME: Insert for others MI rather than Representative MI?
  if (U->getLatency() && VInstrInfo::isWriteUntilFinish(U->getOpcode())) {
    assert(U->getOpcode() == VTM::VOpBRAMTrans &&
           "Only support BRAMTrans at the moment.");
    MachineOperand &MO = MI->getOperand(0);
    unsigned OldR = MO.getReg();
    unsigned R = MRI.createVirtualRegister(RC);
    // Change to the newly allocated register, and kill the new register
    // with VOpPipelineStage.
    MO.ChangeToRegister(R, true);
    unsigned ResultWidth = VInstrInfo::getBitWidth(MO);
    MachineInstr *PipeStage =
      BuildMI(*MBB, IP, dl, VInstrInfo::getDesc(VTM::VOpPipelineStage))
        .addOperand(VInstrInfo::CreateReg(OldR, ResultWidth, true))
        .addOperand(VInstrInfo::CreateReg(R, ResultWidth))
        .addOperand(*VInstrInfo::getPredOperand(MI))
        .addOperand(*VInstrInfo::getTraceOperand(MI));
    assert(U->isControl() && "Only control operation write untill finish!");

    mapMI2SU(PipeStage, U, U->getLatency() - 1, true);

    // Copy the result 1 cycle later after the value is finished, note that
    // the PipeStage is emit to a data path slot, to delay the VOpReadFU 1
    // cycle later, we need to set its delay to 2, other wise the copy will
    // emit right after the RepLI finish, instead of 1 cycle after the RepLI
    // finish. FIXME: Set the right latency.
    addDummyLatencyEntry(PipeStage, 2.0f);
  }

  if (Id.isBound()) {
    assert(Id.getFUType() != VFUs::Mux && "Unexpected FU type!");
    MachineOperand FU = MI->getOperand(0);
    FU.clearParent();
    FU.setIsDef(false);
    MachineInstr *DisableMI =
      BuildMI(*MBB, IP, dl, VInstrInfo::getDesc(VTM::VOpDisableFU))
      .addOperand(FU).addImm(Id.getData())
      .addOperand(*VInstrInfo::getPredOperand(MI))
      .addOperand(*VInstrInfo::getTraceOperand(MI));
    // Add the instruction into the emit list, disable the FU 1 clock later.
    mapMI2SU(DisableMI, U, 1);
    addDummyLatencyEntry(DisableMI);
  }
}

void VSchedGraph::insertReadFU(MachineInstr *MI, VSUnit *U) {
  MachineRegisterInfo &MRI = DLInfo.MRI;
  const TargetRegisterClass *RC
    = VRegisterInfo::getRepRegisterClass(MI->getOpcode());

  MachineBasicBlock *MBB = MI->getParent();
  DebugLoc dl = MI->getDebugLoc();
  FuncUnitId Id = VInstrInfo::getPreboundFUId(MI);
  MachineBasicBlock::instr_iterator IP = MI;
  ++IP;

  unsigned StepsToCopy = getStepsToFinish(MI);
  if (StepsToCopy == 0) return;

  unsigned Slot = U->getSlot();
  unsigned ResultWire = MI->getOperand(0).getReg();
  unsigned RegWidth = VInstrInfo::getBitWidth(MI->getOperand(0));
  MRI.setRegClass(ResultWire, RC);
  unsigned ResultReg = 0;

  typedef MachineRegisterInfo::use_iterator reg_use_iterator;
  for (reg_use_iterator I = MRI.use_begin(ResultWire);
       I != MachineRegisterInfo::use_end(); /*++I*/) {
    MachineInstr &UseMI = *I;
    MachineOperand &UseMO = (I++).getOperand();
    VSUnit *UseSU = lookupSUnit(&UseMI);
    if (UseSU == U) continue;

    assert(UseSU && "Cannot find Use SU!");
    if (UseSU->getSlot() - Slot <= StepsToCopy + (UseSU->isDatapath() ? -1 : 1))
      continue;

    if (ResultReg == 0) ResultReg = MRI.createVirtualRegister(&VTM::DRRegClass);
    UseMO.ChangeToRegister(ResultReg, false);
  }

  // If emit the VOpReadFU, even there is no instruction using the result wire,
  // because we need to wait the MI by the VOpReadFU.
  assert(VInstrInfo::isAlwaysTruePred(*VInstrInfo::getPredOperand(MI))
         && "Unexpected predicated MI!");
  MachineInstr *ReadFU
    = BuildMI(*MBB, IP, dl, VInstrInfo::getDesc(VTM::VOpReadFU))
        .addOperand(VInstrInfo::CreateReg(ResultReg, RegWidth, true))
        .addOperand(VInstrInfo::CreateReg(ResultWire, RegWidth, false))
        .addImm(Id.getData()).addOperand(*VInstrInfo::getPredOperand(MI))
        .addOperand(*VInstrInfo::getTraceOperand(MI));
  mapMI2SU(ReadFU, U, StepsToCopy, true);
  addDummyLatencyEntry(ReadFU);
}

static inline bool top_sort_slot(const VSUnit* LHS, const VSUnit* RHS) {
  if (LHS->getSlot() != RHS->getSlot())
    return LHS->getSlot() < RHS->getSlot();

  return LHS->getIdx() < RHS->getIdx();
}

void VSchedGraph::insertReadFUAndDisableFU() {
  // Sort the SUs by parent BB and its schedule.
  std::sort(CPSUs.begin(), CPSUs.end(), top_sort_slot);

  for (iterator I = CPSUs.begin(), E = CPSUs.end(); I != E; ++I)
    if (MachineInstr *MI = (*I)->getRepresentativePtr()) {
      // Ignore data-path operations at the moment.
      if (VInstrInfo::isDatapath(MI->getOpcode())) continue;

      insertDisableFU(MI, *I);
      insertReadFU(MI, *I);
    }
}

static inline bool top_sort_bb_and_slot(const VSUnit* LHS, const VSUnit* RHS) {
  if (LHS->getParentBB() != RHS->getParentBB())
    return LHS->getParentBB()->getNumber() < RHS->getParentBB()->getNumber();

  return top_sort_slot(LHS, RHS);
}

unsigned VSchedGraph::emitSchedule() {
  // Erase the virtual exit root right now, so that we can avoid the special
  // code to handle it.
  assert(CPSUs.back() == getExitRoot() && "ExitRoot at an unexpected position!");
  CPSUs.resize(CPSUs.size() - 1);
  // Insert the delay blocks to fix the inter-bb-latencies.
  insertDelayBlocks();

  // Merge the data-path SU vector to the control-path SU vector.
  CPSUs.insert(CPSUs.end(), DPSUs.begin(), DPSUs.end());
  DPSUs.clear();
  // Break the chains.
  insertReadFUAndDisableFU();

  // Sort the SUs by parent BB and its schedule.
  std::sort(CPSUs.begin(), CPSUs.end(), top_sort_bb_and_slot);

  unsigned MBBStartSlot = EntrySlot;
  iterator to_emit_begin = CPSUs.begin();
  MachineBasicBlock *PrevBB = (*to_emit_begin)->getParentBB();
  for (iterator I = to_emit_begin, E = CPSUs.end(); I != E; ++I) {
    MachineBasicBlock *CurBB = (*I)->getParentBB();
    if (CurBB == PrevBB) continue;

    // If we are entering a new BB, emit the SUs in the previous bb.
    MBBStartSlot = emitSchedule(to_emit_begin, I, MBBStartSlot, PrevBB);
    to_emit_begin = I;
    PrevBB = CurBB;
  }

  // Dont forget the SUs in last BB.
  MBBStartSlot = emitSchedule(to_emit_begin, CPSUs.end(), MBBStartSlot, PrevBB);

  // Re-add the exit roots into the SU vector.
  CPSUs.push_back(getExitRoot());

  return MBBStartSlot;
}

unsigned VSchedGraph::emitSchedule(iterator su_begin, iterator su_end,
                                   unsigned StartSlot, MachineBasicBlock *MBB) {
  unsigned CurSlot = getStartSlot(MBB), EndSlot = getEndSlot(MBB);
  MachineFunction *MF = MBB->getParent();
  VFInfo *VFI = MF->getInfo<VFInfo>();

  // Build bundle from schedule units.
  MicroStateBuilder StateBuilder(*this, MBB, StartSlot);
  DEBUG(dbgs() << "\nEmitting schedule in MBB#" << MBB->getNumber() << '\n';
        dbgs() << "Sorted AllSUs:\n";
              for (iterator I = su_begin, E = su_end; I != E; ++I)
                (*I)->dump();
        );

  for (iterator I = su_begin, E = su_end; I != E; ++I) {
    VSUnit *A = *I;
    DEBUG(dbgs() << "Going to emit: "; A->dump());
    if (A->getSlot() != CurSlot && CurSlot < EndSlot)
      CurSlot = StateBuilder.advanceToSlot(CurSlot, A->getSlot());

    StateBuilder.emitSUnit(A);
  }
  // Build last state.
  assert(!StateBuilder.emitQueueEmpty() && "Expect atoms for last state!");
  StateBuilder.advanceToSlot(CurSlot, CurSlot + 1);
  // Build the dummy terminator.
  BuildMI(MBB, DebugLoc(), MF->getTarget().getInstrInfo()->get(VTM::EndState))
    .addImm(0).addImm(0);

  DEBUG(dbgs() << "After schedule emitted:\n");
  DEBUG(dump());
  DEBUG(dbgs() << '\n');
  // Remember the schedule information.
  unsigned LoopOpSlot = StartSlot + getLoopOpSlot(MBB) - getStartSlot(MBB);
  VFI->rememberTotalSlot(MBB, StartSlot, getTotalSlot(MBB), LoopOpSlot);
  // Advance 1 slots after the endslot.
  return StartSlot + getTotalSlot(MBB) + 1;
}
