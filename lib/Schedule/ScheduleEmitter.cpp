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

#include "vtm/VFInfo.h"
#include "vtm/MicroState.h"

#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBundle.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/MachineSSAUpdater.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/raw_ostream.h"
#define DEBUG_TYPE "vtm-schedule-emitter"
#include "llvm/Support/Debug.h"

using namespace llvm;

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
  typedef MachineInstr *InsertPosTy;
  InsertPosTy InsertPos;

  const TargetInstrInfo &TII;
  MachineRegisterInfo &MRI;
  VFInfo &VFI;

  SmallVector<VSUnit*, 8> SUnitsToEmit;

  struct WireDef {
    unsigned WireNum;
    ucOperand Pred;
    ucOperand Op;
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
      return ucOperand::CreateReg(WireNum, Op.getBitWidth());
    }
  };

  inline WireDef createWireDef(unsigned WireNum, MachineOperand MO,
                               MachineOperand Pred, OpSlot defSlot,
                               OpSlot copySlot/*, bool isPHI = false*/){
    assert(copySlot.isControl() && "Can only copy at control!");
    // Compute the loop boundary, the last slot before starting a new loop,
    // which is at the same time with the first slot of next iteration.
    unsigned LoopBoundarySlot = copySlot.getSlot();
    if (!State.isPipelined())
      LoopBoundarySlot = State.getLoopOpSlot();
    else {
      unsigned II = State.getII();
      // For a PHI node, we do not need to insert new PHI for those read simply
      // wrap around to preserve SSA form. But we need a copy to pipeline the
      // value to preserve the dependence.
      // if (isPHI) LoopBoundarySlot += II;
      // else {
        // Otherwise, we need to insert a PHI node to preserve SSA form, by
        // avoiding use before define, which occur if the read is wrap around.
        LoopBoundarySlot -= State.getStartSlot();
        LoopBoundarySlot = RoundUpToAlignment(LoopBoundarySlot, II);
        LoopBoundarySlot += State.getStartSlot();
      //}
    }
    
    assert(LoopBoundarySlot > State.getStartSlot()
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

  MicroStateBuilder(VSchedGraph &S)
  : State(S), MBB(*S.getMachineBasicBlock()), InsertPos(MBB.end()),
    TII(*MBB.getParent()->getTarget().getInstrInfo()),
    MRI(MBB.getParent()->getRegInfo()),
    VFI(*MBB.getParent()->getInfo<VFInfo>())
  {
    // Build the instructions for mirco-states.
    unsigned StartSlot = State.getStartSlot();
    unsigned EndSlot = StartSlot + State.getII();
    MachineInstr *Start =
      BuildMI(MBB, InsertPos, DebugLoc(), TII.get(VTM::CtrlStart))
        .addImm(StartSlot).addImm(0).addImm(0);
    MachineInstr *End =
      BuildMI(MBB, InsertPos, DebugLoc(), TII.get(VTM::CtrlEnd))
       .addImm(StartSlot).addImm(0).addImm(0);
    // Control Ops are inserted between Ctrl-Starts and Ctrl-Ends
    CtrlIPs.push_back(End);

    for (unsigned i = StartSlot + 1, e = EndSlot; i <= e; ++i) {
      // Build the header for datapath from in slot.
      BuildMI(MBB, InsertPos, DebugLoc(), TII.get(VTM::Datapath))
        .addImm(i - 1).addImm(0).addImm(0);
      Start = BuildMI(MBB, InsertPos, DebugLoc(), TII.get(VTM::CtrlStart))
                .addImm(i).addImm(0).addImm(0);
      End = BuildMI(MBB, InsertPos, DebugLoc(), TII.get(VTM::CtrlEnd))
              .addImm(i).addImm(0).addImm(0);
      // Datapath Ops are inserted between Ctrl-Ends and Ctrl-Starts
      DataPathIPs.push_back(Start);
      CtrlIPs.push_back(End);
    }
  }

  unsigned getModuloSlot(OpSlot S) const {
    unsigned Slot = S.getSlot();
    bool IsControl = S.isControl();

    unsigned Idx = Slot -  State.getStartSlot();
    if (State.isPipelined()) {
      unsigned II = State.getII();
      Idx %= II;
      // Move the entry of non-first stage to the last slot, so
      // Stage 0: Entry,    state1,        ... state(II - 1),
      // Stage 1: stateII,  state(II + 1), ... state(2II - 1),
      // Stage 2: state2II,  state(2II + 1), ... state(3II - 1),
      // become:
      // Stage 0: Entry,    state1,        ... state(II - 1),   stateII,
      // Stage 1:           state(II + 1), ... state(2II - 1),  state2II,
      // Stage 2:           state(2II + 1), ... state(3II - 1),
      if (IsControl && Idx == 0 && Slot >= State.getLoopOpSlot())
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
  MachineOperand getRegUseOperand(ucOperand MO, OpSlot ReadSlot) {
    // Else this is a use.
    SWDMapTy::iterator at = StateWireDefs.find(MO.getReg());
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
      ucOperand Ret = getRegUseOperand(WDef, ReadSlot, MO);
      if (!MO.isImplicit()) Ret.setBitWidth(MO.getBitWidth());
      return Ret;
    }

    assert(WDef.DefSlot <= ReadSlot && "Dependencies broken!");
    // No need a implicit use, because the implicit operand is used explicitly
    // at the same slot.
    // Dirty Hack: Just return something meaningless.
    if (MO.isImplicit()) return MachineOperand::CreateReg(0, false);

    ucOperand Ret = WDef.createOperand();
    Ret.setBitWidth(MO.getBitWidth());
    return Ret;
  }

  // Build the machine operand that read the wire definition at a specified slot.
  MachineOperand getRegUseOperand(WireDef &WD, OpSlot ReadSlot, ucOperand MO);
 
  void emitPHICopy(MachineInstr *PN, unsigned Slot) {
    for (unsigned i = 1; i != PN->getNumOperands(); i += 2) {
      if (PN->getOperand(i + 1).getMBB() != &MBB) continue;

      ucOperand &SrcMO = cast<ucOperand>(PN->getOperand(i));

      OpSlot CopySlot(Slot, true);
      unsigned DstReg = MRI.createVirtualRegister(VTM::DRRegisterClass);

      BuildMI(MBB, getStateCtrlAt(CopySlot), DebugLoc(), TII.get(VTM::VOpMove))
        .addOperand(ucOperand::CreateReg(DstReg, SrcMO.getBitWidth(), true))
        .addOperand(getRegUseOperand(SrcMO, CopySlot))
        .addOperand(ucOperand::CreatePredicate())
        .addImm(Slot);

      SrcMO.ChangeToRegister(DstReg, false);
    }
  }

  void emitPHIDef(MachineInstr *PN) {
    // FIXME: Place the PHI define at the right slot to avoid the live interval
    // of registers in the PHI overlap. 
    unsigned InsertSlot = /*Slot ? Slot :*/ State.getStartSlot();
    ucOperand &MO = cast<ucOperand>(PN->getOperand(0));
    unsigned PHIReg = MO.getReg();
    assert(MRI.getRegClass(PHIReg) == VTM::DRRegisterClass
           && "Bad register class for PHI!");
    unsigned NewReg = MRI.createVirtualRegister(VTM::DRRegisterClass);

    BuildMI(MBB, getStateCtrlAt(OpSlot(InsertSlot, true)), DebugLoc(),
            TII.get(VTM::VOpDefPhi))
      .addOperand(MO)
      .addOperand(ucOperand::CreateReg(NewReg, MO.getBitWidth(), false))
      .addOperand(ucOperand::CreatePredicate())
      .addImm(InsertSlot);

    // Update the MO of the Original PHI.
    MO.ChangeToRegister(NewReg, true);
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
      unsigned InitReg = MRI.createVirtualRegister(VTM::DRRegisterClass);
      ucOperand InitOp = MachineOperand::CreateReg(InitReg, true);
      InitOp.setBitWidth(SizeInBits);

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
      ucOperand &Op = cast<ucOperand>(PN->getOperand(0));
      Op.setBitWidth(SizeInBits);

      for (unsigned i = 1; i != PN->getNumOperands(); i += 2) {
        ucOperand &SrcOp = cast<ucOperand>(PN->getOperand(i));
        SrcOp.setBitWidth(SizeInBits);
      }

      if (!WrapOnly) {
        // If the PHI need copy, set the its register class to DR.
        MRI.setRegClass(Op.getReg(), VTM::DRRegisterClass);
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
    while (CurSlot < TargetSlot && CurSlot < State.getEndSlot())
      buildMicroState(CurSlot++);

    return CurSlot;
  }

  void buildBundles() {
    typedef MachineBasicBlock::instr_iterator it;
    for (it I = MBB.instr_begin(), E = MBB.instr_end(); I != E; ++I) {
      MachineInstr *MI = I;
      if (MI->isPHI() || MI->isImplicitDef() ||
          MI->getOpcode() == VTM::CtrlStart || MI->getOpcode() == VTM::Datapath)
        continue;

      MI->setIsInsideBundle();
    }    
  }
};
}

//===----------------------------------------------------------------------===//
typedef std::pair<MachineInstr*, int8_t> InSUInstInfo;
static inline bool sort_intra_latency(const InSUInstInfo &LHS,
                                      const InSUInstInfo &RHS) {
  return LHS.second < RHS.second;
}

MachineInstr* MicroStateBuilder::buildMicroState(unsigned Slot) {
  for (SmallVectorImpl<VSUnit*>::iterator I = SUnitsToEmit.begin(),
       E = SUnitsToEmit.end(); I !=E; ++I) {
    VSUnit *A = *I;

    SmallVector<InSUInstInfo, 8> Insts;

    // Insert the DiableFU instruction If necessary.
    MachineInstr *RepMI = A->getRepresentativeInst();
    if (RepMI) {
      FuncUnitId Id = VInstrInfo::getPreboundFUId(RepMI);
      if (!Id.isTrivial() && Id.getFUType() != VFUs::Mux) {
        MachineBasicBlock *MBB = RepMI->getParent();

        MachineOperand FU = RepMI->getOperand(0);
        FU.clearParent();
        FU.setIsDef(false);
        DebugLoc dl = RepMI->getDebugLoc();
        MachineBasicBlock::iterator II = RepMI;
        ++II;
        MachineInstr *DisableMI =
          BuildMI(*MBB, II, dl, VInstrInfo::getDesc(VTM::VOpDisableFU))
            .addOperand(FU).addImm(Id.getData())
            .addOperand(*VInstrInfo::getPredOperand(RepMI))
            .addOperand(ucOperand::CreateTrace(MBB));
        // Add the instruction into the emit list.
        Insts.push_back(std::make_pair(DisableMI, 1));
        State.addDummyLatencyEntry(DisableMI);
      }
    }

    for (unsigned i = 0, e = A->num_instrs(); i < e; ++i) {
      MachineInstr *Inst = A->getInstrAt(i);
      // Ignore the entry node marker (null) and implicit define.
      if (Inst && !Inst->isImplicitDef()) {
        if (Inst->isPHI())
          emitPHIDef(Inst);
        else
          Insts.push_back(std::make_pair(Inst, i ? A->getLatencyAt(i) : 0));
      }
    }

    // Sort the instructions, so we can emit them in order.
    std::sort(Insts.begin(), Insts.end(), sort_intra_latency);

    OpSlot SchedSlot(A->getSlot(), A->isControl());

    typedef SmallVector<InSUInstInfo, 8>::iterator it;
    for (it I = Insts.begin(), E = Insts.end(); I != E; ++I) {
      MachineInstr *MI = I->first;
      OpSlot S = SchedSlot + I->second;
      S = OpSlot::detailStepCeil(S.getDetailStep(),
                                 VInstrInfo::isDatapath(MI->getOpcode()));
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
  // Compute the slots.
  OpSlot ReadSlot = SchedSlot;

  unsigned FinSlot = SchedSlot.getSlot() + State.getStepsToFinish(&Inst);
  OpSlot CopySlot(FinSlot, true);
  // We can not write the value to a register at the same moment we emit it.
  // Unless we read at emit.
  // FIXME: Introduce "Write at emit."
  if (CopySlot < SchedSlot) ++CopySlot;
  // Write to register operation need to wait one more slot if the result is
  // written at the moment (clock event) that the atom finish.
  if (VInstrInfo::isWriteUntilFinish(Inst.getOpcode())) ++CopySlot;

  unsigned Opc = Inst.getOpcode();
  // FIX the opcode of terminators.
  if (Inst.isTerminator()) {
    if (VInstrInfo::isBrCndLike(Opc)) Inst.setDesc(TII.get(VTM::VOpToState_nt));
    else                              Inst.setDesc(TII.get(VTM::VOpRet_nt));
  }

  // Handle the predicate operand.
  ucOperand Pred = *VInstrInfo::getPredOperand(&Inst);
  assert(Pred.isReg() && "Cannot handle predicate operand!");

  // The value defined by this instruction.
  DefVector Defs;
  
  // Adjust the operand by the timing.
  for (unsigned i = 0 , e = Inst.getNumOperands(); i != e; ++i) {
    MachineOperand &MO = Inst.getOperand(i);

    // Ignore the non-register operand (also ignore reg0 which means nothing).
    if (!MO.isReg() || !MO.getReg())
      continue;

    unsigned RegNo = MO.getReg();

    // Remember the defines.
    // DiryHack: Do not emit write define for copy since copy is write at
    // control block.
    if (MO.isDef() && SchedSlot != CopySlot && !isCopyLike) {
      unsigned BitWidth = cast<ucOperand>(MO).getBitWidth();
      // Do not emit write to register unless it not killed in the current state.
      // FIXME: Emit the wire only if the value is not read in a function unit port.
      // if (!NewDef->isSymbol()) {
      // Define wire for operations.
      ucOperand NewOp = MO;
      unsigned WireNum = NewOp.getReg();

      // Define wire for trivial operation, otherwise, the result of function
      // unit should be wire, and there must be a copy follow up.
      if (!VRegisterInfo::IsWire(RegNo, &MRI)) {
        WireNum =
          MRI.createVirtualRegister(VRegisterInfo::getRepRegisterClass(Opc));
        NewOp = ucOperand::CreateReg(WireNum, BitWidth, true);
      }

      // If the wire define and the copy wrap around?
      if (getModuloSlot(SchedSlot) > getModuloSlot(CopySlot))
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

  VInstrInfo::getPredOperand(&Inst)[1].ChangeToImmediate(InstrSlot);
  // Move the instruction to the right place.
  InsertPosTy IP = getMIAt(SchedSlot);
  Inst.removeFromParent();
  MBB.insert((MachineBasicBlock::iterator)IP, &Inst);

  if (isCopyLike) {
    assert(Defs.empty() && "CopyLike instructions do not have extra defs!");
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

    if (WD->shouldBeCopied()) {
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
      // Add the slot number.
      Builder.addImm(Slot);
    }
  }
}

MachineOperand MicroStateBuilder::getRegUseOperand(WireDef &WD, OpSlot ReadSlot,
                                                   ucOperand MO) {
  bool isImplicit = MO.isImplicit();
  unsigned RegNo = WD.getOperand().getReg();
  unsigned PredReg = WD.Pred.getReg();
  unsigned SizeInBits = WD.Op.getBitWidth();

  // Move the value to a new register otherwise the it will be overwritten.
  // If read before write in machine code, insert a phi node.
  while (isReadWrapAround(ReadSlot, WD)) {
    // Because the result of wireops will be copied to register at loop boundary
    // only extend the live interval of its operand to the first loop boundary.
    if (isImplicit && WD.LoopBoundary > WD.DefSlot + State.getII())
      break;

    // Emit the PHI at loop boundary
    RegNo = createPHI(RegNo, SizeInBits, WD.LoopBoundary.getSlot(), false);
    MO = MachineOperand::CreateReg(RegNo, false);
    MO.setBitWidth(SizeInBits);
    WD.Op = MO;

    if (PredReg) {
      PredReg = createPHI(PredReg, 1, WD.LoopBoundary.getSlot(), false);
      WD.Pred = ucOperand::CreatePredicate(PredReg);
    }

    // Update the register.
    WD.CopySlot = WD.LoopBoundary;
    WD.LoopBoundary += State.getII();
    assert(WD.CopySlot <= ReadSlot && "Broken PHI Slot!");
  }

  // Return the up to date machine operand
  MO = WD.Op;
  MO.setIsUse();
  MO.setImplicit(isImplicit);
  return MO;
}

static inline bool top_sort_start(const VSUnit* LHS, const VSUnit* RHS) {
  if (LHS->getSlot() != RHS->getSlot())
    return LHS->getSlot() < RHS->getSlot();

  return LHS->getIdx() < RHS->getIdx();
}

void VSchedGraph::emitSchedule() {
  unsigned CurSlot = startSlot;
  MachineFunction *MF = MBB->getParent();
  VFInfo *VFI = MF->getInfo<VFInfo>();

  // Fix the schedule of PHI's so we can emit the incoming copies at a right
  // slot;
  for (sched_iterator I = sched_begin(), E = sched_end(); I != E; ++I) {
    VSUnit *U = *I;
    if (!U->isPHI()) continue;

    // Schedule the SU to the slot of the PHI Move.
    U->scheduledTo(U->getSlot() + getII());
    assert(U->getSlot() <= getEndSlot() && "Bad PHI schedule!");
  }

  // Build bundle from schedule units.
  MicroStateBuilder StateBuilder(*this);

  std::sort(begin(), end(), top_sort_start);
  DEBUG(dbgs() << "Sorted AllSUs:\n";
        for (iterator I = begin(), E = end(); I != E; ++I)
          (*I)->dump(););


  for (iterator I = begin(), E = end(); I != E; ++I) {
    VSUnit *A = *I;
    DEBUG(dbgs() << "Going to emit: "; A->dump());
    if (A->getSlot() != CurSlot)
      CurSlot = StateBuilder.advanceToSlot(CurSlot, A->getSlot());

    StateBuilder.emitSUnit(A);
  }
  // Build last state.
  assert(!StateBuilder.emitQueueEmpty() && "Expect atoms for last state!");
  StateBuilder.advanceToSlot(CurSlot, CurSlot + 1);
  // Remove all unused instructions.
  StateBuilder.buildBundles();
  // Build the dummy terminator.
  BuildMI(MBB, DebugLoc(), MF->getTarget().getInstrInfo()->get(VTM::EndState))
    .addImm(0).addImm(0);

  DEBUG(dbgs() << "After schedule emitted:\n");
  DEBUG(dump());
  DEBUG(dbgs() << '\n');
  // Remember the schedule information.
  VFI->rememberTotalSlot(MBB, getStartSlot(), getTotalSlot(), getLoopOpSlot());
}
