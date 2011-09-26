//===----- ScheduleEmitter.cpp - Emit the schedule  -------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
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
// Helper class to build Micro state.
struct MicroStateBuilder {
  MicroStateBuilder(const MicroStateBuilder&);     // DO NOT IMPLEMENT
  void operator=(const MicroStateBuilder&); // DO NOT IMPLEMENT

  const VSchedGraph &State;
  MachineBasicBlock &MBB;
  MachineBasicBlock::iterator InsertPos;

  const TargetInstrInfo &TII;
  MachineRegisterInfo &MRI;
  VFInfo &VFI;

  SmallVector<VSUnit*, 8> SUnitsToEmit;
  
  std::vector<MachineInstr*> InstsToDel;

  struct WireDef {
    unsigned WireNum;
    ucOperand Pred;
    ucOperand Op;
    OpSlot DefSlot;
    OpSlot CopySlot;
    OpSlot LoopBoundary;

    WireDef(unsigned wireNum, MachineOperand &pred, MachineOperand &op,
            OpSlot defSlot, OpSlot copySlot, OpSlot loopBoundary)
      : WireNum(wireNum), Pred(pred), Op(op), DefSlot(defSlot),
      CopySlot(copySlot), LoopBoundary(loopBoundary) {}

    // Do not define a register twice by copying it self.
    bool shouldBeCopied() const {
      return (!Op.isReg() || WireNum != Op.getReg());
    }

    MachineOperand getOperand() const { return Op; }

    MachineOperand createOperand() const {
      return ucOperand::CreateWire(WireNum, Op.getBitWidth());
    }
  };

  inline WireDef createWireDef(unsigned WireNum, MachineOperand &MO,
                               MachineOperand &Pred, OpSlot defSlot,
                               OpSlot copySlot){
    assert(copySlot.isControl() && "Can only copy at control!");
    unsigned Slot = copySlot.getSlot();
    unsigned II = State.getII();
    Slot -= State.getStartSlot();
    Slot = RoundUpToAlignment(Slot, II);
    Slot += State.getStartSlot();
    return WireDef(WireNum, Pred, MO, defSlot, copySlot, OpSlot(Slot, true));
  }
  
  typedef std::vector<WireDef*> DefVector;
  
  typedef SmallVector<MachineInstr*, 32> MIVector;
  MIVector StateCtrls, StateDatapaths;

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
    unsigned EndSlot = State.getStartSlot()+ State.getII();
    for (unsigned i = State.getStartSlot(), e = EndSlot; i != e; ++i) {
      MachineInstr *Ctrl =
        BuildMI(MBB, InsertPos, DebugLoc(), TII.get(VTM::Control)).addImm(i);
      StateCtrls.push_back(Ctrl);
      MachineInstr *DP =
        BuildMI(MBB, InsertPos, DebugLoc(), TII.get(VTM::Datapath)).addImm(i);
      StateDatapaths.push_back(DP);
    }

    MachineInstr *TermCtrl =
      BuildMI(MBB, InsertPos, DebugLoc(), TII.get(VTM::Control)).addImm(EndSlot);
    StateCtrls.push_back(TermCtrl);
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
    return (WD.LoopBoundary < ReadSlot);
  }

  MachineInstr &getStateCtrlAt(OpSlot CtrlSlot) {
    unsigned Idx = getModuloSlot(CtrlSlot);
    // Retrieve the instruction at specific slot. 
    MachineInstr *Ret = StateCtrls[Idx];
    assert(Ret && "Unexpected NULL instruction!");
    return *Ret;
  }

  MachineInstr &getStateDatapathAt(OpSlot DatapathSlot) {
    unsigned Idx = getModuloSlot(DatapathSlot);
    // Retrieve the instruction at specific slot.
    MachineInstr *Ret = StateDatapaths[Idx];
    assert(Ret && "Unexpected NULL instruction!");
    return *Ret;
  }

  MachineInstr &getMIAt(OpSlot Slot) {
    if (Slot.isControl()) return getStateCtrlAt(Slot);
    else                  return getStateDatapathAt(Slot);
  }

  MachineBasicBlock::iterator getInsertPos() { return InsertPos; }

  // Builder interface.
  void emitSUnit(VSUnit *A) { SUnitsToEmit.push_back(A); }
  bool emitQueueEmpty() const { return SUnitsToEmit.empty(); }

  // Main state building function.
  MachineInstr *buildMicroState(unsigned Slot);

  void fuseInstr(MachineInstr &Inst, OpSlot SchedSlot, FuncUnitId FUId);

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


  MachineOperand getRegUseOperand(WireDef &WD, OpSlot ReadSlot, ucOperand MO);

  ucOperand buildCopy(OpSlot CopySlot, unsigned SizeInBits, unsigned RegNo,
                     MachineOperand Pred) {
    unsigned PipedReg = MRI.createVirtualRegister(VTM::DRRegisterClass);
    MachineInstr &Ctrl = getStateCtrlAt(CopySlot);
    MachineInstrBuilder CopyBuilder(&Ctrl);
    ucOperand Dst = ucOperand::CreateReg(PipedReg, SizeInBits, true);
    ucOperand Src = ucOperand::CreateWire(RegNo, SizeInBits);
    MachineOperand Opc =
      ucOperand::CreateOpcode(VTM::VOpMove_rw, CopySlot.getSlot());
    CopyBuilder.addOperand(Opc).addOperand(Pred).addOperand(Dst).addOperand(Src);
    return Dst;
  }


  unsigned createPHI(unsigned RegNo, unsigned SizeInBits, unsigned WriteSlot) {
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
      unsigned InitReg = MRI.createVirtualRegister(MRI.getRegClass(RegNo));

      BuildMI(*PredBB, PredBB->getFirstTerminator(), DebugLoc(),
        TII.get(VTM::IMPLICIT_DEF), InitReg);
      SSAUpdate.AddAvailableValue(PredBB, InitReg);
    }

    unsigned NewReg = SSAUpdate.GetValueInMiddleOfBlock(&MBB);

    // Update the bitwidth for newly inserted PHIs, insert it into the
    // First ucSate.
    while (!InsertedPHIs.empty()) {
      MachineInstr *PN = InsertedPHIs.pop_back_val();
      ucOperand &Op = cast<ucOperand>(PN->getOperand(0));
      Op.setBitWidth(SizeInBits);

      VFI.rememberPHISlot(PN, WriteSlot, true);
      for (unsigned i = 1; i != PN->getNumOperands(); i += 2) {
        ucOperand &SrcOp = cast<ucOperand>(PN->getOperand(i));
        SrcOp.setBitWidth(SizeInBits);
      }
    }

    return NewReg;
  }


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

  // Clean up the basic block by remove all unused instructions.
  void clearUp() {
    while (!InstsToDel.empty()) {
      InstsToDel.back()->eraseFromParent();
      InstsToDel.pop_back();
    }
  }
};
}

//===----------------------------------------------------------------------===//

MachineInstr* MicroStateBuilder::buildMicroState(unsigned Slot) {
  for (SmallVectorImpl<VSUnit*>::iterator I = SUnitsToEmit.begin(),
       E = SUnitsToEmit.end(); I !=E; ++I) {
    VSUnit *A = *I;

    OpSlot SchedSlot(A->getSlot(), !A->hasDatapath());
    MachineInstr *RepInst = A->getRepresentativeInst();
    // Handle representative instruction of the VSUnit.
    if (RepInst != 0 && !RepInst->isPHI())
      fuseInstr(*RepInst, SchedSlot, A->getFUId());

    // And other trivially merged instructions.
    const VSUnit::instr_iterator InstrBase = A->instr_begin();
    for (VSUnit::instr_iterator II = InstrBase + 1, IE = A->instr_end();
         II != IE; ++II) {
      MachineInstr &Inst = **II;
      VIDesc VTID = Inst;

      // The instructions in Exit Root are parallel.
      unsigned DetailStep = SchedSlot.getDetailStep();
      DetailStep += A->getLatencyAt(II - InstrBase);

      OpSlot S = OpSlot::detailStepCeil(DetailStep, VTID.hasDatapath());
      // FIXME: Assert the instruction have trivial function unit.
      fuseInstr(Inst, S, FuncUnitId());
    }
  }

  return 0;
}

void MicroStateBuilder::fuseInstr(MachineInstr &Inst, OpSlot SchedSlot,
                                  FuncUnitId FUId) {
  VIDesc VTID = Inst;
  bool IsCtrl = !VTID.hasDatapath();
  bool IsCtrlSlot = SchedSlot.isControl();
  assert(IsCtrlSlot == IsCtrl && "Wrong slot type.");
  bool isCopyLike = VInstrInfo::isCopyLike(Inst.getOpcode());

  typedef SmallVector<MachineOperand, 8> OperandVector;
  // Add the opcode metadata and the function unit id.
  MachineOperand OpCMD = ucOperand::CreateOpcode(Inst.getOpcode(),
                                                 SchedSlot.getSlot(), FUId);

  // Create the default predicate operand, which means always execute.
  ucOperand Pred = ucOperand::CreatePredicate();

  // Handle the predicate operand.
  if (MachineOperand *PredOp = VInstrInfo::getPredOperand(&Inst)) {
    assert(PredOp->isReg() && "Cannot handle predicate operand!");
    Pred = *PredOp;

    if (Pred.getReg() && VRegisterInfo::IsWire(Pred.getReg(), &MRI))
      Pred.setIsWire();

    unsigned PredIdx = PredOp - &Inst.getOperand(0);
    Inst.RemoveOperand(PredIdx);
  }

  unsigned NumOperands = Inst.getNumOperands();
  // FIXME: Use pointer operand.
  OperandVector Ops(NumOperands + 1 + IsCtrlSlot, OpCMD);
  Ops[0] = OpCMD;
  if (IsCtrlSlot) Ops[1] = Pred;

  unsigned OpStart = IsCtrlSlot ? 2 : 1;

  // Remove all operand of Instr.
  while (Inst.getNumOperands() != 0) {
    unsigned i = Inst.getNumOperands() - 1;
    MachineOperand &MO = Inst.getOperand(i);
    Inst.RemoveOperand(i);
    Ops[i + OpStart] = MO;
  }

  //bool isReadAtEmit = VTID.isReadAtEmit();
  //OpSlot DefSlot(A->getSlot(), IsCtrl);
  OpSlot ReadSlot = SchedSlot;
  //if (!isReadAtEmit) ReadSlot = EmitSlot.getNextSlot();

  unsigned FinSlot = SchedSlot.getSlot() + VTID.getLatency();
  OpSlot CopySlot(FinSlot, true);
  // We can not write the value to a register at the same moment we emit it.
  // Unless we read at emit.
  // FIXME: Introduce "Write at emit."
  if (CopySlot < SchedSlot)
    ++CopySlot;
  // Write to register operation need to wait one more slot if the result is
  // written at the moment (clock event) that the atom finish.
 if (VTID.isWriteUntilFinish()) ++CopySlot;

  DefVector Defs;

  for (unsigned i = 1 , e = Ops.size(); i != e; ++i) {
    MachineOperand &MO = Ops[i];

    if (!MO.isReg() || !MO.getReg())
      continue;

    unsigned RegNo = MO.getReg();
    // Set the wire flag now.
    if (VRegisterInfo::IsWire(RegNo, &MRI))
      cast<ucOperand>(MO).setIsWire();

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
      if (!NewOp.isWire()) {
        WireNum =
          MRI.createVirtualRegister(VFUs::getRepRegisterClass(VTID.getFUType()));
        NewOp = ucOperand::CreateWire(WireNum, BitWidth, true);
      }

      // If the wire define and the copy wrap around?
      if (getModuloSlot(SchedSlot) > getModuloSlot(CopySlot))
        WireNum = createPHI(WireNum, BitWidth, SchedSlot.getSlot());

      WireDef WDef = createWireDef(WireNum, MO, Pred, SchedSlot, CopySlot);

      SWDMapTy::iterator mapIt;
      bool inserted;
      tie(mapIt, inserted) = StateWireDefs.insert(std::make_pair(RegNo, WDef));

      assert(inserted && "Instructions not in SSA form!");
      WireDef *NewDef = &mapIt->second;
      // Remember to emit this wire define if necessary.
      Defs.push_back(NewDef);

      // Update the operand.
      Ops[i] = NewOp;
      // }
      continue;
    }

    Ops[i] = getRegUseOperand(MO, ReadSlot);
  }

  // Remember the predicate of the function unit enable signal.
  if (IsCtrl) {
    // FIXME: Iterate over the live time of the FU and get the right predicate
    // operand with getRegUseOperand.
    // Remember the active slot.
    if (FUId.isBound())
      VFI.rememberAllocatedFU(FUId, SchedSlot.getSlot(), FinSlot, Ops[1]);
  }

  MachineInstrBuilder Builder(&getMIAt(SchedSlot));

  for (OperandVector::iterator I = Ops.begin(), E = Ops.end(); I != E; ++I)
    Builder.addOperand(*I);

  // Emit the exported registers at current slot.
  while (!Defs.empty()) {
    WireDef *WD = Defs.back();
    Defs.pop_back();

    MachineOperand MO = WD->getOperand();

    // FIXME: Do we need this?
    // This operand will delete with its origin instruction.
    // Eliminate the dead register.
    //if (MRI.use_empty(MO.getReg()))
    //  continue;

    MachineInstrBuilder CtrlInst(&getStateCtrlAt(CopySlot));
    if (WD->shouldBeCopied()) {
      // Export the register.
      CtrlInst.addOperand(ucOperand::CreateOpcode(VTM::COPY, CopySlot.getSlot()));
      // Get the operand at current slot.
      CtrlInst.addOperand(getRegUseOperand(WD->Pred, CopySlot));
      MO.setIsDef();
      MachineOperand Src = WD->createOperand();
      CtrlInst.addOperand(MO).addOperand(Src);
    }
  }

  // Remove this instruction since they are fused to uc state.
  InstsToDel.push_back(&Inst);
}

MachineOperand MicroStateBuilder::getRegUseOperand(WireDef &WD, OpSlot ReadSlot,
                                                   ucOperand MO) {
  bool isImplicit = MO.isImplicit();
  unsigned RegNo = WD.getOperand().getReg();
  unsigned PredReg = WD.Pred.getReg();
  unsigned SizeInBits = WD.Op.getBitWidth();
  bool IsWireIncoming = VRegisterInfo::IsWire(RegNo, &MRI);

  // Move the value to a new register otherwise the it will be overwritten.
  // If read before write in machine code, insert a phi node.
  while (isReadWrapAround(ReadSlot, WD)) {
    // Because the result of wireops will be copied to register at loop boundary
    // only extend the live interval of its operand to the first loop boundary.
    if (isImplicit && WD.LoopBoundary > WD.DefSlot + State.getII())
      break;

    //assert(!isImplicit && "Unexpected implicit operand!");
    if (IsWireIncoming) {
      WD.Op = MO = buildCopy(WD.CopySlot, SizeInBits, RegNo, WD.Pred);
      RegNo = MO.getReg();

      if (PredReg) {
        WD.Pred = buildCopy(WD.CopySlot, 1, PredReg,
                            ucOperand::CreatePredicate());
        PredReg = WD.Pred.getReg();
      }

      // Not wire anymore.
      IsWireIncoming = false;
    }

    // Emit the PHI at loop boundary
    RegNo = createPHI(RegNo, SizeInBits, WD.LoopBoundary.getSlot());
    MO = MachineOperand::CreateReg(RegNo, false);
    MO.setBitWidth(SizeInBits);
    WD.Op = MO;

    if (PredReg) {
      PredReg = createPHI(PredReg, 1, WD.LoopBoundary.getSlot());
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

void VSchedGraph::preSchedTopSort() {
  std::sort(SUnits.begin(), SUnits.end(), top_sort_start);
}

void VSchedGraph::emitSchedule() {
  unsigned CurSlot = startSlot;
  MachineFunction *MF = MBB->getParent();
  VFInfo *VFI = MF->getInfo<VFInfo>();
  preSchedTopSort();

  // Build bundle from schedule units.
  MicroStateBuilder StateBuilder(*this);

  for (iterator I = begin(), E = end(); I != E; ++I) {
    VSUnit *A = *I;
    // Special case: Ret instruction use the function unit "FSMFinish".
    if (A->getOpcode() == VTM::VOpRet)
      VFI->rememberAllocatedFU(VFUs::FSMFinish, A->getSlot(), A->getSlot()+1,
                               ucOperand::CreatePredicate());

    if (A->getSlot() != CurSlot)
      CurSlot = StateBuilder.advanceToSlot(CurSlot, A->getSlot());

    if (MachineInstr *Inst = A->getRepresentativeInst()) {
      // Ignore some instructions.
      if (Inst->isPHI()) {
        // For a loop, the PHI is copying the value from previous iteration, so
        // we need to issue after first iteration is done.
        unsigned PHISlot = A->getSlot() + getII();
        VFI->rememberPHISlot(Inst, PHISlot);
      }

      assert((!Inst->isCopy() || VIDesc(*Inst).canCopyBeFused())
             && "Cannot handle copy!");
    }

    StateBuilder.emitSUnit(A);
  }
  // Build last state.
  assert(!StateBuilder.emitQueueEmpty() && "Expect atoms for last state!");
  StateBuilder.advanceToSlot(CurSlot, CurSlot + 1);
  // Remove all unused instructions.
  StateBuilder.clearUp();
  // Build the dummy terminator.
  BuildMI(MBB, DebugLoc(), TM.getInstrInfo()->get(VTM::EndState)).addImm(0);

  DEBUG(dbgs() << "After schedule emitted:\n");
  DEBUG(dump());
  DEBUG(dbgs() << '\n');

  // Remember the schedule information.

  VFI->rememberTotalSlot(MBB, getStartSlot(), getTotalSlot(), getLoopOpSlot());
}
