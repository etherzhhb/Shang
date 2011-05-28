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
#include "vtm/BitLevelInfo.h"

#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/MachineSSAUpdater.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/ADT/STLExtras.h"
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
  BitLevelInfo &BLI;

  SmallVector<VSUnit*, 8> SUnitsToEmit;
  SmallVector<VSUnit*, 8> DeferredSUnits;
  
  std::vector<MachineInstr*> InstsToDel;
  struct WireDef {
    unsigned WireNum;
    const char *SymbolName;
    unsigned PredReg;
    ucOperand Op;
    unsigned EmitSlot;
    unsigned WriteSlot;

    WireDef(unsigned wireNum, const char *Symbol, unsigned predReg,
            ucOperand op, unsigned emitSlot, unsigned writeSlot)
      : WireNum(wireNum), SymbolName(Symbol), PredReg(predReg), Op(op),
      EmitSlot(emitSlot), WriteSlot(writeSlot) {}

    bool isSymbol() const { return SymbolName != 0; }
    
    MachineOperand getOperand() const { return Op; }

    MachineOperand createOperand() const {
      if (isSymbol()) return MachineOperand::CreateES(SymbolName);

      return ucOperand::CreateWireRead(WireNum, Op.getBitWidth());
    }
  };

  inline WireDef createWireDef(unsigned WireNum, VSUnit *A, MachineOperand MO,
                               unsigned PredReg, unsigned emitSlot,
                               unsigned writeSlot){
    const char *Symbol = 0;
    if (A->getFUId().isBound()) {
      switch (A->getFUType()) {
      case VFUs::MemoryBus:
        Symbol = VFI.allocateSymbol(VFUMemBus::getInDataBusName(A->getFUNum()));
        break;
      case VFUs::BRam:
        Symbol = VFI.allocateSymbol(VFUBRam::getInDataBusName(A->getFUNum()));
        break;
      default:
        assert(0 && "Unexpected FU Type.");
      }
    }

    return WireDef(WireNum, Symbol, PredReg, MO, emitSlot, writeSlot);
  }
  
  typedef std::vector<WireDef*> DefVector;
  SmallVector<DefVector, 32> DefToEmit;
  
  typedef SmallVector<MachineInstr*, 32> MIVector;
  MIVector StateCtrls, StateDatapaths;

  // register number -> wire define.
  typedef std::map<unsigned, WireDef> SWDMapTy;
  SWDMapTy StateWireDefs;

  MicroStateBuilder(VSchedGraph &S, BitLevelInfo &BitInfo)
  : State(S), MBB(*S.getMachineBasicBlock()), InsertPos(MBB.end()),
  TII(*MBB.getParent()->getTarget().getInstrInfo()),
  MRI(MBB.getParent()->getRegInfo()),
  VFI(*MBB.getParent()->getInfo<VFInfo>()), BLI(BitInfo),
  DefToEmit(State.getTotalSlot() + 2 /*Dirty hack: The last slot never use!*/),
  StateCtrls(State.getII() + 1), StateDatapaths(State.getII()) {}

  DefVector &getDefsToEmitAt(unsigned Slot) {
    return DefToEmit[Slot - State.getStartSlot()];
  }

  unsigned getModuloSlot(unsigned Slot, bool IsControl) {
    // FIXME: perform the modulo only if the BB is pipelined.
    unsigned Idx = (Slot -  State.getStartSlot()) % State.getII();
    // Move the entry of non-first stage to the last slot, so
    // Stage 0: Entry,    state1,        ... state(II - 1),
    // Stage 1: stateII,  state(II + 1), ... state(2II - 1),
    // Stage 2: state2II,  state(2II + 1), ... state(3II - 1),
    // become:
    // Stage 0: Entry,    state1,        ... state(II - 1),   stateII,
    // Stage 1:           state(II + 1), ... state(2II - 1),  state2II,
    // Stage 2:           state(2II + 1), ... state(3II - 1),
    if (IsControl && Idx == 0 && Slot >= State.getLoopOpSlot())
      Idx = State.getII();

    return Idx;
  }

  MachineInstr &getStateCtrlAt(unsigned Slot) {
    unsigned Idx = getModuloSlot(Slot, true);
    // Retrieve the instruction at specific slot. 
    MachineInstr *&Ret = StateCtrls[Idx];
    if (Ret) return *Ret;
    // Create the instruction if it is not created yet.
    Ret = (MachineInstr*)
      BuildMI(MBB, InsertPos, DebugLoc(), TII.get(VTM::Control)).addImm(Slot);
    return *Ret;
  }

  MachineInstr &getStateDatapathAt(unsigned Slot) {
    unsigned Idx = getModuloSlot(Slot, false);
    // Retrieve the instruction at specific slot.
    MachineInstr *&Ret = StateDatapaths[Idx];
    if (Ret) return *Ret;
    // Create the instruction if it is not created yet.
    Ret = (MachineInstr*)
      BuildMI(MBB, InsertPos, DebugLoc(), TII.get(VTM::Datapath)).addImm(Slot);

    return *Ret;
  }

  MachineBasicBlock::iterator getInsertPos() { return InsertPos; }

  // Builder interface.
  void emitSUnit(VSUnit *A) { SUnitsToEmit.push_back(A); }
  bool emitQueueEmpty() const { return SUnitsToEmit.empty(); }

  void defereSUnit(VSUnit *A) { DeferredSUnits.push_back(A); }

  // Main state building function.
  MachineInstr *buildMicroState(unsigned Slot);

  void emitDeferredInsts() {
    // Emit the  deferred atoms before data path need it.
    while (!DeferredSUnits.empty()) {
      VSUnit *A = DeferredSUnits.pop_back_val();
      for (VSUnit::instr_iterator I = A->instr_begin(), E = A->instr_end();
          I != E; ++I)
        MBB.insert(InsertPos, *I);
    }
  }

  void fuseInstr(MachineInstr &Inst, VSUnit *A);

  MachineOperand getRegUseOperand(WireDef &WD, unsigned EmitSlot, bool IsCtrl,
                                  MachineOperand MO);

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
  // Try to force create the state control and data path instruction, and
  // insert the instructions between them.
  MachineInstrBuilder CtrlInst(&getStateCtrlAt(Slot));
  emitDeferredInsts();
  if (Slot < State.getLoopOpSlot())  (void) getStateDatapathAt(Slot);

  for (SmallVectorImpl<VSUnit*>::iterator I = SUnitsToEmit.begin(),
       E = SUnitsToEmit.end(); I !=E; ++I) {
    VSUnit *A = *I;
    for (VSUnit::instr_iterator II = A->instr_begin(), IE = A->instr_end();
        II != IE; ++II)
      fuseInstr(**II, A);
  }

  DefVector &DefsAtSlot = getDefsToEmitAt(Slot);
  // Emit the exported registers at current slot.
  for (DefVector::iterator I = DefsAtSlot.begin(), E = DefsAtSlot.end();
       I != E;) {
    WireDef *WD = *I;

    MachineOperand MO = WD->getOperand();

    // This operand will delete with its origin instruction.
    // Eliminate the dead register.
    if (MRI.use_empty(MO.getReg())) {
      I = DefsAtSlot.erase(I);
      E = DefsAtSlot.end();
      continue;
    }

    // Export the register.
    CtrlInst.addOperand(ucOperand::CreateOpcode(VTM::COPY, Slot));
    CtrlInst.addOperand(ucOperand::CreatePredicate(WD->PredReg));
    MO.setIsDef();
    CtrlInst.addOperand(MO).addOperand(WD->createOperand());
    ++I;
  }

  return 0;
}

void MicroStateBuilder::fuseInstr(MachineInstr &Inst, VSUnit *A) {
  VInstr VTID = Inst;
  bool IsCtrl = !VTID.hasDatapath();

  typedef SmallVector<MachineOperand, 8> OperandVector;
  // Add the opcode metadata and the function unit id.
  MachineOperand OpCMD = ucOperand::CreateOpcode(Inst.getOpcode(),
                                                 A->getSlot(), A->getFUId());
  unsigned NumOperands = Inst.getNumOperands();

  // Create the default predicate operand, which means always execute.
  unsigned PredR = 0;

  // Handle the predicate operand.
  if (Inst.getDesc().OpInfo[NumOperands-1].isPredicate()) {
    --NumOperands;
    PredR = Inst.getOperand(NumOperands).getReg();
    Inst.RemoveOperand(NumOperands);
  }
  
  // FIXME: Use pointer operand.
  OperandVector Ops(NumOperands + 1 + IsCtrl, OpCMD);
  Ops[0] = OpCMD;
  if (IsCtrl) Ops[1] = ucOperand::CreatePredicate(PredR);

  unsigned OpStart = IsCtrl ? 2 : 1;

  // Remove all operand of Instr.
  while (Inst.getNumOperands() != 0) {
    unsigned i = Inst.getNumOperands() - 1;
    MachineOperand &MO = Inst.getOperand(i);
    Inst.RemoveOperand(i);
    Ops[i + OpStart] = MO;
  }

  unsigned EmitSlot = A->getSlot(),
           WriteSlot = A->getFinSlot();
  unsigned ReadSlot = EmitSlot;

  bool isReadAtEmit = VTID.isReadAtEmit();

  // We can not write the value to a register at the same moment we emit it.
  // Unless we read at emit.
  // FIXME: Introduce "Write at emit."
  if (WriteSlot == EmitSlot && !isReadAtEmit) ++WriteSlot;
  // Write to register operation need to wait one more slot if the result is
  // written at the moment (clock event) that the atom finish.
  if (VTID.isWriteUntilFinish()) ++WriteSlot;


  // We read the values after we emit it unless the value is read at emit.
  if (!isReadAtEmit) ++ReadSlot;

  DefVector &Defs = getDefsToEmitAt(WriteSlot);

  for (unsigned i = OpStart , e = Ops.size(); i != e; ++i) {
    MachineOperand &MO = Ops[i];

    if (!MO.isReg() || !MO.getReg())
      continue;

    unsigned RegNo = MO.getReg();

    // Remember the defines.
    // DiryHack: Do not emit write define for copy since copy is write at
    // control block.
    if (MO.isDef() && EmitSlot != WriteSlot && !Inst.isCopy()) {
      unsigned BitWidth = cast<ucOperand>(MO).getBitWidth();
      // Do not emit write to register unless it not killed in the current state.
      // FIXME: Emit the wire only if the value is not read in a function unit port.
      // if (!NewDef->isSymbol()) {
      ucOperand NewOp = ucOperand::CreateWireDefine(MRI, BitWidth);
      unsigned WireNum = NewOp.getReg();
      WireDef WDef = createWireDef(WireNum, A, MO, PredR, EmitSlot, WriteSlot);

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

    // Else this is a use.
    SWDMapTy::iterator at = StateWireDefs.find(RegNo);
    // Using register from previous state.
    if (at == StateWireDefs.end()) {
      // Do not need to worry about if the new loop overwrite the the loop
      // invariants.
      continue;
    }

    WireDef &WDef = at->second;

    // We need the value after it is written to register.
    if (WDef.WriteSlot < ReadSlot) {
      assert(((VTID.hasDatapath() && ReadSlot == EmitSlot + 1)
              || (!VTID.hasDatapath() && ReadSlot == EmitSlot))
              && "Assumption of Slots broken!");
      Ops[i] = getRegUseOperand(WDef, EmitSlot, IsCtrl, MO);
      continue;
    }

    assert(WDef.EmitSlot <= ReadSlot && "Dependencies broken!");

    Ops[i] = WDef.createOperand();
  }

  MachineInstrBuilder Builder;

  if (IsCtrl)
    Builder = MachineInstrBuilder(&getStateCtrlAt(A->getSlot()));
  else
    Builder = MachineInstrBuilder(&getStateDatapathAt(A->getSlot()));

  for (OperandVector::iterator I = Ops.begin(), E = Ops.end(); I != E; ++I)
    Builder.addOperand(*I);

  // Remove this instruction since they are fused to uc state.
  InstsToDel.push_back(&Inst);
}

MachineOperand MicroStateBuilder::getRegUseOperand(WireDef &WD, unsigned EmitSlot,
                                                   bool IsCtrl, MachineOperand MO) {
  unsigned RegNo = WD.getOperand().getReg();
  unsigned SizeInBits = cast<ucOperand>(MO).getBitWidth();
  const TargetRegisterClass *RC = MRI.getRegClass(RegNo);

  // Move the value to a new register otherwise the it will be overwritten.
  while (EmitSlot - WD.WriteSlot > State.getII()) {
    MachineInstr &Ctrl = getStateCtrlAt(WD.WriteSlot);
    MachineInstrBuilder CopyBuilder(&Ctrl);
    WD.WriteSlot += State.getII();
    unsigned PipedReg = MRI.createVirtualRegister(RC);
    MachineOperand Dst = MachineOperand::CreateReg(PipedReg, true);
    BLI.updateBitWidth(Dst, SizeInBits);
    MachineOperand Src = MachineOperand::CreateReg(RegNo, false);
    BLI.updateBitWidth(Src, SizeInBits);
    CopyBuilder.addOperand(ucOperand::CreateOpcode(VTM::COPY, WD.WriteSlot));
    CopyBuilder.addOperand(ucOperand::CreatePredicate(WD.PredReg));
    CopyBuilder.addOperand(Dst).addOperand(Src);
    // Update the register.
    RegNo = PipedReg;
    WD.Op = MO = Dst;
  }

  // If read before write in machine code, insert a phi node.
  if (getModuloSlot(EmitSlot, IsCtrl) < getModuloSlot(WD.WriteSlot, true)) {
    // PHI node needed.
    // TODO: Move to constructor?
    MachineSSAUpdater SSAUpdate(*MBB.getParent());
    SSAUpdate.Initialize(RegNo);
    SSAUpdate.AddAvailableValue(&MBB, RegNo);

    // 1. add an initialize value.
    for (MachineBasicBlock::pred_iterator I = MBB.pred_begin(),
        E = MBB.pred_end();I != E; ++I) {
      MachineBasicBlock *PredBB = *I;
      if (PredBB == &MBB) continue;

      // The register to hold initialize value.
      unsigned InitReg = MRI.createVirtualRegister(RC);
      // Get the insert position.
      MachineInstr *PredCtrl = prior(PredBB->getFirstTerminator());
      // Sometimes there are copies between endstate and the real terminator,
      // we need to skip them.
      while (PredCtrl->getOpcode() != VTM::Control) {
        assert(PredCtrl->getOpcode() == TargetOpcode::COPY
               && "Unexpected instruction when finding Terminator!");
        PredCtrl = prior(MachineBasicBlock::iterator(PredCtrl));
      }
      
      MachineInstrBuilder SetIBuilder(PredCtrl);

      // Compute the corresponding slot predicate.
      unsigned EndSlot = VFI.getStartSlotFor(PredBB)
                         + VFI.getTotalSlotFor(PredBB);
      // Add the instruction token.
      SetIBuilder.addOperand(ucOperand::CreateOpcode(VTM::IMPLICIT_DEF,
                                                     EndSlot));
      SetIBuilder.addOperand(ucOperand::CreatePredicate());
      // Build the register operand.
      MachineOperand DstReg = MachineOperand::CreateReg(InitReg, true);
      BLI.updateBitWidth(DstReg, SizeInBits);
      SetIBuilder.addOperand(DstReg);
      SSAUpdate.AddAvailableValue(PredBB, InitReg);
    }

    unsigned NewReg = SSAUpdate.GetValueInMiddleOfBlock(&MBB);
    
    MO = MachineOperand::CreateReg(NewReg, false);
    BLI.updateBitWidth(MO, SizeInBits);
  }

  MO.setIsUse();
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

MachineBasicBlock *VSchedGraph::emitSchedule(BitLevelInfo &BLI) {
  unsigned CurSlot = startSlot;
  MachineFunction *MF = MBB->getParent();
  VFInfo *VFI = MF->getInfo<VFInfo>();
  preSchedTopSort();

  // Build bundle from schedule units.
  MicroStateBuilder BTB(*this, BLI);

  for (iterator I = begin(), E = end(); I != E; ++I) {
    VSUnit *A = *I;

    FuncUnitId FUId = A->getFUId();
    // Remember the active slot.
    if (FUId.isBound())
      VFI->rememberAllocatedFU(FUId, A->getSlot(), A->getFinSlot());

    // Special case: Ret instruction use the function unit "FSMFinish".
    if (A->getOpcode() == VTM::VOpRet)
      VFI->rememberAllocatedFU(VFUs::FSMFinish, A->getSlot(), A->getSlot()+1);

    if (A->getSlot() != CurSlot)
      CurSlot = BTB.advanceToSlot(CurSlot, A->getSlot());

    if (MachineInstr *Inst = A->getFirstInstr()) {
      // Ignore some instructions.
      if (Inst->isPHI()) {
        assert(BTB.emitQueueEmpty() && "Unexpected atom before PHI.");
        continue;
      }

      if (Inst->isCopy() && !VInstr(*Inst).canCopyBeFused()) {
        // TODO: move this to MicroStateBuilder.
        MBB->remove(Inst);
        BTB.defereSUnit(A);
        continue;
      }

      BTB.emitSUnit(A);
    }
  }
  // Build last state.
  assert(!BTB.emitQueueEmpty() && "Expect atoms for last state!");
  BTB.advanceToSlot(CurSlot, CurSlot + 1);
  // Remove all unused instructions.
  BTB.clearUp();
  // Build the dummy terminator.
  BuildMI(MBB, DebugLoc(), TM.getInstrInfo()->get(VTM::EndState)).addImm(0);

  DEBUG(dbgs() << "After schedule emitted:\n");
  DEBUG(dump());
  DEBUG(dbgs() << '\n');

  return MBB;
}

