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

#include "vtm/VFuncInfo.h"
#include "vtm/MicroState.h"
#include "vtm/VTargetMachine.h"
#include "vtm/BitLevelInfo.h"

#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/Support/raw_ostream.h"
#define DEBUG_TYPE "vtm-schedule-emitter"
#include "llvm/Support/Debug.h"

using namespace llvm;

namespace {
// Helper class to build Micro state.
struct MicroStateBuilder {
  MicroStateBuilder(const MicroStateBuilder&);     // DO NOT IMPLEMENT
  void operator=(const MicroStateBuilder&); // DO NOT IMPLEMENT

  VSchedGraph &State;
  MachineBasicBlock &MBB;
  MachineBasicBlock::iterator InsertPos;

  unsigned WireNum;
  LLVMContext &VMContext;
  const TargetInstrInfo &TII;
  MachineRegisterInfo &MRI;
  VFuncInfo &VFI;
  BitLevelInfo &BLI;

  SmallVector<VSUnit*, 8> SUnitsToEmit;
  SmallVector<VSUnit*, 8> DeferredSUnits;
  
  std::vector<MachineInstr*> InstsToDel;
  struct WireDef {
    unsigned WireNum;
    const char *SymbolName;
    MachineOperand *Op;
    unsigned EmitSlot;
    unsigned WriteSlot;

    WireDef(unsigned wireNum, const char *Symbol, MachineOperand *op,
            unsigned emitSlot, unsigned writeSlot)
      : WireNum(wireNum), SymbolName(Symbol), Op(op), EmitSlot(emitSlot),
      WriteSlot(writeSlot) {}

    bool isSymbol() const { return SymbolName != 0; }
    
    MachineOperand *getOperand() const { return Op; }
  };

  inline WireDef createWireDef(unsigned WireNum, VSUnit *A, MachineOperand *MO,
                               unsigned OpNum, unsigned emitSlot,
                               unsigned writeSlot){
    const char *Symbol = 0;
    if (A->getFUId().isBinded()) {
      assert(A->getFUType() == VFUs::MemoryBus
             && "Only support Membus at this moment!");
      assert(OpNum == 0 && "Bad Operand!");
      Symbol = VFI.allocateSymbol(VFUMemBus::getInDataBusName(A->getFUNum()));
    }
    
    return WireDef(WireNum, Symbol, MO, emitSlot, writeSlot);
  }
  
  typedef std::vector<WireDef*> DefVector;
  SmallVector<DefVector, 32> DefToEmit;
  
  typedef SmallVector<MachineInstr*, 32> MIVector;
  MIVector StateCtrls, StateDatapaths;

  // register number -> wire define.
  typedef std::map<unsigned, WireDef> SWDMapTy;
  SWDMapTy StateWireDefs;

  MicroStateBuilder(VSchedGraph &S, LLVMContext& Context, const VTargetMachine &TM,
                    BitLevelInfo &BitInfo)
  : State(S), MBB(*S.getMachineBasicBlock()), InsertPos(MBB.end()),
  WireNum(MBB.getNumber()),
  VMContext(Context), TII(*TM.getInstrInfo()),
  MRI(MBB.getParent()->getRegInfo()),
  VFI(*MBB.getParent()->getInfo<VFuncInfo>()), BLI(BitInfo),
  DefToEmit(State.getTotalSlot() + 2 /*Dirty hack: The last slot never use!*/),
  StateCtrls(State.getTotalSlot() + 1), StateDatapaths(State.getTotalSlot()) {}

  DefVector &getDefsToEmitAt(unsigned Slot) {
    return DefToEmit[Slot - State.getStartSlot()];
  }

  unsigned computeStateIdx(unsigned Slot) {
    return Slot -  State.getStartSlot();
  }

  MachineInstr &getStateCtrlAt(unsigned Slot) {
    unsigned Idx = computeStateIdx(Slot);
    // Retrieve the instruction at specific slot. 
    MachineInstr *&Ret = StateCtrls[Idx];
    if (Ret) return *Ret;
    // Create the instruction if it is not created yet.
    const TargetInstrDesc &TID = (Idx == State.getTotalSlot()) ?
                                  TII.get(VTM::Terminator)
                                : TII.get(VTM::Control);

    Ret = (MachineInstr*)BuildMI(MBB, InsertPos, DebugLoc(), TID).addImm(Slot);
    return *Ret;
  }

  MachineInstr &getStateDatapathAt(unsigned Slot) {
    unsigned Idx = computeStateIdx(Slot);
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
  MachineInstr *buildMicroState(unsigned Slot, bool IsLastSlot = false);

  void emitDeferredInsts() {
    // Emit the  deferred atoms before data path need it.
    while (!DeferredSUnits.empty()) {
      VSUnit *A = DeferredSUnits.pop_back_val();
      for (VSUnit::instr_iterator I = A->instr_begin(), E = A->instr_end();
          I != E; ++I)
        MBB.insert(InsertPos, *I);
    }
  }

  void fuseInstr(MachineInstr &Inst, VSUnit *A, bool IsLastSlot);

  unsigned advanceToSlot(unsigned CurSlot, unsigned TargetSlot,
                         bool IsLastSlot = false) {
    assert(TargetSlot > CurSlot && "Bad target slot!");
    
    buildMicroState(CurSlot, IsLastSlot);
    SUnitsToEmit.clear();
    
    // Some states may not emit any atoms, but it may read the result from
    // previous atoms.
    // Note that SUnitsToEmit is empty now, so we do not emitting any new
    // atoms.
    while (++CurSlot != TargetSlot && !IsLastSlot)
      buildMicroState(CurSlot);

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

MachineInstr*
MicroStateBuilder::buildMicroState(unsigned Slot, bool IsLastSlot) {
  // Try to force create the state control and data path instruction, and
  // insert the instructions between them.
  MachineInstrBuilder CtrlInst(&getStateCtrlAt(Slot));
  emitDeferredInsts();
  if (!IsLastSlot)  (void) getStateDatapathAt(Slot);

  for (SmallVectorImpl<VSUnit*>::iterator I = SUnitsToEmit.begin(),
       E = SUnitsToEmit.end(); I !=E; ++I) {
    VSUnit *A = *I;
    for (VSUnit::instr_iterator II = A->instr_begin(), IE = A->instr_end();
        II != IE; ++II)
      fuseInstr(**II, A, IsLastSlot);
  }

  DefVector &DefsAtSlot = getDefsToEmitAt(Slot);
  // Emit the exported registers at current slot.
  for (DefVector::iterator I = DefsAtSlot.begin(), E = DefsAtSlot.end();
       I != E;) {
    WireDef *WD = *I;

    MachineOperand *MO = WD->getOperand();

    // This operand will delete with its origin instruction.
    // Eliminate the dead register.
    if (MRI.use_empty(MO->getReg())) {
      I = DefsAtSlot.erase(I);
      E = DefsAtSlot.end();
      continue;
    }

    // Export the register.
    CtrlInst.addMetadata(MetaToken::createInstr(Slot, VTM::COPY, VMContext));
    CtrlInst.addOperand(*MO);
    CtrlInst.addMetadata(MetaToken::createReadWire(WD->WireNum, VMContext));
    ++I;
  }

  return 0;
}

void MicroStateBuilder::fuseInstr(MachineInstr &Inst, VSUnit *A, bool IsLastSlot) {
  VTFInfo VTID = Inst;
  MachineInstrBuilder Builder;

  if (VTID.hasDatapath())
    Builder = MachineInstrBuilder(&getStateDatapathAt(A->getSlot()));
  else
    Builder = MachineInstrBuilder(&getStateCtrlAt(A->getSlot()));

  // Add the opcode metadata and the function unit id.
  Builder.addMetadata(MetaToken::createInstr(A->getSlot(), Inst, A->getFUNum(),
                                             VMContext));
  typedef SmallVector<MachineOperand*, 8> OperandVector;
  OperandVector Ops(Inst.getNumOperands());

  // Remove all operand of Instr.
  while (Inst.getNumOperands() != 0) {
    unsigned i = Inst.getNumOperands() - 1;
    MachineOperand *MO = &Inst.getOperand(i);
    Inst.RemoveOperand(i);
    Ops[i] = MO;
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


  for (unsigned i = 0, e = Ops.size(); i != e; ++i) {
    MachineOperand *MO = Ops[i];

    if (!MO->isReg() || !MO->getReg()) {
      Builder.addOperand(*MO);
      continue;
    }

    unsigned RegNo = MO->getReg();

    // Remember the defines.
    if (MO->isDef() && EmitSlot != WriteSlot) {
      ++WireNum;
      WireDef WDef = createWireDef(WireNum, A, MO, i, EmitSlot, WriteSlot);

      SWDMapTy::iterator mapIt;
      bool inserted;
      tie(mapIt, inserted) = StateWireDefs.insert(std::make_pair(RegNo, WDef));

      assert(inserted && "Instructions not in SSA form!");
      WireDef *NewDef = &mapIt->second;

      unsigned BitWidth = BLI.getBitWidth(*MO);

      // Do not emit write to register unless it not killed in the current state.
      // FIXME: Emit the wire only if the value is not read in a function unit port.
      // if (!NewDef->isSymbol()) {
        MDNode *WireDefOp = MetaToken::createDefWire(WireNum, BitWidth, VMContext);
        Builder.addMetadata(WireDefOp);
        // Remember to emit this wire define if necessary.
        Defs.push_back(NewDef);
      // }
      continue;
    }

    // Else this is a use.
    SWDMapTy::iterator at = StateWireDefs.find(RegNo);
    // Using regster from previous state.
    if (at == StateWireDefs.end()) {
      Builder.addOperand(*MO);
      continue;
    }

    WireDef &WDef = at->second;

    // We need the value after it is written to register.
    if (WDef.WriteSlot < ReadSlot) {
      Builder.addOperand(*MO);
      continue;
    }

    assert(WDef.EmitSlot <= ReadSlot && "Dependencies broken!");

    if (WDef.isSymbol())
      Builder.addExternalSymbol(WDef.SymbolName);
    else
      Builder.addMetadata(MetaToken::createReadWire(WDef.WireNum, VMContext));
  }

  // Remove this instruction since they are fused to uc state.
  InstsToDel.push_back(&Inst);
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
  VFuncInfo *VFI = MBB->getParent()->getInfo<VFuncInfo>();

  preSchedTopSort();

  // Build bundle from schedule units.
  MicroStateBuilder BTB(*this, MBB->getBasicBlock()->getContext(), TM, BLI);

  for (iterator I = begin(), E = end(); I != E; ++I) {
    VSUnit *A = *I;

    FuncUnitId FUId = A->getFUId();
    // Remember the active slot.
    if (FUId.isBinded())
      VFI->rememberAllocatedFU(FUId, A->getSlot(), A->getFinSlot());

    // Special case: Ret instruction use the function unit "FSMFinish".
    if (A->getOpcode() == VTM::VOpRet)
      VFI->rememberAllocatedFU(VFUs::FSMFinish, A->getSlot(), A->getSlot()+1);

    if (A->getSlot() != CurSlot)
      CurSlot = BTB.advanceToSlot(CurSlot, A->getSlot());

    if (MachineInstr *Inst = A->getFirstInstr()) {
      // Ignore some instructions.
      switch (Inst->getOpcode()) {
      case TargetOpcode::PHI:
        assert(BTB.emitQueueEmpty() && "Unexpected atom before PHI.");
        // Do not touch the PHIs, leave them at the beginning of the BB.
        continue;
      case TargetOpcode::COPY:
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
  BTB.advanceToSlot(CurSlot, CurSlot + 1, true);
  // Remove all unused instructions.
  BTB.clearUp();

  DEBUG(dbgs() << "After schedule emitted:\n");
  DEBUG(dump());

  return MBB;
}

