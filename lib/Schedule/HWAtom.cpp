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
// This file implement the HWAtom class, which represent the basic atom
// operation in hardware.
//
//===----------------------------------------------------------------------===//

#include "HWAtom.h"
#include "ForceDirectedScheduling.h"
#include "ScheduleDOT.h"

#include "vtm/MicroState.h"
#include "vtm/VFuncInfo.h"
#include "vtm/VTargetMachine.h"
#include "vtm/BitLevelInfo.h"

#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Analysis/LoopInfo.h"

#define DEBUG_TYPE "vtm-sunit"
#include "llvm/Support/Debug.h"

using namespace llvm;

namespace {
// Helper class to build Micro state.
struct MicroStateBuilder {
  MicroStateBuilder(const MicroStateBuilder&);     // DO NOT IMPLEMENT
  void operator=(const MicroStateBuilder&); // DO NOT IMPLEMENT

  FSMState &State;
  MachineBasicBlock &MBB;
  MachineBasicBlock::iterator InsertPos;

  unsigned WireNum;
  unsigned OpId;
  LLVMContext &VMContext;
  const VTargetMachine &Target;
  const TargetInstrInfo &TII;
  MachineRegisterInfo &MRI;
  VFuncInfo &VFI;
  BitLevelInfo &BLI;

  SmallVector<HWAtom*, 8> AtomsToEmit;
  SmallVector<HWAtom*, 8> DeferredAtoms;
  
  std::vector<MachineInstr*> InstsToDel;
  struct WireDef {

    unsigned WireNum;
    const char *SymbolName;
    PointerIntPair<MachineOperand*, 1, bool> Op;
    unsigned EmitSlot;
    unsigned WriteSlot;

    WireDef(unsigned wireNum, const char *Symbol, MachineOperand *op,
            unsigned emitSlot, unsigned writeSlot)
      : WireNum(wireNum), SymbolName(Symbol), Op(op, false), EmitSlot(emitSlot),
      WriteSlot(writeSlot) {}

    bool isSymbol() const { return SymbolName != 0; }
    bool isKilled() const { return Op.getInt(); }
    void applyKilled(bool SetKilled) { Op.setInt(Op.getInt() || SetKilled); }
    MachineOperand *getOperand() const { return Op.getPointer(); }
  };

  inline WireDef createWireDef(unsigned WireNum, HWAtom *A,
                               MachineOperand *MO, unsigned OpNum,
                               unsigned emitSlot, unsigned writeSlot){
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
  std::vector<DefVector> DefToEmit;
  
  // register number -> wire define.
  typedef std::map<unsigned, WireDef> SWDMapTy;
  SWDMapTy StateWireDefs;

  MicroStateBuilder(FSMState &S, LLVMContext& Context, const VTargetMachine &TM,
                    BitLevelInfo &BitInfo)
  : State(S), MBB(*S.getMachineBasicBlock()), InsertPos(MBB.end()),
  WireNum(MBB.getNumber()),
  OpId(MBB.getNumber() << 24),
  VMContext(Context), Target(TM), TII(*TM.getInstrInfo()),
  MRI(MBB.getParent()->getRegInfo()),
  VFI(*MBB.getParent()->getInfo<VFuncInfo>()),
  DefToEmit(State.getTotalSlot() + 1 /*Dirty hack: The last slot never use!*/),
  BLI(BitInfo) {}

  ~MicroStateBuilder() {
    while (!InstsToDel.empty()) {
      InstsToDel.back()->eraseFromParent();
      InstsToDel.pop_back();
    }
  }

  DefVector &getDefsToEmitAt(unsigned Slot) {
    return DefToEmit[Slot - State.getStartSlot()];
  }

  MachineBasicBlock::iterator getInsertPos() { return InsertPos; }

  void emitAtom(HWAtom *A) { AtomsToEmit.push_back(A); }
  bool emitQueueEmpty() const { return AtomsToEmit.empty(); }

  void defereAtom(HWAtom *A) { DeferredAtoms.push_back(A); }

  MachineInstr *buildMicroState(unsigned Slot, bool IsLastSlot = false);

  void emitDeferredInsts() {
    // Emit the  deferred atoms before data path need it.
    while (!DeferredAtoms.empty()) {
      HWAtom *A = DeferredAtoms.pop_back_val();
      for (HWAtom::instr_iterator I = A->instr_begin(), E = A->instr_end();
          I != E; ++I)
        MBB.insert(InsertPos, *I);
    }
  }

  void fuseInstr(MachineInstr &Inst, HWAtom *A, bool IsLastSlot,
                 MachineInstrBuilder &DPInst, MachineInstrBuilder &CtrlInst);

  unsigned advanceToSlot(unsigned CurSlot, unsigned TargetSlot,
                         bool IsLastSlot = false) {
    assert(TargetSlot > CurSlot && "Bad target slot!");
    
    buildMicroState(CurSlot, IsLastSlot);
    AtomsToEmit.clear();
    
    // Some states may not emit any atoms, but it may read the result from
    // previous atoms.
    // Note that AtomsToEmit is empty now, so we do not emitting any new
    // atoms.
    while (++CurSlot != TargetSlot && !IsLastSlot)
      buildMicroState(CurSlot);

    return CurSlot;
  }

};
}

//===----------------------------------------------------------------------===//

MachineInstr*
MicroStateBuilder::buildMicroState(unsigned Slot, bool IsLastSlot) {
  MachineBasicBlock &MBB = *State.getMachineBasicBlock();

  const TargetInstrDesc &TID = IsLastSlot ? TII.get(VTM::Terminator)
                                          : TII.get(VTM::Control);
  MachineInstrBuilder CtrlInst
    = BuildMI(MBB, InsertPos, DebugLoc(), TID).addImm(Slot);

  emitDeferredInsts();


  MachineInstrBuilder DPInst;
  if (!IsLastSlot) {
    DPInst
      = BuildMI(MBB, InsertPos, DebugLoc(), TII.get(VTM::Datapath)).addImm(Slot);
  }

  for (SmallVectorImpl<HWAtom*>::iterator I = AtomsToEmit.begin(),
       E = AtomsToEmit.end(); I !=E; ++I) {
    HWAtom *A = *I;
    for (HWAtom::instr_iterator II = A->instr_begin(), IE = A->instr_end();
        II != IE; ++II)
      fuseInstr(**II, A, IsLastSlot, DPInst, CtrlInst);
  }

  DefVector &DefsAtSlot = getDefsToEmitAt(Slot);
  // Emit the exported registers at current slot.
  for (DefVector::iterator I = DefsAtSlot.begin(), E = DefsAtSlot.end();
       I != E; ++I) {
    WireDef *WD = *I;

    // This operand will delete with its origin instruction.
    if (WD->isKilled()) {
      // FIXME: Tell someone this register is dead.
      continue;
    }

    MachineOperand *MO = WD->getOperand();

    // Eliminate the dead register.
    if (MO->isDead()) continue;
    
    // Export the register.
    CtrlInst.addMetadata(MetaToken::createDefReg(++OpId, WD->WireNum, VMContext));
    CtrlInst.addOperand(*MO);
  }

  return 0;
}

void MicroStateBuilder::fuseInstr(MachineInstr &Inst, HWAtom *A, bool IsLastSlot,
                                  MachineInstrBuilder &DPInst,
                                  MachineInstrBuilder &CtrlInst) {
  VTFInfo VTID = Inst.getDesc();
  // FIXME: Inline datapath is allow in last slot.
  assert(!(IsLastSlot && VTID.hasDatapath())
    && "Unexpect datapath in last slot!");
  MachineInstrBuilder &Builder = VTID.hasDatapath() ? DPInst : CtrlInst;

  // Add the opcode metadata and the function unit id.
  Builder.addMetadata(MetaToken::createInstr(++OpId, Inst, A->getFUNum(),
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

      std::pair<SWDMapTy::iterator, bool> result =
        StateWireDefs.insert(std::make_pair(RegNo, WDef));

      assert(result.second && "Instructions not in SSA form!");
      WireDef *NewDef = &result.first->second;
      // Remember to emit this wire define if necessary.
      Defs.push_back(NewDef);
      unsigned BitWidth = BLI.getBitWidth(*MO);
      // Do not emit define unless it not killed in the current state.
      // Emit a wire define instead.
      Builder.addMetadata(MetaToken::createDefWire(WireNum, BitWidth,
        VMContext));
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

    // We need the value after it is writed to register.
    if (WDef.WriteSlot < ReadSlot) {
      Builder.addOperand(*MO);
      continue;
    }

    assert(WDef.EmitSlot <= ReadSlot && "Dependencies broken!");
    // Now we emit an operation while the value are computing, just read
    // the value from a wire.
    WDef.applyKilled(MO->isKill());

    if (WDef.isSymbol())
      Builder.addExternalSymbol(WDef.SymbolName);
    else
      Builder.addMetadata(MetaToken::createReadWire(WDef.WireNum, VMContext));
  }

  // Remove this instruction since they are fused to uc state.
  InstsToDel.push_back(&Inst);
}

//===----------------------------------------------------------------------===//

static inline bool top_sort_start(const HWAtom* LHS, const HWAtom* RHS) {
  if (LHS->getSlot() != RHS->getSlot())
    return LHS->getSlot() < RHS->getSlot();

  return LHS->getIdx() < RHS->getIdx();
}

static inline bool top_sort_finish(const HWAtom* LHS, const HWAtom* RHS) {
  if (LHS->getFinSlot() != RHS->getFinSlot())
    return LHS->getFinSlot() < RHS->getFinSlot();

  return LHS->getIdx() < RHS->getIdx();
}

MachineBasicBlock *FSMState::emitSchedule(BitLevelInfo &BLI) {
  const TargetInstrInfo *TII = TM.getInstrInfo();
  unsigned CurSlot = startSlot;
  VFuncInfo *VFI = MBB->getParent()->getInfo<VFuncInfo>();

  std::sort(Atoms.begin(), Atoms.end(), top_sort_start);

  // Build bundle from schedule units.
  {
    MicroStateBuilder BTB(*this, MBB->getBasicBlock()->getContext(), TM, BLI);

    for (iterator I = begin(), E = end(); I != E; ++I) {
      HWAtom *A = *I;

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
          assert(BTB.emitQueueEmpty() && "Unexpect atom before PHI.");
          // TODO: move this to MicroStateBuilder.
          MBB->remove(Inst);
          MBB->insert(BTB.getInsertPos(), Inst);
          continue;
        case TargetOpcode::COPY:
          // TODO: move this to MicroStateBuilder.
          MBB->remove(Inst);
          BTB.defereAtom(A);
          continue;
        }

        BTB.emitAtom(A);
      }
    }
    // Build last state.
    assert(!BTB.emitQueueEmpty() && "Expect atoms for last state!");
    BTB.advanceToSlot(CurSlot, CurSlot + 1, true);
  }

  DEBUG(
  dbgs() << "After schedule emitted:\n";
  for (MachineBasicBlock::iterator I = MBB->begin(), E = MBB->end();
      I != E; ++I) {
    MachineInstr *Instr = I;
    switch (Instr->getOpcode()) {
    default:
      Instr->dump();
      continue;
    case VTM::Control:
      dbgs() << "Control ";
      break;
    case VTM::Datapath:
      dbgs() << "Datapath ";
      break;
    case VTM::Terminator:
      dbgs() << "Terminator ";
      break;
    }

    ucState State = *Instr;
    dbgs() << State.getSlot() << '\n';
    
    for (ucState::iterator UOI = State.begin(), UOE = State.end();
         UOI != UOE; ++UOI) {
      ucOp Op = *UOI;
      Op.dump();
    }
  }
  );

  return MBB;
}

//===----------------------------------------------------------------------===//

void HWAtom::dump() const {
  print(dbgs());
  dbgs() << '\n';
}

void HWMemDep::print(raw_ostream &OS) const {

}

void HWCtrlDep::print(raw_ostream &OS) const {
}

void HWValDep::print(raw_ostream &OS) const {
}

unsigned llvm::HWAtom::getOpcode() const {
  if (MachineInstr *I =getFirstInstr())
    return I->getOpcode();

  return VTM::INSTRUCTION_LIST_END;
}

void HWAtom::scheduledTo(unsigned slot) {
  assert(slot && "Can not schedule to slot 0!");
  SchedSlot = slot;
}

void HWAtom::dropAllReferences() {
  for (dep_iterator I = dep_begin(), E = dep_end(); I != E; ++I)
    I->removeFromList(this);
}

void HWAtom::replaceAllUseBy(HWAtom *A) {
  while (!use_empty()) {
    HWAtom *U = use_back();

    U->setDep(U->getDepIt(this), A);
  }
}

VFUs::FUTypes HWAtom::getFUType() const {
  if (MachineInstr *Instr = getFirstInstr())
    return VTFInfo(*Instr).getFUType();

  return VFUs::Trivial;
}

void HWAtom::print(raw_ostream &OS) const {
  OS << "[" << getIdx() << "] ";

  for (const_instr_iterator I = instr_begin(), E = instr_end(); I != E; ++I) {
    MachineInstr *Instr = *I;

    VTFInfo VTID = Instr->getDesc();
    OS << Instr->getDesc().getName() << '\t';
    OS << " Res: " << VTID.getFUType();
    DEBUG(OS << '\n' << *Instr);

    if (getFUId().isBinded())
      OS << "\nFUId:" << getFUNum();

    OS << '\n';
  }
  
  OS << "\nAt slot: " << getSlot();
}

void FSMState::print(raw_ostream &OS) const {
}

void FSMState::dump() const {
  print(dbgs());
}

#include "llvm/Support/CommandLine.h"

static cl::opt<bool>
NoFDLS("disable-fdls",
       cl::desc("vbe - Do not preform force-directed list schedule"),
       cl::Hidden, cl::init(false));


static cl::opt<bool>
NoFDMS("disable-fdms",
       cl::desc("vbe - Do not preform force-directed modulo schedule"),
       cl::Hidden, cl::init(false));

void FSMState::schedule() {
  std::sort(Atoms.begin(), Atoms.end(), top_sort_start);

  // Create the FDInfo.
  //ModuloScheduleInfo MSInfo(HI, &getAnalysis<LoopInfo>(), State);
  ForceDirectedSchedulingBase *Scheduler = 0;
  if (NoFDLS)
    Scheduler = new ForceDirectedScheduler(this);
  else
    Scheduler = new ForceDirectedListScheduler(this);

  //if (MSInfo.isModuloSchedulable()) {
  //  if (NoFDMS) {
  //    delete Scheduler;
  //    Scheduler = new IteractiveModuloScheduling(HI, State);
  //  }
  //  
  //  scheduleCyclicCodeRegion(MSInfo.computeMII());
  //} else
  scheduleLinear(Scheduler);

  // Do not forget to schedule the delay atom;
  for (FSMState::iterator I = begin(), E = end(); I != E; ++I) {
    HWAtom *A = *I;
    assert(A->isScheduled() && "Schedule incomplete!");
  }
}


void FSMState::scheduleLinear(ForceDirectedSchedulingBase *Scheduler) {
  while (!Scheduler->scheduleState())
    Scheduler->lengthenCriticalPath();

  // Set the Initial Interval to the total slot, so we can generate the correct
  // control logic for loop if MS is disable.
  if (haveSelfLoop()) {
    setNoOverlapII();
    DEBUG(dbgs() << "Latency: " << getTotalSlot() << '\n');
  }
  DEBUG(Scheduler->dumpTimeFrame());
}

void FSMState::scheduleLoop(ForceDirectedSchedulingBase *Scheduler, 
                                        unsigned II) {
  dbgs() << "MII: " << II << "...";
  // Ensure us can schedule the critical path.
  while (!Scheduler->scheduleCriticalPath(true))
    Scheduler->lengthenCriticalPath();

  Scheduler->setMII(II);
  while (!Scheduler->scheduleCriticalPath(true))
    Scheduler->increaseMII();

  // The point of current solution.
  typedef std::pair<unsigned, unsigned> SolutionPoint;
  SolutionPoint CurPoint
    = std::make_pair(Scheduler->getMII(), Scheduler->getCriticalPathLength());
  SmallVector<SolutionPoint, 3> NextPoints;

  double lastReq = 1e9;

  while (!Scheduler->scheduleState()) {
    double CurReq = Scheduler->getExtraResReq();
    if (lastReq > CurReq) {
      CurPoint = std::make_pair(Scheduler->getMII(),
        Scheduler->getCriticalPathLength());
      lastReq = CurReq;
      NextPoints.clear();
    }

    if (NextPoints.empty()) {
      NextPoints.push_back(std::make_pair(CurPoint.first + 1, CurPoint.second  + 1));
      if (Scheduler->getCriticalPathLength() >= Scheduler->getMII())
        NextPoints.push_back(std::make_pair(CurPoint.first + 1, CurPoint.second));
      NextPoints.push_back(std::make_pair(CurPoint.first, CurPoint.second  + 1));
      // Add both by default.
      CurPoint = std::make_pair(CurPoint.first + 1, CurPoint.second  + 1);
    }

    Scheduler->setMII(NextPoints.back().first);
    Scheduler->setCriticalPathLength(NextPoints.back().second);
    NextPoints.pop_back();
  }
  DEBUG(dbgs() << "SchedII: " << Scheduler->getMII()
    << " Latency: " << getTotalSlot() << '\n');

  //Scheduler->getCriticalPathLength() < Scheduler->getMII())
  setII(Scheduler->getMII());
}

void FSMState::viewGraph() {
  ViewGraph(this, this->getMachineBasicBlock()->getName());
}
