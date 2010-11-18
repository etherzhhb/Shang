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
#include "llvm/Support/Debug.h"
#include "llvm/Analysis/LoopInfo.h"

using namespace llvm;

namespace {
// Helper class to build Micro state.
struct MicroStateBuilder {
  MicroStateBuilder(const MicroStateBuilder&);     // DO NOT IMPLEMENT
  void operator=(const MicroStateBuilder&); // DO NOT IMPLEMENT

  unsigned WireNum;
  unsigned OpId;
  FSMState &State;
  LLVMContext &VMContext;
  const TargetMachine &Target;
  const TargetInstrInfo &TII;
  MachineRegisterInfo &MRI;
  VFuncInfo &VFI;
  BitLevelInfo &BLI;
  
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

  MicroStateBuilder(FSMState &S, LLVMContext& Context, const TargetMachine &TM,
                    BitLevelInfo &BitInfo)
  : WireNum(S.getMachineBasicBlock()->getNumber() << 24),
  OpId(S.getMachineBasicBlock()->getNumber() << 24),
  State(S), VMContext(Context), Target(TM), TII(*TM.getInstrInfo()),
  MRI(S.getMachineBasicBlock()->getParent()->getRegInfo()),
  VFI(*S.getMachineBasicBlock()->getParent()->getInfo<VFuncInfo>()),
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

  MachineInstr *buildMicroState(unsigned Slot, MachineBasicBlock::iterator InsertPos,
                                SmallVectorImpl<HWAtom *> &Insts);
};
}

//===----------------------------------------------------------------------===//

MachineInstr*
MicroStateBuilder::buildMicroState(unsigned Slot,
                                   MachineBasicBlock::iterator InsertPos,
                                   SmallVectorImpl<HWAtom *> &Atoms) {
  MachineInstrBuilder Builder = BuildMI(*State.getMachineBasicBlock(),
                                        InsertPos, DebugLoc(), 
                                        TII.get(VTM::VOpBundle));
  Builder.addImm(Slot);

  for (SmallVectorImpl<HWAtom*>::iterator I = Atoms.begin(),
       E = Atoms.end(); I !=E; ++I) {
    HWAtom *A = *I;
    MachineInstr *Inst = A->getInst();

    assert(Inst && "Inst can not be null!");
    VTIDReader VTID(Inst);

    // Add the opcode metadata and the function unit id.
    Builder.addMetadata(MetaToken::createInstr(++OpId, Inst, A->getFUNum(),
                                               VMContext));
    typedef SmallVector<MachineOperand*, 8> OperandVector;
    OperandVector Ops(Inst->getNumOperands());
    
    // Remove all operand of Instr.
    while (Inst->getNumOperands() != 0) {
      unsigned i = Inst->getNumOperands() - 1;
      MachineOperand *MO = &Inst->getOperand(i);
      Inst->RemoveOperand(i);
      Ops[i] = MO;
    }

    unsigned EmitSlot = A->getSlot(),
             WriteSlot = A->getFinSlot();
    unsigned ReadSlot = EmitSlot;
    
    bool isReasAtEmit = VTID.isReadAtEmit();

    // We can not write the value to a register at the same moment we emit it.
    // Unless we read at emit.
    // FIXME: Introduce "Write at emit."
    if (WriteSlot == EmitSlot && !isReasAtEmit) ++WriteSlot;
    // Write to register operation need to wait one more slot if the result is
    // written at the moment (clock event) that the atom finish.
    if (VTID.isWriteUntilFinish()) ++WriteSlot;
    

    // We read the values after we emit it unless the value is read at emit.
    if (!isReasAtEmit) ++ReadSlot;
    
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
    Builder.addMetadata(MetaToken::createDefReg(++OpId, WD->WireNum, VMContext));
    // The def operands are written at the same time that the use operands
    // are read.
    MO->setIsEarlyClobber();
    Builder.addOperand(*MO);
  }

  // Delete the instructions.
  while (!Atoms.empty())
    InstsToDel.push_back(Atoms.pop_back_val()->getInst());

  return 0;
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
  MachineBasicBlock::iterator InsertPos = MBB->end();
  SmallVector<HWAtom*, 8> AtomsToEmit;
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

      if (A->getSlot() != CurSlot) {
        BTB.buildMicroState(CurSlot, InsertPos, AtomsToEmit);
        // Some states may not emit any atoms, but it may read the result from
        // previous atoms.
        // Note that AtomsToEmit is empty now, so we do not emitting any new
        // atoms.
        while (++CurSlot != A->getSlot())
          BTB.buildMicroState(CurSlot, InsertPos, AtomsToEmit);
      }
      
      if (MachineInstr *Inst = A->getInst()) {
        // Ignore some instructions.
        switch (Inst->getOpcode()) {
        case TargetOpcode::PHI:
          assert(AtomsToEmit.empty() && "Unexpect atom before PHI.");
          MBB->remove(Inst);
          MBB->insert(InsertPos, Inst);
          continue;
        }

        AtomsToEmit.push_back(A);
      }
    }
    // Build last state.
    assert(!AtomsToEmit.empty() && "Expect atoms for last state!");
    BTB.buildMicroState(CurSlot, InsertPos, AtomsToEmit);
  }

  // DEBUG(
  dbgs() << "After schedule emitted:\n";
  for (MachineBasicBlock::iterator I = MBB->begin(), E = MBB->end();
      I != E; ++I) {
    ucState State(*I);

    dbgs() << "Bundle " << State.getSlot() << '\n';
    
    for (ucState::iterator UOI = State.begin(), UOE = State.end();
         UOI != UOE; ++UOI) {
      ucOp Op = *UOI;
      Op.dump();
    }
  }
  // );

  return MBB;
}

//===----------------------------------------------------------------------===//

HWAtom::~HWAtom() {}

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

HWAtom::HWAtom(MachineInstr *MI, unsigned short latency, unsigned short Idx,
               unsigned fuid)
  : Latency(latency), SchedSlot(0), InstIdx(Idx), FUNum(fuid), Instr(MI) {}

unsigned llvm::HWAtom::getOpcode() const
{
  if (MachineInstr *I =getInstr())
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
  if (MachineInstr *Instr = getInst())
    return VTIDReader(Instr).getFUType();

  return VFUs::Trivial;
}

void HWAtom::print(raw_ostream &OS) const {
  MachineInstr *Instr = getInst();
  VTIDReader VTID(Instr);

  OS << "[" << getIdx() << "] ";

  if (Instr) {
    OS << Instr->getDesc().getName() << '\t';
    OS << " Res: " << VTID.getFUType();
    DEBUG(OS << '\n' << *Instr);
  } else
    OS << "null";

  if (Instr && getFUId().isBinded()) OS << "\nFUId:" << getFUNum();
  
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

void FSMState::scheduleState() {
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
  scheduleACyclicCodeRegion(Scheduler);

  // Do not forget to schedule the delay atom;
  for (FSMState::iterator I = begin(), E = end(); I != E; ++I) {
    HWAtom *A = *I;
    assert(A->isScheduled() && "Schedule incomplete!");
  }
}


void FSMState::scheduleACyclicCodeRegion(ForceDirectedSchedulingBase *Scheduler) {
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

void FSMState::scheduleCyclicCodeRegion(ForceDirectedSchedulingBase *Scheduler, 
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
