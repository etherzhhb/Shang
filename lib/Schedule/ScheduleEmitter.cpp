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

template<typename T>
struct ValDefBase {
  unsigned RegNum;
  unsigned FinishSlot;
  T *Next;

  ValDefBase(unsigned RegNum, unsigned FinishSlot)
    : RegNum(RegNum), FinishSlot(FinishSlot), Next(0) {}

  // Return the last value of the define chain.
  inline T *getLatestValue() {
    T *Def = static_cast<T*>(this);
    while (Def->Next)
      Def = Def->Next;

    return Def;
  }

  // Insert V at the end of the link list.
  void push_back(T *V) {
    assert(Next == 0 && "Not at the back of the list!");
    Next = V;
  }

  // Insert V after the current node, and return the newly inserted node.
  T *insertAfter(T *V) {
    V->Next = Next;
    Next = V;
    return V;
  }
};

struct SimpleValDef : public ValDefBase<SimpleValDef> {
  SimpleValDef(unsigned RegNum, unsigned FinishSlot)
    : ValDefBase<SimpleValDef>(RegNum, FinishSlot) {}
};

// Helper class to build bundles.
struct BundleBuilder {
  BundleBuilder(const BundleBuilder&);     // DO NOT IMPLEMENT
  void operator=(const BundleBuilder&); // DO NOT IMPLEMENT

  VSchedGraph &State;
  MachineBasicBlock &MBB;
  const unsigned ScheduleStartSlot, ScheduleLoopOpSlot, ScheduleEndSlot, II,
                 StartSlot;
  const bool isMBBPipelined;
  typedef MachineInstr *InsertPosTy;
  InsertPosTy InsertPos, EndOfBB;

  MachineRegisterInfo &MRI;
  VFInfo &VFI;

  SmallVector<VSUnit*, 8> SUnitsToEmit;
  SpecificBumpPtrAllocator<SimpleValDef> Allocator;
  inline SimpleValDef *createValDef(unsigned RegNum, unsigned SchedSlot){
    // Compute the loop boundary, the last slot before starting a new loop,
    // which is at the same time with the first slot of next iteration.
    unsigned LoopBoundarySlot = 0;
    assert(isMBBPipelined && "Only create value define for pipelined BB!");

    // We need to insert a PHI node to preserve SSA form, by
    // avoiding use before define, which occur if the read is wrap around.
    LoopBoundarySlot = std::max(SchedSlot - ScheduleStartSlot, 1u);
    LoopBoundarySlot = RoundUpToAlignment(LoopBoundarySlot, II);
    LoopBoundarySlot += ScheduleStartSlot;
    
    assert(LoopBoundarySlot > ScheduleStartSlot
            && "LoopBoundary should bigger than start slot!");

    return new (Allocator.Allocate()) SimpleValDef(RegNum, LoopBoundarySlot);
  }
    
  typedef SmallVector<InsertPosTy, 32> IPVector;
  IPVector CtrlIPs, DataPathIPs;

  // register number -> wire define.
  typedef std::map<unsigned, std::pair<SimpleValDef*, unsigned> > PHIDefMapType;
  PHIDefMapType PHIDefs;

  BundleBuilder(VSchedGraph &S, MachineBasicBlock *MBB, unsigned StartSlot)
  : State(S), MBB(*MBB), ScheduleStartSlot(S.getStartSlot(MBB)),
    ScheduleLoopOpSlot(S.getLoopOpSlot(MBB)), ScheduleEndSlot(S.getEndSlot(MBB)),
    II(S.getII(MBB)), StartSlot(StartSlot), isMBBPipelined(S.isPipelined(MBB)),
    InsertPos(MBB->end()), MRI(MBB->getParent()->getRegInfo()),
    VFI(*MBB->getParent()->getInfo<VFInfo>())
  {
    // Build the instructions for mirco-states.
    unsigned EndSlot = StartSlot + II;
    MachineInstr *Start =
      BuildMI(*MBB, InsertPos, DebugLoc(), VInstrInfo::getDesc(VTM::CtrlStart))
        .addImm(StartSlot).addImm(0).addImm(0);
    MachineInstr *End =
      BuildMI(*MBB, InsertPos, DebugLoc(), VInstrInfo::getDesc(VTM::CtrlEnd))
       .addImm(StartSlot).addImm(0).addImm(0);
    // Control Ops are inserted between Ctrl-Starts and Ctrl-Ends
    CtrlIPs.push_back(End);

    for (unsigned i = StartSlot + 1, e = EndSlot; i <= e; ++i) {
      // Build the header for datapath from in slot.
      BuildMI(*MBB, InsertPos, DebugLoc(), VInstrInfo::getDesc(VTM::Datapath))
        .addImm(i - 1).addImm(0).addImm(0);
      Start = BuildMI(*MBB, InsertPos, DebugLoc(),
                      VInstrInfo::getDesc(VTM::CtrlStart))
                .addImm(i).addImm(0).addImm(0);
      End = BuildMI(*MBB, InsertPos, DebugLoc(),
                    VInstrInfo::getDesc(VTM::CtrlEnd))
              .addImm(i).addImm(0).addImm(0);
      // Datapath Ops are inserted between Ctrl-Ends and Ctrl-Starts
      DataPathIPs.push_back(Start);
      CtrlIPs.push_back(End);
    }

    // Build Datapath bundle for dangling data-paths.
    BuildMI(*MBB, InsertPos, DebugLoc(), VInstrInfo::getDesc(VTM::Datapath))
      .addImm(EndSlot).addImm(0).addImm(0);

    // Build the dummy terminator.
    EndOfBB = BuildMI(MBB, DebugLoc(), VInstrInfo::getDesc(VTM::EndState))
                .addImm(0).addImm(0);
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

  bool isReadWrapAround(OpSlot ReadSlot, SimpleValDef *VD) const {
    return OpSlot(VD->FinishSlot, true) < ReadSlot;
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
  void buildBundle(unsigned Slot);

  // Fuse instructions in a bundle.
  void moveInstr(MachineInstr &Inst, OpSlot SchedSlot);

  void updateOperand(MachineInstr &Inst, OpSlot SchedSlot);

  // Build the machine operand that use at a specified slot.
  unsigned getRegAtSlot(unsigned Reg, OpSlot ReadSlot) {
    PHIDefMapType::iterator at = PHIDefs.find(Reg);

    if (at == PHIDefs.end()) return Reg;

    return getRegAtSlot(at->second.first, ReadSlot, at->second.second);;
  }

  // Build the machine operand that read the wire definition at a specified slot.
  unsigned getRegAtSlot(SimpleValDef *VD, OpSlot ReadSlot, unsigned SizeInBits);

  void emitPHIDef(MachineInstr *PN) {
    // FIXME: Place the PHI define at the right slot to avoid the live interval
    // of registers in the PHI overlap. 
    unsigned InsertSlot = /*Slot ? Slot :*/ ScheduleStartSlot;
    MachineOperand &MO = PN->getOperand(0);
    unsigned PHIReg = MO.getReg();
    const TargetRegisterClass *RC = MRI.getRegClass(PHIReg);
    unsigned NewReg = MRI.createVirtualRegister(RC);

    DebugLoc dl;
    InsertPosTy IP = getStateCtrlAt(OpSlot(InsertSlot, true));
    unsigned BitWidth = VInstrInfo::getBitWidth(MO);
    BuildMI(MBB, IP, dl, VInstrInfo::getDesc(VTM::VOpDefPhi))
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
  unsigned createPHI(unsigned RegNo, unsigned SizeInBits) {
    SmallVector<MachineInstr*, 4> InsertedPHIs;
    const TargetRegisterClass *RC = MRI.getRegClass(RegNo);

    unsigned LiveOutReg = MRI.createVirtualRegister(RC);
    // Put the copy at then end of the block.
    BuildMI(&MBB, DebugLoc(), VInstrInfo::getDesc(VTM::COPY), LiveOutReg)
      .addReg(RegNo, RegState::Kill);
    RegNo = LiveOutReg;

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
      unsigned InitReg = MRI.createVirtualRegister(RC);
      MachineOperand InitOp = MachineOperand::CreateReg(InitReg, true);
      VInstrInfo::setBitWidth(InitOp, SizeInBits);

      MachineBasicBlock::iterator IP = PredBB->getFirstTerminator();
      // Insert the imp_def before the PHI incoming copies.
      while (llvm::prior(IP)->getOpcode() == VTM::VOpMvPhi)
        --IP;

      BuildMI(*PredBB, IP, DebugLoc(), VInstrInfo::getDesc(VTM::IMPLICIT_DEF))
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

      // Insert the PHIDef, so we can avoid the PHIElimination inserting
      // strange COPY instruction. Note that we may defining "wire" in a control
      // slot, however it is ok since the PHIDef will be eliminated.
      emitPHIDef(PN);
    }

    return NewReg;
  }

  // Increase the slot counter and emit all pending schedule units.
  unsigned advanceToSlot(unsigned CurSlot, unsigned TargetSlot) {
    assert(TargetSlot > CurSlot && "Bad target slot!");
    buildBundle(CurSlot);
    SUnitsToEmit.clear();
    // Advance current slot.
    ++CurSlot;

    // Some states may not emit any atoms, but it may read the result from
    // previous atoms.
    // Note that SUnitsToEmit is empty now, so we do not emitting any new
    // atoms.
    while (CurSlot < TargetSlot && CurSlot < ScheduleEndSlot)
      buildBundle(CurSlot++);

    return CurSlot;
  }

  void updateLiveOuts();

  void finalizeMBB() {
    updateLiveOuts();

    // Place the end of MBB at the end.
    EndOfBB->removeFromParent();
    MBB.push_back(EndOfBB);
  }
};
}

//===----------------------------------------------------------------------===//
typedef std::pair<MachineInstr*, OpSlot> InSUInstInfo;
template<typename T>
static inline bool sort_intra_latency(const T &LHS, const T &RHS) {
  return LHS.second < RHS.second;
}

void BundleBuilder::buildBundle(unsigned Slot) {
  typedef SmallVectorImpl<VSUnit*>::iterator iterator;
  SmallVector<InSUInstInfo, 8> Insts;

  for (iterator I = SUnitsToEmit.begin(), E = SUnitsToEmit.end(); I !=E; ++I) {
    VSUnit *A = *I;
    Insts.clear();

    for (unsigned i = 0, e = A->num_instrs(); i < e; ++i) {
      MachineInstr *Inst = A->getPtrAt(i);
      // Ignore the entry node marker (null) and implicit define.
      if (Inst && !Inst->isImplicitDef()) {
        unsigned S = A->getSlot() +  A->getLatencyAt(i);
        bool IsCtrl = VInstrInfo::isControl(Inst->getOpcode());
        Insts.push_back(std::make_pair(Inst, OpSlot(S, IsCtrl)));
      }
    }

    // Sort the instructions, so we can emit them in order.
    std::sort(Insts.begin(), Insts.end(), sort_intra_latency<InSUInstInfo>);

    bool IsDangling = A->isDangling();

    typedef SmallVector<InSUInstInfo, 8>::iterator it;
    for (it I = Insts.begin(), E = Insts.end(); I != E; ++I) {
      MachineInstr *MI = I->first;
      OpSlot S = I->second;

      updateOperand(*MI, S);

      if (MI->isPHI()) {
        emitPHIDef(MI);
        continue;
      }

      // Simply place the dangling node at the end.
      if (IsDangling){
        assert(VInstrInfo::isDatapath(MI->getOpcode())
               && "Unexpected dangling control-path operation!");
        ++DanglingDatapath;
        VInstrInfo::setInstrSlotNum(MI, 0);
        MI->removeFromParent();
        MBB.insert(MBB.getFirstInstrTerminator(), MI);
        continue;
      }

      // Move the instructions to the right place according to its schedule.
      moveInstr(*MI, S);
    }
  }
}

void BundleBuilder::updateOperand(MachineInstr &Inst, OpSlot SchedSlot) {
  // Adjust the operand by the timing.
  for (unsigned i = 0 , e = Inst.isPHI() ? 1 : Inst.getNumOperands();
       i != e; ++i) {
    MachineOperand &MO = Inst.getOperand(i);

    // Ignore the non-register operand (also ignore reg0 which means nothing).
    if (!MO.isReg() || !MO.getReg())
      continue;

    const unsigned Reg = MO.getReg();

    if (MO.isUse()) {
      // Update the operand.
      if (isMBBPipelined)
        MO.ChangeToRegister(getRegAtSlot(MO.getReg(), SchedSlot), false);
      
      MO.setIsKill(false);
      continue;
    }

    if (MRI.use_empty(Reg)) {
      MO.ChangeToRegister(0, true);
      continue;
    }

    // We only create the value define if the BB is pipelined, in that case, we
    // may need to insert PHI nodes to preserve SSA form.
    if (!isMBBPipelined) continue;

    unsigned BitWidth = VInstrInfo::getBitWidth(MO);
    SimpleValDef *VD = createValDef(Reg, SchedSlot.getSlot());

    bool inserted
      = PHIDefs.insert(std::make_pair(Reg, std::make_pair(VD, BitWidth))).second;
    assert(inserted && "Instructions not in SSA form!");  
  }
}

void BundleBuilder::moveInstr(MachineInstr &Inst, OpSlot SchedSlot) {
  bool IsCtrl = VInstrInfo::isControl(Inst.getOpcode());

  unsigned Opcode = Inst.getOpcode();
  // FIX the opcode of terminators.
  if (Inst.isTerminator()) {
    if (VInstrInfo::isBrCndLike(Opcode))
      Inst.setDesc(VInstrInfo::getDesc(VTM::VOpToState_nt));
    else
      Inst.setDesc(VInstrInfo::getDesc(VTM::VOpRet_nt));
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
  MBB.insert(MachineBasicBlock::instr_iterator(IP), &Inst);
}

unsigned BundleBuilder::getRegAtSlot(SimpleValDef *VD, OpSlot ReadSlot,
                                         unsigned SizeInBits) {
  // If the use is appear before the definition of the register, we need to
  // insert PHI nodes to preserve SSA-form.
  while (isReadWrapAround(ReadSlot, VD)) {
    if (VD->Next == 0) {
      unsigned RegNo = createPHI(VD->RegNum, SizeInBits);

      VD->push_back(createValDef(RegNo, VD->FinishSlot + II));
    }

    VD = VD->Next;
  }

  return VD->RegNum;
}

void BundleBuilder::updateLiveOuts() {
  typedef MachineRegisterInfo::use_iterator use_it;
  typedef PHIDefMapType::iterator iterator;

  for (iterator I = PHIDefs.begin(), E = PHIDefs.end(); I != E; ++I) {
    unsigned Reg = I->first;
    // Get the live-out value.
    SimpleValDef *Def = I->second.first->getLatestValue();

    // No need to update if the value is not wrapped.
    if (Reg == Def->RegNum) continue;

    for (use_it UI = MRI.use_begin(Reg); UI != MachineRegisterInfo::use_end();){
      MachineInstr &MI = *UI;
      MachineOperand &MO = (UI++).getOperand();

      if (MI.getParent() == &MBB) continue;

      // Use the live-out value in other MBBs.
      MO.ChangeToRegister(Def->RegNum, false);
    }
  }
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

void VSchedGraph::insertDelayBlock(const BBInfo &Info) {
  VSUnit *BBEntry = Info.Entry;
  MachineBasicBlock *MBB = BBEntry->getParentBB();

  typedef VSUnit::dep_iterator dep_it;
  for (dep_it I = cp_begin(Info.Entry), E = cp_end(Info.Entry); I != E; ++I) {
    VSUnit *PredTerminator = *I;

    int ExtraLatency = int(BBEntry->getSlot()) - int(PredTerminator->getSlot());
    if (ExtraLatency > 0)
      insertDelayBlock(PredTerminator->getParentBB(), MBB, ExtraLatency);
  }
}

void VSchedGraph::insertDelayBlocks() {
  // Because we will push new BBInfo to BBInfoMap during inserting delay blocks,
  // we should use the index to iterate over the exiting BBInfos, and the newly
  // pushed BBInfos will not be visited.
  for (unsigned i = 0, e = BBInfoMap.size(); i != e; ++i)
    insertDelayBlock(BBInfoMap[i]);
}

namespace {
struct ChainValDef : public ValDefBase<ChainValDef> {
  unsigned ChainStart;
  MachineInstr &MI;
  bool IsChainedWithFU;

  explicit ChainValDef(MachineInstr &MI, unsigned SchedSlot, unsigned FinishSlot)
    : ValDefBase<ChainValDef>(0, FinishSlot), ChainStart(SchedSlot), MI(MI),
      IsChainedWithFU(!VInstrInfo::getPreboundFUId(&MI).isTrivial()
                       // Dirty Hack: Ignore the copy like FU operations, i.e. ReadFU,
                       // and VOpDstMux.
                       && !VInstrInfo::isCopyLike(MI.getOpcode())) {}

  MachineBasicBlock *getParentBB() const { return MI.getParent(); }
  unsigned getParentBBNum() const { return getParentBB()->getNumber(); }

  bool isCopy() const { return FinishSlot == ChainStart; }
};

struct ChainBreaker {
  VSchedGraph &G;
  MachineRegisterInfo &MRI;

  SpecificBumpPtrAllocator<ChainValDef> Allocator;
  typedef std::map<unsigned, ChainValDef*> ValDefMapTy;
  ValDefMapTy ValDefs;

  void addValDef(ChainValDef *Def) {
    bool succ = ValDefs.insert(std::make_pair(Def->RegNum, Def)).second;
    assert(succ && "ValDef already existed?");
    (void) succ;
  }

  ChainValDef *lookupValDef(unsigned RegNum) {
    ValDefMapTy::iterator at = ValDefs.find(RegNum);
    assert(at != ValDefs.end() && "ValDef not found!");
    return at->second;
  }

  unsigned getFinishAtCurBB(const ChainValDef &Def) {
    return Def.getParentBB() == MBB ? Def.FinishSlot : StartSlot;
  }

  unsigned getChainStartAtCurBB(const ChainValDef &Def) {
    return Def.getParentBB() == MBB ? Def.ChainStart : StartSlot;
  }

  inline int getSlack(unsigned UseSlot, unsigned UseBBNum,
                      unsigned DefSlot, unsigned DefBBNum) const {
    int Slack = UseSlot - DefSlot;

    return Slack;
  }

  int getSlackFromChainStart(unsigned UseSlot, unsigned UseBBNum,
                             const ChainValDef &Def) const {
    return getSlack(UseSlot, UseBBNum, Def.ChainStart, Def.getParentBBNum());
  }

  int getSlackFromFinish(unsigned UseSlot, unsigned UseBBNum,
                         const ChainValDef &Def) const {
    return getSlack(UseSlot, UseBBNum, Def.FinishSlot, Def.getParentBBNum());
  }

  MachineBasicBlock *MBB;
  unsigned CurII, StartSlot;
  bool IsPipelined;

  explicit ChainBreaker(VSchedGraph *G)
    : G(*G), MRI(*G->DLInfo.MRI), MBB(0), CurII(0), StartSlot(0),
      IsPipelined(false) {}

  void updateMBB(MachineBasicBlock *NewMBB) {
    MBB = NewMBB;
    StartSlot = G.getStartSlot(MBB);
    CurII = G.getII(MBB);
    IsPipelined = G.isPipelined(MBB);
  }

  MachineInstr *buildReadFU(MachineInstr *MI, VSUnit *U, unsigned Offset,
                            FuncUnitId Id = FuncUnitId(), bool Dead = false);

  ChainValDef *getValAtSlot(ChainValDef *Def, unsigned Slot, bool CreateCopy);
  ChainValDef *insertCopyAtSlot(ChainValDef &SrcDef, unsigned Slot);
  ChainValDef *breakChainForAntiDep(ChainValDef *SrcDef, unsigned ChainEndSlot,
                                    unsigned ReadSlot, unsigned II);

  void buildFUCtrl(VSUnit *U);

  void visitUse(MachineInstr *MI, ChainValDef &Def, bool IsDangling,
                unsigned LatestChainEnd);

  void visitDef(MachineInstr *MI, ChainValDef &Def);

  void visit(VSUnit *U);
};
}

MachineInstr *ChainBreaker::buildReadFU(MachineInstr *MI, VSUnit *U,
                                        unsigned Offset, FuncUnitId Id,
                                        bool Dead) {
  unsigned RegWidth = VInstrInfo::getBitWidth(MI->getOperand(0));
  unsigned ResultWire = MI->getOperand(0).getReg();
  unsigned ResultReg = Dead ? 0 : MRI.createVirtualRegister(&VTM::DRRegClass);

  DebugLoc dl = MI->getDebugLoc();
  MachineBasicBlock::instr_iterator IP = MI;
  ++IP;

  assert((MI->isPseudo()
          || VInstrInfo::isAlwaysTruePred(*VInstrInfo::getPredOperand(MI)))
         && "Unexpected predicated MI!");

  MachineInstr *ReadFU
    = BuildMI(*MI->getParent(), IP, dl, VInstrInfo::getDesc(VTM::VOpReadFU))
      .addOperand(VInstrInfo::CreateReg(ResultReg, RegWidth, true))
      .addOperand(VInstrInfo::CreateReg(ResultWire, RegWidth, false))
      .addImm(Id.getData()).addOperand(VInstrInfo::CreatePredicate())
      .addOperand(VInstrInfo::CreateTrace());
  G.mapMI2SU(ReadFU, U, Offset, true);
  G.addDummyLatencyEntry(ReadFU);

  return ReadFU;
}

void ChainBreaker::buildFUCtrl(VSUnit *U) {
  MachineInstr *MI = U->getRepresentativePtr();
  // The machine instruction that define the output value.
  assert(MI && "Unexpected BBEntry SU!");
  const TargetRegisterClass *RC
    = VRegisterInfo::getRepRegisterClass(MI->getOpcode());

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
    unsigned BRAMOpResult = MO.getReg();
    unsigned BRAMPortReg = MRI.createVirtualRegister(RC);
    // Change to the newly allocated register, and kill the new register
    // with VOpPipelineStage.
    MO.ChangeToRegister(BRAMPortReg, true);
    unsigned ResultWidth = VInstrInfo::getBitWidth(MO);
    MachineInstr *PipeStage =
      BuildMI(*MBB, IP, dl, VInstrInfo::getDesc(VTM::VOpPipelineStage))
        .addOperand(VInstrInfo::CreateReg(BRAMOpResult, ResultWidth, true))
        .addOperand(VInstrInfo::CreateReg(BRAMPortReg, ResultWidth)).addImm(Id.getData())
        .addOperand(*VInstrInfo::getPredOperand(MI))
        .addOperand(*VInstrInfo::getTraceOperand(MI));
    assert(U->isControl() && "Only control operation write until finish!");
    G.mapMI2SU(PipeStage, U, U->getLatency() - 1, true);

    // Copy the result 1 cycle later after the value is finished, note that
    // the PipeStage is emit to a data path slot, to delay the VOpReadFU 1
    // cycle later, we need to set its delay to 2, other wise the copy will
    // emit right after the RepLI finish, instead of 1 cycle after the RepLI
    // finish. FIXME: Set the right latency.
    G.addDummyLatencyEntry(PipeStage, 2.0f);
    // The result is not provided by PipeStage.
    // Copy the result of the pipe stage to register if it has any user.
    if (!MRI.use_empty(BRAMOpResult))
      buildReadFU(PipeStage, U, G.getStepsToFinish(PipeStage), Id);
  } else if (!Id.isTrivial()) {
    // The result of InternalCall is hold by the ReadReturn operation, so we
    // do not need to create a new register to hold the result. Otherwise, a
    // new register is needed.
    bool IsWaitOnly = Id.getFUType() == VFUs::CalleeFN;
    buildReadFU(MI, U, G.getStepsToFinish(MI), Id, IsWaitOnly);
    MRI.setRegClass(MI->getOperand(0).getReg(), RC);
  }

  // We also need to disable the FU.
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
    G.mapMI2SU(DisableMI, U, 1);
    G.addDummyLatencyEntry(DisableMI);
  }
}

ChainValDef *ChainBreaker::getValAtSlot(ChainValDef *Def, unsigned Slot,
                                        bool CreateCopy) {
  assert((Slot >= Def->FinishSlot || !CreateCopy)
         && "Read before write detected!");
  // Only return the value at then end of the BB, do not perform cross BB copy.
  if (Def->getParentBB() != MBB) Slot = Def->FinishSlot;

  // Get the value just before ReadSlot.
  while (Def->Next && Def->Next->FinishSlot <= Slot)
    Def = Def->Next;

  if (Def->ChainStart != Slot && CreateCopy) {
    // Insert the copy operation.
    VSUnit *DefSU = G.lookupSUnit(&Def->MI);
    assert(!DefSU->isDangling() && "Cannot insert Copy for dangling SU!");

    unsigned CopyOffset = Slot - DefSU->getSlot();
    assert(Slot <= G.getEndSlot(MBB) && "Bad copy slot!");
    MachineInstr *ReadFU = buildReadFU(&Def->MI, DefSU, CopyOffset);

    // Remember the value definition for the copy.
    ChainValDef *NewVal = Allocator.Allocate();
    new (NewVal) ChainValDef(*ReadFU, Slot, Slot);
    NewVal->RegNum = ReadFU->getOperand(0).getReg();
    addValDef(NewVal);

    Def = Def->insertAfter(NewVal);
  }

  return Def;
}

ChainValDef *ChainBreaker::insertCopyAtSlot(ChainValDef &SrcDef, unsigned Slot){
  assert(SrcDef.ChainStart < Slot && SrcDef.Next == 0 && "Cannot insert copy!");

  // Create the copy.
  MachineInstr *MI = &SrcDef.MI;

  unsigned RegWidth = VInstrInfo::getBitWidth(MI->getOperand(0));
  unsigned ResultWire = MI->getOperand(0).getReg();
  unsigned ResultReg = MRI.createVirtualRegister(&VTM::DRRegClass);

  DebugLoc dl = MI->getDebugLoc();
  bool InSameBB = MI->getParent() == MBB;
  MachineBasicBlock::iterator IP = InSameBB ?
                                   llvm::next(MachineBasicBlock::iterator(MI)) :
                                   MBB->getFirstNonPHI();
  // FIXME: Predicate the copy with the condition of the user.
  MachineInstr *ReadFU
    = BuildMI(*MBB, IP, dl, VInstrInfo::getDesc(VTM::VOpReadFU))
    .addOperand(VInstrInfo::CreateReg(ResultReg, RegWidth, true))
    .addOperand(VInstrInfo::CreateReg(ResultWire, RegWidth, false))
    .addImm(FuncUnitId().getData()).addOperand(VInstrInfo::CreatePredicate())
    .addOperand(VInstrInfo::CreateTrace());

  VSUnit *U = InSameBB ? G.lookupSUnit(MI) : G.lookupSUnit(MBB);
  assert(Slot >= U->getSlot() && "Unexpected negative offset!");
  assert(Slot <= G.getEndSlot(MBB) && "Bad copy slot!");
  G.mapMI2SU(ReadFU, U, Slot - U->getSlot(), true);
  G.addDummyLatencyEntry(ReadFU);

  // Remember the value definition for the copy.
  ChainValDef *NewDef = Allocator.Allocate();
  new (NewDef) ChainValDef(*ReadFU, Slot, Slot);
  NewDef->RegNum = ResultReg;
  addValDef(NewDef);

  return NewDef;
}

ChainValDef *ChainBreaker::breakChainForAntiDep(ChainValDef *SrcDef,
                                                unsigned ChainEndSlot,
                                                unsigned ReadSlot, unsigned II){
  // Break the chain to preserve anti-dependencies.
  // FIXME: Calculate the first slot available for inserting the copy.
  unsigned ChainStartSlot = getChainStartAtCurBB(*SrcDef);
  int ChainLength =  ChainEndSlot - ChainStartSlot;

  while (ChainLength > int(II)) {
    assert(SrcDef->FinishSlot <= ReadSlot
           && "Cannot break the chain to preserve anti-dependency!");
    // Try to get the next value define, which hold the value of UseVal in
    // another register in later slots.
    if (!SrcDef->Next || SrcDef->Next->FinishSlot > ReadSlot) {
      unsigned CopySlot = std::min(ChainStartSlot + II, ReadSlot);
      SrcDef->insertAfter(insertCopyAtSlot(*SrcDef, CopySlot));
    }

    SrcDef = SrcDef->Next;
    ChainStartSlot = getChainStartAtCurBB(*SrcDef);
    ChainLength =  ChainEndSlot - ChainStartSlot;
  }

  return SrcDef;
}

void ChainBreaker::visitUse(MachineInstr *MI, ChainValDef &Def, bool IsDangling,
                            unsigned LatestChainEnd) {
  if (MI->isPseudo()) return;

  unsigned CurBBNum = MBB->getNumber();
  bool IsDatapath = VInstrInfo::isDatapath(MI->getOpcode());
  bool IsPipeStage = MI->getOpcode() == VTM::VOpPipelineStage;
  // The control-path operation can read the old value just before the new
  // value come out, calculate the latest slot at which the value can be read
  // by current operation.
  const unsigned ReadSlot = IsDatapath ? Def.ChainStart : Def.ChainStart - 1;
  for (unsigned i = 0, e = MI->getNumOperands(); i != e; ++i) {
    MachineOperand &MO = MI->getOperand(i);

    if (!MO.isReg() || !MO.isUse() || !MO.getReg()) continue;

    ChainValDef *SrcVal = lookupValDef(MO.getReg());
    bool InSameBB = SrcVal->getParentBB() == MBB;

    // Try to break the chain to shorten the live-interval of the FU.
    // FIXME: Calculate the slot the break the chain, and break the chain lazily.
    int ChainBreakingSlack = getSlackFromFinish(ReadSlot, CurBBNum, *SrcVal);
    if (ChainBreakingSlack >= 0) {
      if (SrcVal->IsChainedWithFU)
        SrcVal = getValAtSlot(SrcVal, SrcVal->FinishSlot, true);

      if (IsPipelined && InSameBB && !IsDangling
          && LatestChainEnd - SrcVal->ChainStart > CurII
          && LatestChainEnd - ReadSlot < CurII)
        // Insert the copy to break the chain.
        SrcVal = getValAtSlot(SrcVal, ReadSlot, true);
    }

    // If the current MI is dangling, read the latest value.
    SrcVal = getValAtSlot(SrcVal, ReadSlot, false);

    // Try to break the chain to preserve the anti-dependencies in pipelined
    // block, but ignore the value from other BB, which are invariants.
    if (InSameBB) {
      // The dangling MI is not read by the MI in the same BB, hence there is
      // no anti-dependencies to preserve.
      if (IsPipelined && !IsDangling) {
        unsigned ChainEndSlot = IsDatapath ? Def.FinishSlot : Def.ChainStart;
        // DIRTY HACK: The PipeStage is actually a copy operation, and copy the
        // value 1 slot after its schedule slot.
        if (IsPipeStage) ChainEndSlot = ReadSlot + 1;

        // Break the chain to preserve anti-dependencies.
        SrcVal = breakChainForAntiDep(SrcVal, ChainEndSlot, ReadSlot, CurII);
      }

      // Find the longest chain so that we can break the chain to preserve the
      // anti-dependencies.
      if (IsDatapath && !IsPipeStage)
        Def.ChainStart = std::min(Def.ChainStart, SrcVal->ChainStart);

      Def.IsChainedWithFU |= SrcVal->IsChainedWithFU && IsDatapath;
    }

    // FIXME: The finish slot calculation should be improve:
    // unsigned ExtraLatencyFromSrc = std::max(Src.Latency - Def.Latnecy, 0);
    // Def.FinishSlot = std::max(Def.FinishSlot,
    //                           SrcVal->FinishSlot + ExtraLatencyFromSrc);
    Def.FinishSlot = std::max(Def.FinishSlot, SrcVal->FinishSlot);

    assert((SrcVal->getParentBB() == MBB || !SrcVal->IsChainedWithFU)
           && "Unexpected cross BB chain.");

    // Update the used register.
    if (MO.getReg() != SrcVal->RegNum)
      MO.ChangeToRegister(SrcVal->RegNum, false);
  }
}

void ChainBreaker::visitDef(MachineInstr *MI, ChainValDef &Def) {
  for (unsigned i = 0, e = MI->getNumOperands(); i != e; ++i) {
    MachineOperand &MO = MI->getOperand(i);

    if (!MO.isReg() || !MO.isDef() || !MO.getReg()) continue;

    assert(Def.RegNum == 0 && "Unexpected multiple defines!");
    Def.RegNum = MO.getReg();
  }
}

static inline bool canForwardSrcVal(unsigned Opcode) {
  // Try to forward the copied value. However, do not forward the VOpDstMux,
  // otherwise we may extend the live-interval of the VOpDstMux and prevent
  // it from binding to the specific Mux.
  return VInstrInfo::isCopyLike(Opcode) && Opcode != VTM::VOpPipelineStage &&
         Opcode != VTM::PHI && Opcode != VTM::VOpDstMux;
}

void ChainBreaker::visit(VSUnit *U) {
  if (U->getFUId().isBound()) {
    assert(U->isControl() && "Unexpected data-path operation!");
    buildFUCtrl(U);
  }

  // Try to break the chain aggressively, to preserve the anti-dependencies in
  // pipelined block.
  unsigned LatestChainEnd = U->getFinSlot();
  if (IsPipelined && U->isDatapath() && !U->isDangling()) {
    typedef VSUnit::use_iterator use_it;
    for (use_it I = duse_begin(U), E = duse_end(U); I != E; ++I) {
      VSUnit *User = *I;
      // There is no anti-dependencies from these U to these user.
      if (User->getParentBB() != MBB || User->isDangling() || User->isDatapath())
        continue;

      unsigned ChainEnd = User->getSlot();
      LatestChainEnd = std::max(LatestChainEnd, ChainEnd);
    }
  }

  SmallVector<std::pair<MachineInstr*, int>, 8> Insts;

  for (unsigned i = 0, e = U->num_instrs(); i != e; ++i)
    if (MachineInstr *MI = U->getPtrAt(i)) {
      bool IsDatapath = VInstrInfo::isDatapath(MI->getOpcode());
      int Offset = U->getLatencyAt(i) * 2 + (IsDatapath ? 1 : 0);
      Insts.push_back(std::make_pair(MI, Offset));
    }

  std::sort(Insts.begin(), Insts.end(),
            sort_intra_latency<std::pair<MachineInstr*, int> >);

  typedef SmallVectorImpl<std::pair<MachineInstr*, int> >::iterator iterator;
  for (iterator I = Insts.begin(), E = Insts.end(); I != E; ++I) {
    MachineInstr *MI = I->first;
    unsigned Opcode = MI->getOpcode();
    unsigned SchedSlot = U->getSlot() + I->second / 2;
    unsigned Latency = G.getStepsToFinish(MI);
    bool IsDangling = U->isDangling();
    ChainValDef Def(*MI, SchedSlot, SchedSlot + Latency);

    visitUse(MI, Def, IsDangling, LatestChainEnd);
    // Do not export the define of VOpMvPhi, which is used by the PHI only.
    if (Opcode == VTM::VOpMvPhi) continue;

    visitDef(MI, Def);

    // This MI define nothing, do not remember the value define.
    if (Def.RegNum == 0) continue;

    // Fix the register class for the result.
    MRI.setRegClass(Def.RegNum, VRegisterInfo::getRepRegisterClass(Opcode));

    if (VInstrInfo::isDatapath(Opcode)) {
      // The result of data-path operation is available 1 slot after the
      // operation start.
      Latency = std::max(1u, Latency);
      Def.FinishSlot = std::max(SchedSlot + Latency, Def.FinishSlot);
    }

    ChainValDef *NewDef = new (Allocator.Allocate()) ChainValDef(Def);
    addValDef(NewDef);

    if (!canForwardSrcVal(Opcode)) continue;

    // Link the source and the destinate value define of copy operation together.
    const MachineOperand &CopiedMO = MI->getOperand(1);
    if (CopiedMO.isReg() && CopiedMO.getReg()) {
      ChainValDef *CopiedDef = lookupValDef(CopiedMO.getReg());
      CopiedDef->push_back(NewDef);
    }
  }
}

void VSchedGraph::insertFUCtrlAndCopy() {
  ChainBreaker Breaker(this);

  for (iterator I = CPSUs.begin(), E = CPSUs.end(); I != E; ++I) {
    VSUnit *U = *I;

    if (U->isBBEntry())
      Breaker.updateMBB(U->getRepresentativePtr().get_mbb());

    Breaker.visit(U);
  }
}

static inline bool top_sort_slot(const VSUnit* LHS, const VSUnit* RHS) {
  if (LHS->getSlot() != RHS->getSlot()) return LHS->getSlot() < RHS->getSlot();

  // In the same slot, control-path operations are ahead of data-path operations.
  if (LHS->isControl() != RHS->isControl()) return LHS->isControl();

  return LHS->getIdx() < RHS->getIdx();
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

  // Break the multi-cycles chains to expose more FU sharing opportunities.
  for (iterator I = cp_begin(this), E = cp_end(this); I != E; ++I)
    clearDanglingFlagForTree(*I);

  for (iterator I = dp_begin(this), E = dp_end(this); I != E; ++I) {
    VSUnit *U = *I;

    if (!U->isDangling()) continue;

    // If the SU is scheduled within the boundary of its parent BB, then it is
    // not dangling.
    MachineBasicBlock *ParentBB = U->getParentBB();
    if (U->getSlot() < getEndSlot(ParentBB)) U->setIsDangling(false);
  }

  // Merge the data-path SU vector to the control-path SU vector.
  CPSUs.insert(CPSUs.end(), DPSUs.begin(), DPSUs.end());

  // Sort the SUs by parent BB and its schedule.
  std::sort(CPSUs.begin(), CPSUs.end(), top_sort_bb_and_slot);

  // Break the chains.
  insertFUCtrlAndCopy();

  // Move the dangling node to the last slot of its parent BB, prepare for
  // schedule emission.
  for (iterator I = dp_begin(this), E = dp_end(this); I != E; ++I) {
    VSUnit *U = *I;

    if (!U->isDangling()) continue;

    U->scheduledTo(getEndSlot(U->getParentBB()));
  }
  // We will not use DPSUs anymore.
  DPSUs.clear();

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
  BundleBuilder StateBuilder(*this, MBB, StartSlot);
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
  StateBuilder.finalizeMBB();

  DEBUG(dbgs() << "After schedule emitted:\n");
  DEBUG(dump());
  DEBUG(dbgs() << '\n');
  // Remember the schedule information.
  unsigned LoopOpSlot = StartSlot + getLoopOpSlot(MBB) - getStartSlot(MBB);
  VFI->rememberTotalSlot(MBB, StartSlot, getTotalSlot(MBB), LoopOpSlot);
  // Advance 1 slots after the endslot.
  return StartSlot + getTotalSlot(MBB) + 1;
}
