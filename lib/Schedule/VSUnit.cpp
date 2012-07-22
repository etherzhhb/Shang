//===------------ VSUnit.cpp - Translate LLVM IR to VSUnit  -----*- C++ -*-===//
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
// This file implement the VSUnit class, which represent the basic atom
// operation in hardware.
//
//===----------------------------------------------------------------------===//

#include "VSUnit.h"
#include "ScheduleDOT.h"

#include "vtm/VerilogBackendMCTargetDesc.h"
#include "vtm/SynSettings.h"
#include "vtm/VFInfo.h"

#include "llvm/ADT/PostOrderIterator.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/CommandLine.h"
#define DEBUG_TYPE "vtm-sunit"
#include "llvm/Support/Debug.h"

using namespace llvm;

static cl::opt<bool>
ScheduleDataPathALAP("vtm-schedule-datapath-alap",
                     cl::desc("Schedule datapath operaton As Last As Possible "
                              "to allow more effecient resource sharing"),
                     cl::init(true)),
// FIXME: When multi-cycles chain is disabled, also set the chaining threshold
// of all functional units to 0 to avoid chaining.
DisableMultiCyclesChain("vtm-disable-multi-cycles-chain",
                        cl::desc("Disable multi-cycles chaining "
                                 "(manually setting all chaining threshold to "
                                 "0 is need)"),
                        cl::init(false));

//===----------------------------------------------------------------------===//
void VSchedGraph::print(raw_ostream &OS) const {
  getEntryBB()->dump();
}

void VSchedGraph::dump() const {
  print(dbgs());
}

bool VSchedGraph::trySetLoopOp(MachineInstr *MI) {
  assert(MI->getDesc().isTerminator() && "Bad instruction!");

  if (!VInstrInfo::isBrCndLike(MI->getOpcode())) return false;

  if (MI->getOperand(1).getMBB() != getEntryBB()) return false;

  // Ok, remember this instruction as self enable.
  LoopOp.setPointer(MI);
  return true;
}

bool VSchedGraph::isLoopPHIMove(MachineInstr *MI) {
  assert(MI->getOpcode() == VTM::VOpMvPhi && "Bad opcode!");

  return MI->getOperand(2).getMBB() == getEntryBB() && enablePipeLine();
}

void VSchedGraph::verify() const {
  if (getEntryRoot()->num_deps())
    llvm_unreachable("Entry root should not have any dependence!");
  if (getExitRoot()->num_uses())
    llvm_unreachable("Exit root should not have any use!");

  for (sched_iterator I = sched_begin(), E = sched_end(); I != E; ++I)
    verifySU(*I);
}

void VSchedGraph::verifySU(const VSUnit *SU) const {
  typedef VSUnit::const_dep_iterator dep_it;

  bool IsBBEntry = SU->getRepresentativePtr().isMBB();
  MachineBasicBlock *ParentMBB = SU->getParentBB();
  bool AnyDepFromTheSameParent = IsBBEntry;

  for (dep_it DI = SU->dep_begin(), DE = SU->dep_end(); DI != DE; ++DI) {
    const VSUnit *Dep = *DI;
    assert((DI.getEdgeType() == VDEdge::edgeMemDep
            || SU->getIdx() > Dep->getIdx())
           && "Bad value dependent edge!");
    assert((!IsBBEntry || (Dep->getRepresentativePtr()->isTerminator()
                           && DI.getLatency() == 0))
           && "Bad inter BB dependent edge.");
    AnyDepFromTheSameParent |= DI->getParentBB() == ParentMBB;
  }

  assert((SU->isScheduled() || !SU->use_empty() || SU == getExitRoot())
          && "Unexpected deteched SU!");
  assert((SU->isScheduled() || SU->hasFixedTiming() || AnyDepFromTheSameParent)
         && "Find SU not constrained by the BB entry!");
}

VSUnit *VSchedGraph::createVSUnit(InstPtrTy Ptr, unsigned fuid) {
  VSUnit *SU = new VSUnit(NextSUIdx++, fuid);

  AllSUs.push_back(SU);

  MachineInstr *MI = Ptr;
  bool mapped = mapMI2SU(Ptr, SU, MI ? DLInfo.getStepsToFinish(MI) : 0);
  (void) mapped;
  assert(mapped && "Cannot add SU to the inst2su map!");
  return SU;
}

// Sort the schedule to place all control schedule unit at the beginning of the
// AllSU vector.
static inline bool sort_by_type(const VSUnit* LHS, const VSUnit* RHS) {
  if (LHS->isControl() != RHS->isControl())
    return LHS->isControl();

  return LHS->getIdx() < RHS->getIdx();
}

VSchedGraph::iterator
VSchedGraph::mergeSUsInSubGraph(VSchedGraph &SubGraph) {
  // 1. Merge the MI2SU map.
  // Prevent the virtual exit root from being inserted to the current MI2SU map.
  SubGraph.InstToSUnits.erase(SubGraph.getExitRoot()->getRepresentativePtr());

  InstToSUnits.insert(SubGraph.InstToSUnits.begin(),
                      SubGraph.InstToSUnits.end());

  // 2. Add the SUs in subgraph to the SU list of current SU list.
  unsigned NewIdxBase = NextSUIdx;
  assert(AllSUs.size() == NextSUIdx - FirstSUIdx && "Index mis-matched!");
  VSUnit *Terminator = SubGraph.lookUpTerminator(SubGraph.getEntryBB());
  assert(Terminator && "Terminator SU not found!");
  // We need to clear the dependencies of the terminator before we adding
  // local schedule constraint.
  Terminator->cleanDepAndUse();
  unsigned TerminatorSlot = Terminator->getSlot();

  for (iterator I = SubGraph.begin(), E = SubGraph.end(); I != E; ++I) {
    VSUnit *U = *I;

    // Ignore the virtual exit root.
    if (U == SubGraph.getExitRoot()) continue;

    // Update the index of the scheduled SU.
    AllSUs.push_back(U->updateIdx(NewIdxBase + U->getIdx() - FirstSUIdx));
    ++NextSUIdx;

    // We may need to build a fixed timing dependencies according to the
    // schedule of U.
    unsigned USlot = U->getSlot();
    U->resetSchedule();

    // Do not add the edge from a data-path SU, which is not scheduled yet.
    // And do not add self-loop to the terminator.
    if (U->isDatapath()) {
      assert(U->num_deps() == 0 && U->num_uses() == 0
             && "Unexpected data-path dependencies!");
      continue;
    }

    // All scheduled control SU has fixed a timing constraint.
    U->setHasFixedTiming();

    // Do not add loop.
    if (U == Terminator) continue;

    assert(USlot && "Unexpected unscheduled SU!");
    unsigned ScheduleOffset = TerminatorSlot - USlot;
    // We need to build the new dependencies.
    U->cleanDepAndUse();
    // A dependencies to constrain the SU with local schedule.
    Terminator->addDep(U, VDEdge::CreateFixTimingConstraint(ScheduleOffset));
  }

  SubGraph.AllSUs.clear();

  // 3. Merge the II Map.
  IIMap.insert(SubGraph.IIMap.begin(), SubGraph.IIMap.end());

  //4. Merge the terminator map.
  Terminators.insert(SubGraph.Terminators.begin(), SubGraph.Terminators.end());

  // No need to subtract by FirstSUIdx because we need to skip the entry node
  // of the block.
  return AllSUs.begin() + NewIdxBase/*- FirstSUIdx*/;
}

void VSchedGraph::topologicalSortScheduleUnits() {
  unsigned Idx = 0;
  VSUnit *Exit = getExitRoot();
  typedef po_iterator<VSUnit*, SmallPtrSet<VSUnit*, 64>, false,
                      GraphTraits<Inverse<VSUnit*> > >
          top_it;

  for (top_it I = top_it::begin(Exit), E = top_it::end(Exit); I != E; ++I)
    AllSUs[Idx++] = *I;

  assert(Idx == num_scheds() && "Bad topological sort!");
}

void VSchedGraph::prepareForCtrlSched() {
  unsigned NumCtrls = 0;
  std::sort(AllSUs.begin(), AllSUs.end(), sort_by_type);

#ifndef NDEBUG
  int LastIdx = -1;
#endif
  for (iterator I = begin(), E = end(); I != E; ++I) {
    VSUnit *U = *I;
    if (U->isDatapath())
      break;

    ++NumCtrls;
#ifndef NDEBUG
    assert(LastIdx < U->getIdx() && "Schedule units not sorted!");
#endif
  }

  SUsToSched = ArrayRef<VSUnit*>(AllSUs.data(), NumCtrls);
}

void VSchedGraph::prepareForDatapathSched() {
  for (sched_iterator I = sched_begin(), E = sched_end(); I != E; ++I) {
    VSUnit *U = *I;
    assert(U->isControl() && "Unexpected datapath op in to schedule list!");
    U->cleanDepAndUse();
  }

  SUsToSched = ArrayRef<VSUnit*>(AllSUs);
  // TODO: Sort them?
}

void VSchedGraph::resetSchedule(unsigned MII) {
  for (sched_iterator I = sched_begin(), E = sched_end(); I != E; ++I) {
    VSUnit *U = *I;
    U->resetSchedule();
  }

  getEntryRoot()->scheduledTo(EntrySlot);
  // Also schedule the LoopOp to MII step.
  if (MII) {
    assert(hasLoopOp() && "MII provided but LoopOp not exist!");
    getLoopOp()->scheduledTo(EntrySlot + MII);
  }

  // Make sure the PHI copy emit before the BB jump to other BBs.
  typedef VSUnit::dep_iterator dep_it;
  VSUnit *ExitRoot = getExitRoot();
  for (dep_it I = ExitRoot->dep_begin(), E = ExitRoot->dep_end(); I != E; ++I)
    if (I->isPHI()) I.getEdge().setLatency(MII);
}

static SchedulingBase *createLinearScheduler(VSchedGraph &G, MachineFunction *F)
{
  const SynSettings &I = F->getInfo<VFInfo>()->getInfo();

  switch (I.getScheduleAlgorithm()) {
  case SynSettings::ILP:   return new ILPScheduler(G);
  case SynSettings::ASAP:  return new ASAPScheduler(G);
  case SynSettings::SDC:  return new SDCScheduler(G);
  }
  return 0;
}

static SchedulingBase *createLoopScheduler(VSchedGraph &G, MachineFunction *F) {
  const SynSettings &I = F->getInfo<VFInfo>()->getInfo();
  switch (I.getPipeLineAlgorithm()) {
  case SynSettings::IMS:
    return new IterativeModuloScheduling(G);
  case SynSettings::ILPMS:
    return new ILPScheduler(G);
  default:
    return createLinearScheduler(G, F);
  }
}

void VSchedGraph::scheduleCtrl() {
  if (enablePipeLine())
    scheduleLoop();
  else
    scheduleLinear();
}

void VSchedGraph::scheduleLinear() {
  MachineFunction *F = getEntryBB()->getParent();
  OwningPtr<SchedulingBase> Scheduler(createLinearScheduler(*this, F));

  while (!Scheduler->scheduleState())
    Scheduler->lengthenCriticalPath();

  DEBUG(Scheduler->dumpTimeFrame());
}

void VSchedGraph::scheduleLoop() {
  MachineBasicBlock *MBB = getEntryBB();
  MachineFunction *F = MBB->getParent();
  DEBUG(dbgs() << "Try to pipeline MBB#" << MBB->getNumber()
               << " MF#" << F->getFunctionNumber() << '\n');
  OwningPtr<SchedulingBase> Scheduler(createLoopScheduler(*this, F));
  // Ensure us can schedule the critical path.
  while (!Scheduler->scheduleCriticalPath(true))
    Scheduler->lengthenCriticalPath();

  // computeMII may return a very big II if we cannot compute the RecII.
  Scheduler->computeMII();
  DEBUG(dbgs() << "Pipelining BB# " << MBB->getNumber()
               << " in function " << MBB->getParent()->getFunction()->getName()
               << " #" << MBB->getParent()->getFunctionNumber() << '\n');

  DEBUG(dbgs() << "MII: " << Scheduler->getMII() << "...");
  while (!Scheduler->scheduleCriticalPath(true)) {
    // Make sure MII smaller than the critical path length.
    if (2 * Scheduler->getMII() < Scheduler->getCriticalPathLength())
      Scheduler->increaseMII();
    else
      Scheduler->lengthenCriticalPath();
  }

  assert(Scheduler->getMII() <= Scheduler->getCriticalPathLength()
         && "MII bigger then Critical path length!");

  while (!Scheduler->scheduleState()) {
    // Make sure MII smaller than the critical path length.
    if (Scheduler->getMII() < Scheduler->getCriticalPathLength())
      Scheduler->increaseMII();
    else
      Scheduler->lengthenCriticalPath();
  }

  DEBUG(dbgs() << "SchedII: " << Scheduler->getMII()
               << " Latency: " << getTotalSlot(MBB) << '\n');
  assert(getLoopOp()->getSlot() - EntrySlot == Scheduler->getMII()
         && "LoopOp was not scheduled to the right slot!");
  assert(getLoopOp()->getSlot() <= getEndSlot(MBB)
         && "Expect MII is not bigger then critical path length!");

  bool succ = IIMap.insert(std::make_pair(MBB, Scheduler->getMII())).second;
  assert(succ && "Cannot remember II!");
  (void) succ;

  fixPHISchedules(begin(), begin() + num_scheds());
}

void VSchedGraph::viewGraph() {
  ViewGraph(this, this->getEntryBB()->getName());
}

void VSchedGraph::fixChainedDatapathRC(VSUnit *U) {
  assert(U->isDatapath() && "Expected datapath operation!");
  assert(U->num_instrs() == 1 && "Unexpected datapath operation merged!");

  MachineInstr *MI = U->getRepresentativePtr();
  MachineBasicBlock *ParentMBB = MI->getParent();
  const DetialLatencyInfo::DepLatInfoTy *DepLatInfo = DLInfo.getDepLatInfo(MI);
  assert(DepLatInfo && "dependence latency information not available?");

  typedef DetialLatencyInfo::DepLatInfoTy::const_iterator dep_it;
  // If multi-cycle chain is disable, a copy is always need.
  bool NeedCopyToReg = DisableMultiCyclesChain;

  for (dep_it I = DepLatInfo->begin(), E = DepLatInfo->end(); I != E; ++I) {
    const MachineInstr *SrcMI = I->first;

    // Ignore the entry root.
    if (SrcMI == 0 || SrcMI->getParent() != ParentMBB)
      continue;

    unsigned SrcOpC = SrcMI->getOpcode();
    // Ignore the operations without interesting function unit.
    if (VInstrInfo::hasTrivialFU(SrcOpC)) continue;

    VSUnit *SrcSU = lookupSUnit(SrcMI);
    assert(SrcSU && "Source schedule unit not exist?");
    unsigned SrcCopySlot =
      SrcSU->getFinSlot() + VInstrInfo::isWriteUntilFinish(SrcOpC);
    // Is the datapath operation chained with its depending control operation?
    if (SrcCopySlot > U->getSlot()) {
      NeedCopyToReg = true;
      // FIXME: Also compute the optimize copy slot.
      break;
    }
  }

  if (!NeedCopyToReg) {
    assert(MI->getDesc().getNumDefs() == 1
           && "Expect datapath operation have only 1 define!");

    unsigned Reg = MI->getOperand(0).getReg();
    DLInfo.MRI.setRegClass(Reg, &VTM::WireRegClass);
  }
}

void VSchedGraph::scheduleDatapathALAP() {
  typedef SUnitVecTy::reverse_iterator rev_it;
  for (rev_it I = AllSUs.rbegin(), E = AllSUs.rend(); I != E; ++I) {
    VSUnit *A = *I;
    if (A->isScheduled()) continue;

    MachineBasicBlock *MBB = A->getParentBB();
    unsigned Step = getEndSlot(MBB);

    for (VSUnit::use_iterator UI = A->use_begin(), UE = A->use_end();
         UI != UE; ++UI) {
      const VSUnit *Use = *UI;
      VDEdge UseEdge = Use->getEdgeFrom(A);
      assert(Use->isScheduled() && "Expect use scheduled!");

      unsigned UseSlot = Use->getSlot();
      // We had already adjusted the schedule the PHI, so do not add the II
      // to UseSlot again.
      if (isPipelined(MBB) && !Use->isPHI())
        UseSlot += (getII(MBB) * UseEdge.getDistance());
      unsigned CurStep = UseSlot - UseEdge.getLatency();
      // All control operations are read at emit, do not schedule the datapath
      // operation which is the control operation depends on to the same slot
      // with the control operation.
      if (Use->isControl()) CurStep = std::min(UseSlot - 1, CurStep);

      Step = std::min(CurStep, Step);
    }

    assert(Step < getEndSlot(MBB) && Step >= getStartSlot(MBB)
           && "Bad schedule for datapath SU!");

    // Schedule As late as possible to reduce register usage.
    A->scheduledTo(Step);

    fixChainedDatapathRC(A);
  }
}

void VSchedGraph::scheduleDatapathASAP() {
  for (iterator I = AllSUs.begin(), E = AllSUs.end(); I != E; ++I) {
    VSUnit *A = *I;
    if (A->isScheduled()) continue;

    MachineBasicBlock *MBB = A->getParentBB();
    unsigned Step = getStartSlot(getEntryBB());
    for (VSUnit::dep_iterator DI = A->dep_begin(), DE = A->dep_end();
         DI != DE; ++DI) {
      const VSUnit *DepSU = *DI;
      assert(DepSU->isScheduled() && "Datapath dependence not schedule!");
      unsigned NewStep = DepSU->getSlot() + DI.getLatency();
      if (isPipelined(MBB)) NewStep -= getII(MBB) * DI.getDistance();

      Step = std::max(Step, NewStep);
    }

    assert(Step < getEndSlot(MBB) && Step >= getStartSlot(MBB)
           && "Bad schedule for datapath SU!");
    // Schedule As soon as possible.
    A->scheduledTo(Step);

    fixChainedDatapathRC(A);
  }
}

void VSchedGraph::fixPHISchedules(iterator su_begin, iterator su_end) {
  // Fix the schedule of PHI's so we can emit the incoming copies at a right
  // slot;
  for (iterator I = su_begin, E = su_end; I != E; ++I) {
    VSUnit *U = *I;
    if (!U->isPHI()) continue;

    MachineBasicBlock *MBB = U->getParentBB();
    // Schedule the SU to the slot of the PHI Move.
    U->scheduledTo(U->getSlot() + getII(MBB));
    assert(U->getSlot() <= getEndSlot(MBB) && "Bad PHI schedule!");
  }
}

void VSchedGraph::scheduleDatapath() {
  if (ScheduleDataPathALAP) scheduleDatapathALAP();
  else                      scheduleDatapathASAP();
}

//===----------------------------------------------------------------------===//

void VSUnit::dump() const {
  print(dbgs());
  dbgs() << '\n';
}

void VDEdge::print(raw_ostream &OS) const {}

// TODO: Implement edge bundle, calculate the edge for
void llvm::VSUnit::addDep(VSUnit *Src, VDEdge NewE) {
  assert(Src != this && "Cannot add self-loop!");
  edge_iterator at = Deps.find(Src);

  NewE.setIsCrossBB(Src->getParentBB() != getParentBB());

  if (at == Deps.end()) {
    Deps.insert(std::make_pair(Src, NewE));
    Src->addToUseList(this);
    return;
  }

  VDEdge &CurE = at->second;
  assert(NewE.getEdgeType() != VDEdge::edgeFixedTiming
         && CurE.getEdgeType() != VDEdge::edgeFixedTiming
         && "Cannot override fixed timing dependencies!");
  // If the new dependency constraint tighter?
  assert((NewE.getDistance() == 0 || CurE.getDistance() == 0
          || CurE.getDistance() == NewE.getDistance())
         && "Unexpected multiple loop carried dependencies!");
  if (NewE.getDistance() <= CurE.getDistance()
      && NewE.getLatency() > CurE.getLatency()) {
    CurE = NewE;
  }
}

VSUnit::VSUnit(unsigned short Idx, uint16_t FUNum)
  : SchedSlot(0), IsDangling(true), HasFixedTiming(false), InstIdx(Idx),
    FUNum(FUNum) {
  assert(Idx > VSchedGraph::NullSUIdx && "Bad index!");
}

VSUnit::VSUnit(MachineBasicBlock *MBB, uint16_t Idx)
  : SchedSlot(0), IsDangling(true), HasFixedTiming(false), InstIdx(Idx),
    FUNum(0) {
  assert(Idx > VSchedGraph::NullSUIdx && "Bad index!");
  Instrs.push_back(MBB);
  latencies.push_back(0);
}

VSUnit *VSUnit::updateIdx(unsigned short Idx) {
  assert(Idx > VSchedGraph::NullSUIdx && "Bad index!");

  InstIdx = Idx;
  return this;
}

unsigned VSUnit::countValDeps() const {
  unsigned DepCounter = 0;

  for(const_dep_iterator I = dep_begin(), E = dep_end(); I != E; ++I) {
    if(I.getEdgeType() != VDEdge::edgeValDep) continue;

    ++DepCounter;
  }

  return DepCounter;
}

unsigned VSUnit::countValUses() const {
  unsigned DepCounter = 0;

  for(const_use_iterator I = use_begin(), E = use_end(); I != E; ++I){
    const VSUnit* V =*I;
    if(V->getEdgeFrom(this).getEdgeType() != VDEdge::edgeValDep) continue;

    ++DepCounter;
  }

  return DepCounter;
}

unsigned VSUnit::getOpcode() const {
  if (MachineInstr *I = getRepresentativePtr())
    return I->getOpcode();

  return VTM::INSTRUCTION_LIST_END;
}

void VSUnit::scheduledTo(unsigned slot) {
  assert(slot && "Can not schedule to slot 0!");
  SchedSlot = slot;
  // TODO: Assert the schedule is not locked?
}

VFUs::FUTypes VSUnit::getFUType() const {
  if (MachineInstr *Instr = getRepresentativePtr())
    return VInstrInfo::getFUType(Instr->getOpcode());

  return VFUs::Trivial;
}

bool VSUnit::isDatapath() const {
  if (MachineInstr *Instr = getRepresentativePtr())
    return VInstrInfo::isDatapath(Instr->getOpcode());

  return false;
}

int8_t VSUnit::getLatencyFor(MachineInstr *MI) const {
  const_instr_iterator at = std::find(instr_begin(), instr_end(), MI);
  assert(at != instr_end() && "Instruction not exist!");
  return getLatencyAt(at - instr_begin());
}

int VSUnit::getLatencyFrom(MachineInstr *SrcMI, int SrcLatency) const{
  int Latency = SrcLatency;
  if (SrcMI != getRepresentativePtr()) {
    Latency += getLatencyFor(SrcMI);
  }

  return Latency;
}

void VSUnit::print(raw_ostream &OS) const {
  OS << "[" << getIdx() << "] MBB#" << getParentBB()->getNumber() << ' ';

  for (unsigned i = 0, e = num_instrs(); i < e; ++i) {
    InstPtrTy Ptr = getPtrAt(i);
    if (MachineInstr *Instr = Ptr.dyn_cast_mi()) {
      const TargetInstrInfo *TII = Instr->getParent()->getParent()->getTarget()
                                         .getInstrInfo();
      OS << TII->getName(Instr->getDesc().getOpcode()) << ' ';
      if (!Instr->isPseudo())
        OS << *VInstrInfo::getTraceOperand(Instr);
      if (i) OS << ' ' << int(getLatencyAt(i));
      OS << '\n';
      DEBUG(OS << *Instr << '\n');
    }
  }

  OS << getFUId() << "\nAt slot: " << getSlot();
  if (isDangling()) OS << " <Dangling>";
}
