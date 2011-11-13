//===------------ VSUnit.cpp - Translate LLVM IR to VSUnit  -----*- C++ -*-===//
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
// This file implement the VSUnit class, which represent the basic atom
// operation in hardware.
//
//===----------------------------------------------------------------------===//

#include "VSUnit.h"
#include "ScheduleDOT.h"

#include "vtm/SynSettings.h"
#include "vtm/MicroState.h"
#include "vtm/VFInfo.h"

#include "llvm/Support/raw_ostream.h"

#define DEBUG_TYPE "vtm-sunit"
#include "llvm/Support/Debug.h"

using namespace llvm;


//===----------------------------------------------------------------------===//
void VSchedGraph::print(raw_ostream &OS) const {
  printVMBB(OS, *MBB);
}

void VSchedGraph::dump() const {
  print(dbgs());
}

bool VSchedGraph::trySetLoopOp(MachineInstr *MI) {
  assert(MI->getDesc().isTerminator() && "Bad instruction!");

  if (!VInstrInfo::isBrCndLike(MI->getOpcode())) return false;

  if (MI->getOperand(1).getMBB() != MBB) return false;

  // Ok, remember this instruction as self enable.
  LoopOp.setPointer(MI);
  return true;
}

VSUnit *VSchedGraph::createVSUnit(MachineInstr *I, unsigned fuid) {
  VSUnit *SU = new VSUnit(SUCount, fuid);
  ++SUCount;

  AllSUs.push_back(SU);

  bool mapped = mapMI2SU(I, SU, VInstrInfo::getStepsToFinish(I));
  (void) mapped;
  assert(mapped && "Cannot add SU to the inst2su map!");
  return SU;
}

void VSchedGraph::mergeSU(VSUnit *Src, VSUnit *Dst, int8_t Latency) {
  assert(!Src->isEntry() && "Cannot replace entry!");

  const VSUnit::instr_iterator InstrBase = Src->instr_begin();
  for (VSUnit::instr_iterator I = InstrBase, E = Src->instr_end();
       I != E; ++I) {
    MachineInstr *MI = *I;
    Dst->addInstr(MI, Latency + Src->getLatencyAt(I - InstrBase));
    InstToSUnits[MI] = Dst;
  }

  // Delete source and mark it as dead.
 iterator I = std::find(begin(), end(), Src);
 delete *I;
 *I = 0;
}

// Sort the schedule to place all control schedule unit at the beginning of the
// AllSU vector.
static inline bool sort_by_type(const VSUnit* LHS, const VSUnit* RHS) {
  if (LHS->isControl() != RHS->isControl())
    return LHS->isControl();

  return LHS->getIdx() < RHS->getIdx();
}

void VSchedGraph::removeDeadSU() {
  unsigned Idx = 0;
  for (unsigned i = 0, e = AllSUs.size(); i != e; ++i) {
    if (AllSUs[i]) {
      // Also update InstIdx.
      AllSUs[Idx] = AllSUs[i]->updateIdx(Idx);
      ++Idx;
    }
  }

  AllSUs.resize(Idx);
  SUCount = Idx;
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
  SUsToSched = ArrayRef<VSUnit*>(AllSUs);
  // TODO: Sort them?
}

void VSchedGraph::resetSchedule(unsigned MII) {
  for (sched_iterator I = sched_begin(), E = sched_end(); I != E; ++I) {
    VSUnit *U = *I;
    U->resetSchedule();
  }

  getEntryRoot()->scheduledTo(startSlot);
  // Also schedule the LoopOp to MII step.
  if (MII) {
    assert(hasLoopOp() && "MII provided but LoopOp not exist!");
    getLoopOp()->scheduledTo(startSlot + MII);
  }
}

static SchedulingBase *createLinearScheduler(VSchedGraph &G) {
  MachineFunction *F = G.getMachineBasicBlock()->getParent();
  const SynSettings &I = F->getInfo<VFInfo>()->getInfo();

  switch (I.getScheduleAlgorithm()) {
  case SynSettings::FDS:
  case SynSettings::FDLS:
    errs() << "Force-directed scheduler was temporary removed!\n"
              "Going to use ILP scheduler.\n";
  case SynSettings::ILP:   return new ILPScheduler(G);
  case SynSettings::ASAP:  return new ASAPScheduler(G);
  }
  return 0;
}

static SchedulingBase *createLoopScheduler(VSchedGraph &G) {
  MachineFunction *F = G.getMachineBasicBlock()->getParent();
  const SynSettings &I = F->getInfo<VFInfo>()->getInfo();
  switch (I.getPipeLineAlgorithm()) {
  case SynSettings::IMS:
    return new IterativeModuloScheduling(G);
  case SynSettings::ILPMS:
    return new ILPScheduler(G);
  default:
    return createLinearScheduler(G);
  }
}

void VSchedGraph::scheduleCtrl() {
  if (enablePipeLine())
    scheduleLoop();
  else
    scheduleLinear();
}

void VSchedGraph::scheduleLinear() {
  OwningPtr<SchedulingBase> Scheduler(createLinearScheduler(*this));

  while (!Scheduler->scheduleState())
    Scheduler->lengthenCriticalPath();

  DEBUG(Scheduler->dumpTimeFrame());
}

void VSchedGraph::scheduleLoop() {
  DEBUG(dbgs() << "Try to pipeline MBB#" << MBB->getNumber()
               << " MF#" << MBB->getParent()->getFunctionNumber() << '\n');
  OwningPtr<SchedulingBase> Scheduler(createLoopScheduler(*this));
  // Ensure us can schedule the critical path.
  while (!Scheduler->scheduleCriticalPath(true))
    Scheduler->lengthenCriticalPath();

  // computeMII may return a very big II if we cannot compute the RecII.
  bool IIFound = Scheduler->computeMII();
  DEBUG(dbgs() << "Pipelining BB# " << MBB->getNumber()
               << " in function " << MBB->getParent()->getFunction()->getName()
               << " #" << MBB->getParent()->getFunctionNumber() << '\n');

  DEBUG(dbgs() << "MII: " << Scheduler->getMII() << "...");
  while (!Scheduler->scheduleCriticalPath(true)) {
    // Make sure MII smaller than the critical path length.
    if (Scheduler->getMII() < Scheduler->getCriticalPathLength())
      Scheduler->increaseMII();
    else
      Scheduler->lengthenCriticalPath();
  }

  assert(Scheduler->getMII() <= Scheduler->getCriticalPathLength()
         && "MII bigger then Critical path length!");

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
      // Do not try to pipeline the loop if we cannot find MII.
      // FIXME: Just schedule the loop with linear scheduler.
      if (IIFound) {
        if (Scheduler->getCriticalPathLength() > Scheduler->getMII())
          NextPoints.push_back(std::make_pair(CurPoint.first + 1, CurPoint.second));
        NextPoints.push_back(std::make_pair(CurPoint.first, CurPoint.second  + 1));
      }
      // Add both by default.
      CurPoint = std::make_pair(CurPoint.first + 1, CurPoint.second  + 1);
    }

    Scheduler->setMII(NextPoints.back().first);
    Scheduler->setCriticalPathLength(NextPoints.back().second);
    NextPoints.pop_back();
  }
  DEBUG(dbgs() << "SchedII: " << Scheduler->getMII()
               << " Latency: " << getTotalSlot() << '\n');
  assert(getII() == Scheduler->getMII()
         && "LoopOp was not scheduled to the right slot!");
  assert(getLoopOpSlot() <= getEndSlot()
         && "Expect MII is not bigger then critical path length!");
}

void VSchedGraph::viewGraph() {
  ViewGraph(this, this->getMachineBasicBlock()->getName());
}

void VSchedGraph::scheduleDatapath() {
  for (iterator I = AllSUs.begin(), E = AllSUs.end(); I != E; ++I) {
    VSUnit *DU = *I;
    if (DU->isScheduled()) continue;

    assert(DU->isDatapath() && "Unexpected ctrl-sunit not to yet scheduled!");
    unsigned Step = 0;
    for (VSUnit::dep_iterator DI = DU->dep_begin(), DE = DU->dep_end();
         DI != DE; ++DI) {
      VSUnit *DepSU = *DI;
      assert(DepSU->isScheduled() && "Datapath dependence not schedule!");
      Step = std::max(Step, DepSU->getSlot() + DI.getEdge()->getLatency());
    }
    assert(Step && "Datapath SU do not have depending SUs?");
    // Schedule As soon as possible.
    DU->scheduledTo(Step);
  }
}

//===----------------------------------------------------------------------===//

void VSUnit::dump() const {
  print(dbgs());
  dbgs() << '\n';
}

void VDEdge::print(raw_ostream &OS) const {}

unsigned VSUnit::getOpcode() const {
  if (MachineInstr *I = getRepresentativeInst())
    return I->getOpcode();

  return VTM::INSTRUCTION_LIST_END;
}

void VSUnit::scheduledTo(unsigned slot) {
  assert(slot && "Can not schedule to slot 0!");
  SchedSlot = slot;
}

VFUs::FUTypes VSUnit::getFUType() const {
  if (MachineInstr *Instr = getRepresentativeInst())
    return VInstrInfo::getFUType(Instr->getOpcode());

  return VFUs::Trivial;
}

bool VSUnit::isDatapath() const {
  if (MachineInstr *Instr = getRepresentativeInst())
    return VInstrInfo::isDatapath(Instr->getOpcode());

  return false;
}

int8_t VSUnit::getLatencyFor(MachineInstr *MI) const {
  const_instr_iterator at = std::find(instr_begin(), instr_end(), MI);
  assert(at != instr_end() && "Instruction not exist!");
  return getLatencyAt(at - instr_begin());
}

unsigned VSUnit::getLatencyTo(MachineInstr *SrcMI, MachineInstr *DstMI) const {
  int Latency = VInstrInfo::getCtrlStepBetween(SrcMI, DstMI);
  if (SrcMI != getRepresentativeInst()) {
    Latency += getLatencyFor(SrcMI);
    assert(Latency >= 0 && "Unexpected negative latency!");
  }

  return Latency;
}

unsigned VSUnit::getLatencyFrom(MachineInstr *SrcMI, unsigned SrcLatency) const{
  int Latency = SrcLatency;
  if (SrcMI != getRepresentativeInst()) {
    Latency += getLatencyFor(SrcMI);
    assert(Latency >= 0 && "Unexpected negative latency!");
  }

  return Latency;
}

void VSUnit::print(raw_ostream &OS) const {
  OS << "[" << getIdx() << "] ";

  const VSUnit::const_instr_iterator InstrBase = instr_begin();
  for (const_instr_iterator I = InstrBase, E = instr_end(); I != E; ++I)
    if (MachineInstr *Instr = *I) {
      OS << Instr->getDesc().getName();
      unsigned Idx = I - InstrBase;
      if (Idx) OS << '+' << unsigned(getLatencyAt(Idx));
      OS << '\n';
      DEBUG(OS << *Instr << '\n');
    }

  OS << getFUId() << "\nAt slot: " << getSlot();
}
