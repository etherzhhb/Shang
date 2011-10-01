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
#include "vtm/BitLevelInfo.h"

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
  VSUnit *SU = new VSUnit(VIDesc(*I).hasDatapath(), SUCount, fuid);
  ++SUCount;

  SUnits.push_back(SU);
  mapMI2SU(I, SU, 0);
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

void VSchedGraph::removeDeadSU() {
  unsigned Idx = 0;
  for (unsigned i = 0, e = SUnits.size(); i != e; ++i) {
    if (SUnits[i]) {
      // Also update InstIdx.
      SUnits[Idx] = SUnits[i]->updateIdx(Idx);
      ++Idx;
    }
  }

  SUnits.resize(Idx);
  SUCount = Idx;
}

void VSchedGraph::resetSchedule() {
  for (iterator I = begin(), E = end(); I != E; ++I) {
    VSUnit *U = *I;
    U->resetSchedule();
  }
  getEntryRoot()->scheduledTo(startSlot);
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

void VSchedGraph::schedule() {
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
      // Do not try to pipeline the loop if we cannot find MII.
      // FIXME: Just schedule the loop with linear scheduler.
      if (IIFound) {
        if (Scheduler->getCriticalPathLength() >= Scheduler->getMII())
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
  unsigned FinalII = Scheduler->getCriticalPathLength();
  assert((IIFound || Scheduler->getMII() == Scheduler->getCriticalPathLength())
          && "II not found but MII != Critical path length!");

  VSUnit *LoopOp = getLoopOp();
  assert(LoopOp && "Where is Loop op?");
  // Get finish slot?
  assert(LoopOp->getSlot() <= getStartSlot() + FinalII
         && "Loop can not restart in time!");

  // Ditry Hack: Fix the schedule of loop op.
  LoopOp->resetSchedule();
  LoopOp->scheduledTo(getStartSlot() + FinalII);
}

void VSchedGraph::viewGraph() {
  ViewGraph(this, this->getMachineBasicBlock()->getName());
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
  SchedSlot.setSlot(slot);
}

VFUs::FUTypes VSUnit::getFUType() const {
  if (MachineInstr *Instr = getRepresentativeInst())
    return VIDesc(*Instr).getFUType();

  return VFUs::Trivial;
}

bool VSUnit::hasDatapath() const {
  if (MachineInstr *Instr = getRepresentativeInst())
    return VIDesc(*Instr).hasDatapath();

  return false;
}

int8_t VSUnit::getLatencyFor(MachineInstr *MI) const {
  const_instr_iterator at = std::find(instr_begin(), instr_end(), MI);
  assert(at != instr_end() && "Instruction not exist!");
  return latencies[at - instr_begin()];
}

unsigned VSUnit::getLatencyTo(MachineInstr *SrcMI, MachineInstr *DstMI) const {
  int Latency = VInstrInfo::computeLatency(SrcMI, DstMI);
  if (SrcMI != getRepresentativeInst()) {
    Latency += getLatencyFor(SrcMI);
    assert(Latency >= 0 && "Unexpected negative latency!");
  }

  return Latency;
}

void VSUnit::print(raw_ostream &OS) const {
  OS << "[" << getIdx() << "] ";

  for (const_instr_iterator I = instr_begin(), E = instr_end(); I != E; ++I)
    if (MachineInstr *Instr = *I) {
      VIDesc VTID = *Instr;
      OS << Instr->getDesc().getName() << '\n';
      DEBUG(OS << *Instr << '\n');
    }

  OS << getFUId() << "\nAt slot: " << getDetailStep();
}

OpSlot OpSlot::detailStepCeil(int S, bool isDatapath) {
  //OpSlot s(S);

  //// If the type not match, get the next slot.
  //if (s.isControl() != isCtrl)
  //  return s.getNextSlot();

  //return s;
  bool SIsDataPath = S & 0x1;
  bool TypeNotMatch = SIsDataPath != isDatapath;
  return OpSlot(S + TypeNotMatch);
}

OpSlot OpSlot::detailStepFloor(int S, bool isDatapath) {
  //OpSlot s(S);

  //// If the type not match, get the next slot.
  //if (s.isControl() != isCtrl)
  //  return s.getPrevSlot();

  //return s;
  bool SIsDataPath = S & 0x1;
  bool TypeNotMatch = SIsDataPath != isDatapath;
  return OpSlot(S - TypeNotMatch);
}
