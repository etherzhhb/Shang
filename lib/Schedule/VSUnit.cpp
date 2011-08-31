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

  if (MI->getOperand(0).getMBB() != MBB) return false;

  // Ok, remember this instruction as self enable.
  LoopOp.setPointer(MI);
  return true;
}

VSUnit *VSchedGraph::createVSUnit(MachineInstr *I, unsigned fuid) {
  VSUnit *SU = new VSUnit(VIDesc(*I).hasDatapath(), SUCount, fuid);
  ++SUCount;

  SUnits.push_back(SU);
  mapSUnit(I, SU);
  return SU;
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
  }
  return 0;
}

static SchedulingBase *createLoopScheduler(VSchedGraph &G) {
  MachineFunction *F = G.getMachineBasicBlock()->getParent();
  const SynSettings &I = F->getInfo<VFInfo>()->getInfo();
  if (I.getPipeLineAlgorithm() == SynSettings::IMS)
    return new IteractiveModuloScheduling(G);
  
  return createLinearScheduler(G);
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

unsigned VSchedGraph::computeResMII() {
  std::map<FuncUnitId, unsigned> TotalResUsage;
  for (VSchedGraph::iterator I = begin(), E = end(); I != E; ++I) {
    VSUnit *SU = *I;
    if (SU->getFUId().isTrivial()) continue;

    ++TotalResUsage[SU->getFUId()];
  }

  unsigned MaxResII = 0;
  typedef std::map<FuncUnitId, unsigned>::iterator UsageIt;
  for (UsageIt I = TotalResUsage.begin(), E = TotalResUsage.end(); I != E; ++I){
    MaxResII = std::max(MaxResII,
      I->second / I->first.getTotalFUs());
  }
  DEBUG(dbgs() << "ResMII: " << MaxResII << '\n');
  return MaxResII;
}

unsigned VSchedGraph::computeMII() {
  unsigned RecMII = computeRecMII();
  unsigned ResMII = computeResMII();
  return std::max(RecMII, ResMII);
}

void VSchedGraph::scheduleLoop() {
  OwningPtr<SchedulingBase> Scheduler(createLoopScheduler(*this));
  unsigned II = computeMII();

  DEBUG(dbgs() << "MII: " << II << "...");
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
  unsigned FinalII = Scheduler->getMII();
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

void VSUnit::print(raw_ostream &OS) const {
  OS << "[" << getIdx() << "] ";

  for (const_instr_iterator I = instr_begin(), E = instr_end(); I != E; ++I) {
    MachineInstr *Instr = *I;

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
