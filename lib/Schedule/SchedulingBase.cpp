//===- ForceDirectedSchedulingBase.cpp - ForceDirected information analyze --*- C++ -*-===//
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
// This file implement the Force Direct information computation pass describe in
// Force-Directed Scheduling for the Behavioral Synthesis of ASIC's
//
//===----------------------------------------------------------------------===//

#include "SchedulingBase.h"
#include "ScheduleDOT.h"
#include "vtm/Passes.h"

#include "llvm/Support/CommandLine.h"

#define DEBUG_TYPE "vbe-fd-info"
#include "llvm/Support/Debug.h"

using namespace llvm;
using namespace llvm;

//===----------------------------------------------------------------------===//
void SchedulingBase::buildTimeFrame() {
  VSUnit *EntryRoot = State.getEntryRoot();
  assert(EntryRoot->isScheduled() && "Entry must be scheduled first!");

  // Reset the time frames
  typedef std::map<const VSUnit*, TimeFrame> TFMapTy;
  for (TFMapTy::iterator I = SUnitToTF.begin(), E = SUnitToTF.end();I != E;++I)
    I->second = std::make_pair(0, VSUnit::MaxSlot);

  // Build the time frames
  buildASAPStep();
  buildALAPStep();

  DEBUG(dumpTimeFrame());
}

void SchedulingBase::buildASAPStep() {
  VSUnit *Entry = State.getEntryRoot();
  SUnitToTF[Entry].first = Entry->getSlot();
  typedef VSchedGraph::sched_iterator it;
  it Start = State.sched_begin();

  bool changed = false;

  // Build the time frame iteratively.
  do {
    changed = false;
    for (it I = Start + 1, E = State.sched_end(); I != E; ++I) {
      VSUnit *A = *I;
      if (A->isScheduled()) {
        SUnitToTF[A].first = A->getSlot();
        continue;
      }

      DEBUG(dbgs() << "\n\nCalculating ASAP step for \n";
            A->dump(););

      unsigned NewStep = 0;
      for (VSUnit::dep_iterator DI = A->dep_begin(), DE = A->dep_end();
          DI != DE; ++DI) {
        const VSUnit *Dep = *DI;
        if (!DI.getEdge()->isLoopCarried() || MII) {
          unsigned DepASAP = Dep->isScheduled() ?
                             Dep->getSlot() : getASAPStep(Dep);
          int Step = DepASAP + DI.getEdge()->getLatency()
                     - (MII * DI.getEdge()->getItDst());
          DEBUG(dbgs() << "From ";
                if (DI.getEdge()->isLoopCarried())
                  dbgs() << "BackEdge ";
                Dep->print(dbgs());
                dbgs() << " Step " << Step << '\n');
          unsigned UStep = std::max(0, Step);
          NewStep = std::max(UStep, NewStep);
        }
      }

      DEBUG(dbgs() << "Update ASAP step to: " << NewStep << " for \n";
      A->dump();
      dbgs() << "\n\n";);

      unsigned &ASAPStep = SUnitToTF[A].first;
      if (ASAPStep != NewStep) {
        ASAPStep = NewStep;
        changed |= true;
      }
    }
  } while (changed);

  VSUnit *Exit = State.getExitRoot();
  CriticalPathEnd = std::max(CriticalPathEnd, getASAPStep(Exit));
}

void SchedulingBase::buildALAPStep() {
  VSUnit *Exit = State.getExitRoot();
  int LastSlot = CriticalPathEnd;
  SUnitToTF[Exit].second = LastSlot;

  bool changed = false;
  // Build the time frame iteratively.
  do {
    changed = false;
    for (int Idx = State.num_scheds()/*skip exitroot*/- 2; Idx >= 0; --Idx){
      VSUnit *A = State.getCtrlAt(Idx);
      if (A->isScheduled()) {
        SUnitToTF[A].second = A->getSlot();
        continue;
      }

      DEBUG(dbgs() << "\n\nCalculating ALAP step for \n";
            A->dump(););
      unsigned NewStep = VSUnit::MaxSlot;
      for (VSUnit::use_iterator UI = A->use_begin(), UE = A->use_end();
           UI != UE; ++UI) {
        const VSUnit *Use = *UI;
        VDEdge *UseEdge = Use->getEdgeFrom(A);

        if (!UseEdge->isLoopCarried() || MII) {
          unsigned UseALAP = Use->isScheduled() ?
                             Use->getSlot() : getALAPStep(Use);
          if (UseALAP == 0) {
            assert(UseEdge->isLoopCarried() && "Broken time frame!");
            UseALAP = VSUnit::MaxSlot;
          }
          unsigned Step = UseALAP - UseEdge->getLatency()
                          + (MII * UseEdge->getItDst());
          DEBUG(dbgs() << "From ";
                if (UseEdge->isLoopCarried())
                  dbgs() << "BackEdge ";
                Use->print(dbgs());
                dbgs() << " Step " << Step << '\n');
          NewStep = std::min(Step, NewStep);
        }
      }

      DEBUG(dbgs() << "Update ALAP step to: " << NewStep << " for \n";
            A->dump();
            dbgs() << "\n\n";);

      unsigned &ALAPStep = SUnitToTF[A].second;
      if (ALAPStep != NewStep) {
        assert(getASAPStep(A) <= NewStep && "Broken ALAP step!");
        ALAPStep = NewStep;
        changed = true;
      }
    }
  } while (changed);
}

void SchedulingBase::printTimeFrame(raw_ostream &OS) const {
  OS << "Time frame:\n";
  typedef VSchedGraph::sched_iterator it;
  for (it I = State.sched_begin(), E = State.sched_end(); I != E; ++I) {
    VSUnit *A = *I;
    A->print(OS);
    OS << " : {" << getASAPStep(A) << "," << getALAPStep(A)
      << "} " <<  getTimeFrame(A);

    for (VSUnit::dep_iterator DI = A->dep_begin(), DE = A->dep_end(); DI != DE;
        ++DI)
      OS << " [" << DI->getIdx() << "]"; 
    
    OS << '\n';
  }
}

unsigned SchedulingBase::computeResMII() {
  // FIXME: Compute the resource area cost
  std::map<FuncUnitId, unsigned> TotalResUsage;
  typedef VSchedGraph::sched_iterator it;
  for (it I = State.sched_begin(), E = State.sched_end(); I != E; ++I) {
    VSUnit *SU = *I;
    if (!SU->getFUId().isBound()) continue;

    ++TotalResUsage[SU->getFUId()];
  }

  unsigned MaxResII = 0;
  typedef std::map<FuncUnitId, unsigned>::iterator UsageIt;
  for (UsageIt I = TotalResUsage.begin(), E = TotalResUsage.end(); I != E; ++I){
    /*There is only 1 resource avaialbe for Prebound function unit kind*/
    const unsigned NumFUs = 1;
    MaxResII = std::max(MaxResII, I->second / NumFUs);
  }
  DEBUG(dbgs() << "ResMII: " << MaxResII << '\n');
  return MaxResII;
}

bool SchedulingBase::computeMII() {
  unsigned RecMII = computeRecMII();
  if (RecMII == 0) {
    MII = this->getCriticalPathLength();
    return false;
  }

  unsigned ResMII = computeResMII();
  MII = std::max(RecMII, ResMII);
  // Also adjust the critical path length.
  setCriticalPathLength(std::max(MII, getCriticalPathLength()));
  return true;
}

void SchedulingBase::dumpTimeFrame() const {
  printTimeFrame(dbgs());
}

unsigned SchedulingBase::getPredicateChannel(MachineInstr *MI) {
  //MachineOperand *P = VInstrInfo::getPredOperand(MI);
  //unsigned PredReg = P->getReg();
  //if (*P).isPredicateInverted()) PredReg = ~PredReg;
  //return PredReg;
  return 0;
}

void SchedulingBase::takeFU(MachineInstr *MI, unsigned step, unsigned Latency,
                            FuncUnitId FU) {
  unsigned PredReg = getPredicateChannel(MI);
  VFUs::FUTypes Ty = FU.getFUType();

  for (unsigned i = step, e = step + Latency; i != e; ++i) {
    unsigned s = computeStepKey(i);
    SparseBitVector<> &V = getRTFor(PredReg, FU);
    assert(!V.test(s - StartSlot) && "FU already in use!");
    V.set(s - StartSlot);
    // Also take the un-predicated channel.
    //if (PredReg) getRTFor(PredicatedChannel, FU).set(s - StartSlot);

    // FIXME: Provide method "hasPipelineStage" and method "isVariableLatency".
    if (Ty == VFUs::BRam)
      ++PipelineStageStatus[computeStepKey(i + 1)];
    else if (Ty == VFUs::MemoryBus || Ty == VFUs::CalleeFN)
      ++PipelineBreakerStatus[s];
  }
}

void SchedulingBase::takeFU(VSUnit *U, unsigned step) {
  MachineInstr *MI = U->getRepresentativeInst();
  FuncUnitId FU = VInstrInfo::getPreboundFUId(MI);
  if (FU.isTrivial()) return;

  takeFU(MI, step, U->getLatency(), FU);

  // Take the FU of DstMux.
  //for (unsigned i = 1, e = U->num_instrs(); i < e; ++i) {
  //  MI = U->getInstrAt(i);
  //  FU = VInstrInfo::getPreboundFUId(MI);
  //  if (FU.getFUType() != VFUs::Mux) continue;

  //  takeFU(MI, step + U->getLatencyAt(i), 1, FU);
  //}
}

bool SchedulingBase::hasSpareFU(MachineInstr *MI,unsigned step,unsigned Latency,
                                FuncUnitId FU) {
  unsigned PredReg = getPredicateChannel(MI);
  VFUs::FUTypes Ty = FU.getFUType();
  // Do all resource at step been reserve?
  for (unsigned i = step, e = step + Latency; i != e; ++i) {
    unsigned s = computeStepKey(i);
    SparseBitVector<> &V = getRTFor(PredReg, FU);
    if (V.test(s - StartSlot))
      return false;
    //// Do not conflict with the predicated channel as well.
    //if (PredReg == 0 && getRTFor(PredicatedChannel, FU).test(s - StartSlot))
    //  return false;
    //// Do not conflict with the un-predicated channel as well.
    //if (PredReg && getRTFor(0, FU).test(s - StartSlot))
    //  return false;

    // FIXME: Provide method "hasPipelineStage" and method "isVariableLatency".
    if (Ty == VFUs::BRam) {
      // Dont schedule Pipeline FU here, if the pipeline cannot flush before
      // any pipeline breaker.
      // FIXME: Should test the the status from start interval + 1 to latency.
      if (PipelineBreakerStatus.lookup(computeStepKey(i + 1)))
        return false;
    } else if (Ty == VFUs::MemoryBus || Ty == VFUs::CalleeFN)
      // We need to flush the pipeline before issue these operations.
      if (PipelineStageStatus.lookup(s))
        return false;
  }

  return true;
}

bool SchedulingBase::hasSpareFU(VSUnit *U, unsigned step) {
  MachineInstr *MI = U->getRepresentativeInst();
  FuncUnitId FU = VInstrInfo::getPreboundFUId(MI);
  if (FU.isTrivial()) return true;

  if (!hasSpareFU(MI, step, U->getLatency(), FU))
    return false;

  // Take the FU of DstMux.
  //for (unsigned i = 1, e = U->num_instrs(); i < e; ++i) {
  //  MI = U->getInstrAt(i);
  //  FU = VInstrInfo::getPreboundFUId(MI);
  //  if (FU.getFUType() != VFUs::Mux) continue;

  //  if (!hasSpareFU(MI, step + U->getLatencyAt(i), 1, FU))
  //    return false;
  //}

  return true;
}

bool SchedulingBase::tryTakeResAtStep(VSUnit *U, unsigned step) {
  if (!hasSpareFU(U, step)) return false;

  takeFU(U, step);

  return true;
}

void SchedulingBase::scheduleSU(VSUnit *U, unsigned step) {
  U->scheduledTo(step);

  takeFU(U, step);
}

void SchedulingBase::revertFUUsage(MachineInstr *MI, unsigned step,
                                   unsigned Latency, FuncUnitId FU) {
  unsigned PredReg = getPredicateChannel(MI);
  VFUs::FUTypes Ty = FU.getFUType();

  for (unsigned i = step, e = step + Latency; i != e; ++i) {
    unsigned s = computeStepKey(i);
    getRTFor(PredReg, FU).reset(s - StartSlot);
    // Also take the un-predicated channel.
    if (PredReg) getRTFor(PredicatedChannel, FU).reset(s - StartSlot);

    if (Ty == VFUs::BRam) {
      unsigned &Status = PipelineStageStatus[computeStepKey(i + 1)];
      assert(Status && "Pipeline stage not used!");
      --Status;
    } else if (Ty == VFUs::MemoryBus || Ty == VFUs::CalleeFN) {
      unsigned &Status = PipelineBreakerStatus[s];
      assert(Status && "Pipeline breaker not used!");
      --Status;
    }
  }
}

void SchedulingBase::revertFUUsage(VSUnit *U, unsigned step) {
  FuncUnitId FU = U->getFUId();
  // We will always have enough trivial resources.
  if (FU.isTrivial()) return;

  revertFUUsage(U->getRepresentativeInst(), step, U->getLatency(), FU);
  // Revert the Usage of DstMux.
  //for (unsigned i = 1, e = U->num_instrs(); i < e; ++i) {
  //  MachineInstr *MI = U->getInstrAt(i);
  //  FU = VInstrInfo::getPreboundFUId(MI);
  //  if (FU.getFUType() != VFUs::Mux) continue;

  //  revertFUUsage(MI, step + U->getLatencyAt(i), 1, FU);
  //}
}

void SchedulingBase::unscheduleSU(VSUnit *U) {
  unsigned step = U->getSlot();
  U->resetSchedule();
  revertFUUsage(U, step);
}

void SchedulingBase::verifyFUUsage() {
  resetRT();

  typedef VSchedGraph::sched_iterator it;
  for (it I = State.sched_begin(), E = State.sched_end(); I != E; ++I) {
    VSUnit *A = *I;
    FuncUnitId FU = A->getFUId();
    // We only try to balance the post bind resource.
    // if (A->getFUId().isBinded()) continue;
    // Ignore the DG for trivial resources.
    if (!FU.isBound()) continue;

    // Check ifwe have enough function unit by try to fit them in the resource
    // table including ALAPStep.
    for (unsigned i = getASAPStep(A), e = getALAPStep(A) + 1; i != e; ++i) {
      if (!tryTakeResAtStep(A, i)) {
        llvm_unreachable("Resource conflict detected!");
        break;
      }
    }
  }
}

unsigned SchedulingBase::computeStepKey(unsigned step) const {
  if (MII != 0)
    step = StartSlot + (step - StartSlot) % MII;

  return step;
}

unsigned SchedulingBase::buildFDepHD(bool rstSTF) {
  if (rstSTF) State.resetSchedule(getMII());

  buildTimeFrame();

  return CriticalPathEnd;
}

//===----------------------------------------------------------------------===//
void SchedulingBase::schedulePassiveSUnits() {
  typedef VSchedGraph::sched_iterator it;
  for (it I = State.sched_begin(), E = State.sched_end(); I != E; ++I) {
    VSUnit *A = *I;
    if (A->isScheduled())
      continue;

    assert(A->getFUId().isTrivial()
      && "SUnit that taking non-trivial not scheduled?");

    DEBUG(A->print(dbgs()));
    unsigned step = getASAPStep(A);
    A->scheduledTo(step);
    buildFDepHD(false);
  }
}

bool SchedulingBase::allNodesSchedued() const {
  typedef VSchedGraph::sched_iterator it;
  for (it I = State.sched_begin(), E = State.sched_end(); I != E; ++I) {
    VSUnit *A = *I;
    if (!A->isScheduled()) return false;
  }

  return true;
}

bool SchedulingBase::scheduleCriticalPath(bool refreshFDepHD) {
  if (refreshFDepHD) buildFDepHD(true);

  typedef VSchedGraph::sched_iterator it;
  for (it I = State.sched_begin(), E = State.sched_end(); I != E; ++I) {
    VSUnit *A = *I;

    if (A->isScheduled() || getTimeFrame(A) != 1)
      continue;

    unsigned step = getASAPStep(A);
    if (!tryTakeResAtStep(A, step)) return false;
    DEBUG(A->print(dbgs()));
    DEBUG(dbgs() << " asap step: " << step << " in critical path.\n");
    A->scheduledTo(step);
  }

  return true;
}

void SchedulingBase::viewGraph() {
  ViewGraph(this, State.getMachineBasicBlock()->getName());
}
