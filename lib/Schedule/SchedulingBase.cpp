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
//===----------------------------------------------------------------------===//
template<bool IsCtrlPath>
void Scheduler<IsCtrlPath>::resetTimeFrame() {
  // Reset the time frames
  for (TFMapTy::iterator I = SUnitToTF.begin(), E = SUnitToTF.end();I != E;++I)
    I->second = std::make_pair(0, VSUnit::MaxSlot);
}

template<bool IsCtrlPath>
void Scheduler<IsCtrlPath>::buildTimeFrame() {
  VSUnit *EntryRoot = G.getEntryRoot();
  assert(EntryRoot->isScheduled() && "Entry must be scheduled first!");

  resetTimeFrame();
  // Build the time frames
  bool HasNegativeCycle = buildASAPStep();
  assert(!HasNegativeCycle && "Unexpected negative cycle!");
  buildALAPStep();

  DEBUG(dumpTimeFrame());
}

template<bool IsCtrlPath>
unsigned Scheduler<IsCtrlPath>::calculateASAP(const VSUnit * A) {
  unsigned NewStep = 0;
  for (const_dep_it DI = dep_begin(A), DE = dep_end(A); DI != DE; ++DI) {
    const VSUnit *Dep = *DI;
    // Ignore the back-edges when we are not pipelining the BB.
    if (DI.isLoopCarried() && !MII) continue;

    unsigned DepASAP = Dep->isScheduled() ? Dep->getSlot() : getASAPStep(Dep);
    int Step = DepASAP + DI.getLatency(MII);
    DEBUG(dbgs() << "From ";
          if (DI.isLoopCarried()) dbgs() << "BackEdge ";
          Dep->print(dbgs());
          dbgs() << " Step " << Step << '\n');
    unsigned UStep = std::max(0, Step);
    NewStep = std::max(UStep, NewStep);
  }

  return NewStep;
}

template<bool IsCtrlPath>
bool Scheduler<IsCtrlPath>::buildASAPStep() {
  bool NeedToReCalc = true;
  unsigned NumCalcTimes = 0;
  const unsigned GraphSize = G.size<IsCtrlPath>();

  // Build the time frame iteratively.
  while(NeedToReCalc) {
    NeedToReCalc = false;
    for (iterator I = begin(), E = end(); I != E; ++I) {
      const VSUnit *A = *I;
      if (A->isScheduled()) {
        SUnitToTF[A].first = A->getSlot();
        continue;
      }

      DEBUG(dbgs() << "\n\nCalculating ASAP step for \n";
            A->dump(););

      unsigned NewStep = calculateASAP(A);

      DEBUG(dbgs() << "Update ASAP step to: " << NewStep << " for \n";
            A->dump();
            dbgs() << "\n\n";);

      unsigned &ASAPStep = SUnitToTF[A].first;
      if (ASAPStep == NewStep) continue;
      ASAPStep = NewStep;

      if (NeedToReCalc) continue;

      // We need to re-calculate the ASAP steps if the sink of the back-edges,
      // need to be update.
      for (const_use_it UI = use_begin(A), UE = use_end(A); UI != UE; ++UI) {
        const VSUnit *Use = *UI;
        NeedToReCalc |= Use->getIdx() < A->getIdx()
                        && calculateASAP(Use) != getASAPStep(Use);
      }
    }

    if (NeedToReCalc) {
      ++NumCalcTimes;
      // Only iterating |V|-1 times, otherwise there is negative cycle.
      if (NumCalcTimes >= GraphSize) return true;
    }
  }

  VSUnit *Exit = G.getExitRoot();
  unsigned ExitASAP = IsCtrlPath ? getASAPStep(Exit) : Exit->getSlot();
  CriticalPathEnd = std::max(CriticalPathEnd, ExitASAP);
  assert((IsCtrlPath || ExitASAP == CriticalPathEnd)
         && "Bad time frame in data-path dependencies graph!");
  return false;
}

template<bool IsCtrlPath>
unsigned Scheduler<IsCtrlPath>::computeRecMII(unsigned MinRecMII) {
  unsigned CriticalPathLength = getCriticalPathLength();
  unsigned MaxRecMII = CriticalPathLength;
  unsigned RecMII = 0;

  MinRecMII = std::max(1u, MinRecMII);
  G.resetSchedule<IsCtrlPath>();

  // Find the RecMII by binary search algorithm.
  while (MinRecMII <= MaxRecMII) {
    unsigned MidRecMII = MinRecMII + (MaxRecMII - MinRecMII) / 2;

    setMII(MidRecMII);
    resetTimeFrame();

    if (!buildASAPStep()) {
      RecMII = MidRecMII;
      // There is no negative cycles, try to further reduce the MII.
      MaxRecMII = MidRecMII - 1;
    } else
      // Else we need to relax the MII.
      MinRecMII = MidRecMII + 1;
  }

  assert(RecMII && "Negative cycle found even pipeline is disabled!");

  return RecMII;
}

template<bool IsCtrlPath>
unsigned Scheduler<IsCtrlPath>::calculateALAP(const VSUnit *A) {
  unsigned NewStep = VSUnit::MaxSlot;
  for (const_use_it UI = use_begin(A), UE = use_end(A); UI != UE; ++UI) {
    const VSUnit *Use = *UI;
    VDEdge UseEdge = Use->getEdgeFrom<IsCtrlPath>(A, MII);

    // Ignore the back-edges when we are not pipelining the BB.
    if (UseEdge.isLoopCarried() && !MII) continue;

    unsigned UseALAP = Use->isScheduled() ?
                       Use->getSlot() : getALAPStep(Use);
    if (UseALAP == 0) {
      assert(UseEdge.isLoopCarried() && "Broken time frame!");
      UseALAP = VSUnit::MaxSlot;
    }

    unsigned Step = UseALAP - UseEdge.getLatency(MII);
    DEBUG(dbgs() << "From ";
          if (UseEdge.isLoopCarried()) dbgs() << "BackEdge ";
          Use->print(dbgs());
          dbgs() << " Step " << Step << '\n');
    NewStep = std::min(Step, NewStep);
  }

  return NewStep;
}

template<bool IsCtrlPath>
void Scheduler<IsCtrlPath>::buildALAPStep() {
  const VSUnit *Exit = G.getExitRoot();
  int LastSlot = CriticalPathEnd;
  SUnitToTF[Exit].second = LastSlot;

  bool NeedToReCalc = true;
  // Build the time frame iteratively.
  while (NeedToReCalc) {
    NeedToReCalc = false;
    for (int Idx = G.size<IsCtrlPath>() - 1; Idx >= 0; --Idx){
      const VSUnit *A = begin()[Idx];
      if (A == Exit) continue;
      
      if (A->isScheduled()) {
        SUnitToTF[A].second = A->getSlot();
        continue;
      }

      DEBUG(dbgs() << "\n\nCalculating ALAP step for \n";
            A->dump(););

      unsigned NewStep = calculateALAP(A);

      DEBUG(dbgs() << "Update ALAP step to: " << NewStep << " for \n";
            A->dump();
            dbgs() << "\n\n";);

      unsigned &ALAPStep = SUnitToTF[A].second;
      if (ALAPStep == NewStep) continue;
      assert(getASAPStep(A) <= NewStep && "Broken ALAP step!");
      ALAPStep = NewStep;

      for (const_dep_it DI = dep_begin(A), DE = dep_end(A); DI != DE; ++DI) {
        const VSUnit *Dep = *DI;
        NeedToReCalc |= A->getIdx() < Dep->getIdx()
                        && calculateALAP(Dep) != getALAPStep(Dep);
      }
    }
  }
}

template<bool IsCtrlPath>
void Scheduler<IsCtrlPath>::printTimeFrame(raw_ostream &OS) const {
  OS << "Time frame:\n";
  for (iterator I = begin(), E = end(); I != E; ++I) {
    const VSUnit *A = *I;
    A->print(OS);
    OS << " : {" << getASAPStep(A) << "," << getALAPStep(A)
      << "} " <<  getTimeFrame(A);

    for (const_dep_it DI = dep_begin(A), DE = dep_end(A); DI != DE; ++DI)
      OS << " [" << DI->getIdx() << "]"; 
    
    OS << '\n';
  }
}

template<bool IsCtrlPath>
void Scheduler<IsCtrlPath>::dumpTimeFrame() const {
  printTimeFrame(dbgs());
}

template<bool IsCtrlPath>
unsigned Scheduler<IsCtrlPath>::buildTimeFrameAndResetSchedule(bool reset) {
  if (reset) G.resetSchedule<IsCtrlPath>();

  buildTimeFrame();

  return CriticalPathEnd;
}

template class Scheduler<true>;
template class Scheduler<false>;

unsigned SchedulingBase::computeResMII() {
  // FIXME: Compute the resource area cost
  std::map<FuncUnitId, unsigned> TotalResUsage;
  for (iterator I = cp_begin(&G), E = cp_end(&G); I != E; ++I) {
    const VSUnit *SU = *I;
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

SchedulingBase::InstSetTy::const_iterator
SchedulingBase::findConflictedInst(const InstSetTy &Set, const MachineInstr *MI) {
  typedef InstSetTy::const_iterator it;
  it I = Set.begin(), E = Set.end();
  const MachineBasicBlock *MIParent = MI->getParent();

  while (I != E) {
    if (MIParent == (*I)->getParent() && !VInstrInfo::isPredicateMutex(MI, *I))
      return I;

    ++I;
  }

  return I;
}

void SchedulingBase::takeFU(const MachineInstr *MI, unsigned step,
                            unsigned Latency, FuncUnitId FU) {
  VFUs::FUTypes Ty = FU.getFUType();

  for (unsigned i = step, e = step + Latency; i != e; ++i) {
    unsigned s = computeStepKey(i);
    InstSetTy &InstSet = getRTFor(s, FU);
    assert(!hasConflictedInst(InstSet, MI) && "FU conflict detected!");
    InstSet.push_back(MI);

    // FIXME: Provide method "hasPipelineStage" and method "isVariableLatency".
    if (Ty == VFUs::BRam) {
      InstSetTy &InstSet = PipeFUs[s];
      assert(std::find(InstSet.begin(), InstSet.end(), MI) == InstSet.end()
             && "FU conflict detected!");
      InstSet.push_back(MI);
    } else if (Ty == VFUs::MemoryBus || Ty == VFUs::CalleeFN) {
      InstSetTy &InstSet = PipeBreakerFUs[s];
      assert(std::find(InstSet.begin(), InstSet.end(), MI) == InstSet.end()
             && "FU conflict detected!");
      InstSet.push_back(MI);
    }
  }
}

void SchedulingBase::takeFU(const VSUnit *U, unsigned step) {
  MachineInstr *MI = U->getRepresentativePtr();
  if (MI == 0) return;

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

const MachineInstr *
SchedulingBase::getConflictedInst(const MachineInstr *MI,unsigned step,
                                  unsigned Latency, FuncUnitId FU) {
  VFUs::FUTypes Ty = FU.getFUType();
  // Do all resource at step been reserve?
  for (unsigned i = step, e = step + Latency; i != e; ++i) {
    unsigned CurSlot = computeStepKey(i);
    InstSetTy &InstSet = getRTFor(CurSlot, FU);
    if (const MachineInstr *OtherMI = getConflictedInst(InstSet, MI))
      return OtherMI;
    //// Do not conflict with the predicated channel as well.
    //if (PredReg == 0 && getRTFor(PredicatedChannel, FU).test(s - StartSlot))
    //  return false;
    //// Do not conflict with the un-predicated channel as well.
    //if (PredReg && getRTFor(0, FU).test(s - StartSlot))
    //  return false;
    // Pipeline conflict:
    // Pipeline FU | Pipeline Breaker
    // Op0
    // Op1         | Op2
    // ...Pipeline Stall...
    // Read Op0
    // Read Op1
    // Because the pipeline breaker, i.e. op2 has a variable latency,
    // We may read the result of Op1 when we suppose read the result Op0 in the
    // above example.
    // FIXME: Provide method "hasPipelineStage" and method "isVariableLatency",
    if (Ty == VFUs::BRam) {
      unsigned NextSlot = computeStepKey(i + 1);
      const InstSetTy &NextBreakers = PipeBreakerFUs.lookup(NextSlot);
      if (const MachineInstr *BreakerMI = getConflictedInst(NextBreakers, MI)) {
        // Cannot support II = 1
        if (NextSlot == CurSlot) return BreakerMI;

        InstSetTy &NextPipe = getRTFor(NextSlot, FU);
        if (const MachineInstr *OtherMI = getConflictedInst(NextPipe, MI))
          return OtherMI;
      }

      if (hasConflictedInst(PipeBreakerFUs.lookup(CurSlot), MI)) {
        unsigned PrevSlot = computeStepKey(i - 1);
        // Already detected by previous code.
        //if (PrevSlot == CurSlot) return BreakerMI;
        InstSetTy &PrevPipes = getRTFor(PrevSlot, FU);
        if (const MachineInstr *OtherMI = getConflictedInst(PrevPipes, MI))
          return OtherMI;
      }
    } else if (Ty == VFUs::MemoryBus || Ty == VFUs::CalleeFN) {
      const InstSetTy &CurPipes = PipeFUs.lookup(CurSlot);
      if (const MachineInstr *PipeLineMI = getConflictedInst(CurPipes, MI)) {
        unsigned PrevSlot = computeStepKey(i - 1);
        FuncUnitId PipeFU = VInstrInfo::getPreboundFUId(PipeLineMI);
        if (PrevSlot == CurSlot // Cannot support II = 1
            || hasConflictedInst(getRTFor(PrevSlot, PipeFU), MI))
          return PipeLineMI;
      }
    }
  }

  return 0;
}

const MachineInstr *SchedulingBase::getConflictedInst(const VSUnit *U,
                                                      unsigned step) {
  MachineInstr *MI = U->getRepresentativePtr();
  if (MI == 0) return 0;

  FuncUnitId FU = VInstrInfo::getPreboundFUId(MI);
  if (FU.isTrivial()) return 0;

  return getConflictedInst(MI, step, U->getLatency(), FU);
}

bool SchedulingBase::tryTakeResAtStep(const VSUnit *U, unsigned step) {
  if (getConflictedInst(U, step)) return false;

  takeFU(U, step);

  return true;
}

void SchedulingBase::scheduleSU(VSUnit *U, unsigned step) {
  assert(getConflictedInst(U, step) == 0
         && "Cannot schedule VSUnit to the given step!");
  U->scheduledTo(step);

  takeFU(U, step);
}

void SchedulingBase::revertFUUsage(const MachineInstr *MI, unsigned step,
                                   unsigned Latency, FuncUnitId FU) {
  VFUs::FUTypes Ty = FU.getFUType();

  for (unsigned i = step, e = step + Latency; i != e; ++i) {
    unsigned s = computeStepKey(i);
    InstSetTy &InstSet = getRTFor(s, FU);
    InstSetTy::iterator at = std::find(InstSet.begin(), InstSet.end(), MI);
    assert(at != InstSet.end() && "MI not exist in FU table!");
    InstSet.erase(at);

    if (Ty == VFUs::BRam) {
      InstSetTy &PipeSet = PipeFUs[computeStepKey(i)];
      InstSetTy::iterator at = std::find(PipeSet.begin(), PipeSet.end(), MI);
      assert(at != PipeSet.end() && "MI not exist in FU table!");
      PipeSet.erase(at);
    } else if (Ty == VFUs::MemoryBus || Ty == VFUs::CalleeFN) {
      InstSetTy &BreakerSet = PipeBreakerFUs[computeStepKey(i)];
      InstSetTy::iterator at = std::find(BreakerSet.begin(), BreakerSet.end(), MI);
      assert(at != BreakerSet.end() && "MI not exist in FU table!");
      BreakerSet.erase(at);
    }
  }
}

void SchedulingBase::revertFUUsage(const VSUnit *U, unsigned step) {
  FuncUnitId FU = U->getFUId();
  // We will always have enough trivial resources.
  if (FU.isTrivial()) return;

  revertFUUsage(U->getRepresentativePtr(), step, U->getLatency(), FU);
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

void SchedulingBase::verifyFUUsage(iterator I, iterator E) {
  resetRT();

  while (I != E) {
    const VSUnit *A = *I++;
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
  if (MII != 0) {
    int offset = int(step - EntrySlot) % int(MII);
    // Wrap the offset if necessary.
    if (offset < 0) offset = MII + offset;

    step = EntrySlot + offset;
  }

  return step;
}

bool SchedulingBase::allNodesSchedued(const_iterator I, const_iterator E) const {
  while (I != E) {
    const VSUnit *A = *I++;

    if (!A->isScheduled()) return false;
  }

  return true;
}

bool SchedulingBase::scheduleCriticalPath(iterator I, iterator E) {
  while (I != E) {
    VSUnit *A = *I++;

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
