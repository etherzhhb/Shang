//===---------- IPLScheduling.cpp - IPL Scheduler ---------------*- C++ -*-===//
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
// This file implement the Scheduler with Integer Linear Programming approach.
// Please reference paper:
// Hwang, C.-T.; Lee, J.-H.; Hsu, Y.-C.; ,
// "A formal approach to the scheduling problem in high level synthesis ,
// " Computer-Aided Design of Integrated Circuits and Systems,
//   IEEE Transactions on , vol.10, no.4, pp.464-475, Apr 1991
// for more detail.
//
//===----------------------------------------------------------------------===//

#include "SchedulingBase.h"

#include "llvm/ADT/DepthFirstIterator.h"
#include "llvm/Support/ErrorHandling.h"
#define DEBUG_TYPE "vbe-ilps"
#include "llvm/Support/Debug.h"

#include "lp_solve/lp_lib.h"

using namespace llvm;

// Helper function
static const char *transSolveResult(int result) {
  if (result == -2) return "NOMEMORY";
  else if (result > 13) return "Unkwon result!";

  static const char *ResultTable[] = {
    "OPTIMAL",
    "SUBOPTIMAL",
    "INFEASIBLE",
    "UNBOUNDED",
    "DEGENERATE",
    "NUMFAILURE",
    "USERABORT",
    "TIMEOUT",
    "PRESOLVED",
    "PROCFAIL",
    "PROCBREAK",
    "FEASFOUND",
    "NOFEASFOUND"
  };

  return ResultTable[result];
}

ILPScheduler::ILPScheduler(VSchedGraph &S) 
  : SchedulingBase(S), SUnitToSV(S.getNumSUnits()), NumStepVars(0) {
}

unsigned ILPScheduler::buildSVIdx() {
  unsigned totalSV = 0;
  for (VSchedGraph::const_iterator I = State.begin(), E = State.end();
       I != E; ++I) {
    VSUnit *U = *I;
    SUnitToSV[U->getIdx()] = totalSV;
    totalSV += getTimeFrame(U);    
  }

  return totalSV;
}

void ILPScheduler::setUpVariables(lprec *lp) {
  ActiveSUs.clear();

  // Set up the step variables.
  for (VSchedGraph::const_iterator I = State.begin(), E = State.end();
       I != E; ++I) {
    const VSUnit *U = *I;

    // Get the index of first step variable of U.
    unsigned FstIdx = getFstSVIdxOf(U),
    // And the total step variable count of U.
             NumSVs = getTimeFrame(U),
             ASAP = getASAPStep(U);

    for (unsigned i = 0, e = NumSVs; i < e; ++i) {
      unsigned CurStep = i + ASAP, CurIdx = i + FstIdx + 1;
#ifndef NDEBUG
      // Set the name of the step variable.
      std::string SVName = "sv" + utostr_32(U->getIdx()) + "_"
                           + utostr_32(CurStep);
      if (!set_col_name(lp, CurIdx, const_cast<char*>(SVName.c_str())))
        report_fatal_error("ILPScheduler: Can NOT set column name for step"
                           " variable: " + SVName);
#endif
      if (!set_binary(lp, CurIdx, TRUE))
        report_fatal_error("ILPScheduler: Can NOT set step variable: "
                           + SVName + " as binary variable!");

      // Build the active map.
      ActiveSUs[CurStep].push_back(U);
    }
  }

  // Set up the function unit variables.
  for (VFUs::FUTypes fu = VFUs::Shift; fu <= VFUs::LastCommonFUType;
       fu = (VFUs::FUTypes)(fu + 1u)) {
    char *Name = const_cast<char*>(VFUDesc::getTypeName(fu));
#ifndef NDEBUG
    if (!set_col_name(lp, getIdxOf(fu) + 1, Name))
      report_fatal_error("ILPScheduler: Can NOT set column name for FU"
                         " variable: " + std::string(Name));
#endif
    if (!set_int(lp, getIdxOf(fu) + 1, true))
      report_fatal_error("ILPScheduler: Can NOT set integer for FU variable: "
                         + std::string(Name));
  }

  // Set up step variable.
#ifndef NDEBUG
  if (!set_col_name(lp, StepIdx + 1, "TotalStep"))
    report_fatal_error("ILPScheduler: Can NOT set column name for Step"
                        " variable!");
#endif
  if (!set_int(lp, StepIdx + 1, TRUE))
    report_fatal_error("ILPScheduler: Can NOT set integer for Step variable!");

  if (!set_bounds(lp, StepIdx + 1, 0.0, getALAPStep(State.getExitRoot())))
    report_fatal_error("ILPScheduler: Can NOT add bounds for total step!");
}

void ILPScheduler::buildOneActiveStepConstraints(lprec *lp) {
  SmallVector<REAL, 64> Row;
  SmallVector<int, 64> ColIdx;

  for (VSchedGraph::const_iterator I = State.begin(), E = State.end();
       I != E; ++I) {
    const VSUnit *U = *I;

    // Get the index of first step variable of U.
    unsigned FstIdx = getFstSVIdxOf(U);
    // And the total step variable count of U.
    unsigned NumSVs = getTimeFrame(U);

    // Resize the row and corresponding indexes
    Row.reserve(NumSVs);
    Row.set_size(NumSVs);
    ColIdx.reserve(NumSVs);
    ColIdx.set_size(NumSVs);

    for (unsigned i = 0; i < NumSVs; ++i) {
      Row[i] = 1.0;
      // The column index is started from 1 in lp_slove.
      ColIdx[i] = FstIdx + i + 1;
    }

    // Add the constraints to the model.
    if (!add_constraintex(lp, NumSVs, Row.data(), ColIdx.data(), EQ, 1.0))
      report_fatal_error("ILPScheduler: Can NOT add step variable constraints"
                         " of schedule unit " + utostr_32(U->getIdx()));
  }

  // Now we have the one step active constraints of the exit node.
  const VSUnit *Exit = State.getExitRoot();
  unsigned ASAP = getASAPStep(Exit);

  // Get the index of first step variable of U.
  assert(ColIdx[0] == getFstSVIdxOf(Exit) + 1 && "What is in the vectors then?");
  // And the total step variable count of U.
  unsigned NumSVs = getTimeFrame(Exit);

  // Translate the step variable to control step value.
  for (unsigned i = 0; i < NumSVs; ++i)
    Row[i] = ASAP + i;

  // Set up the total step constraint LastStep <= TotalStep.
  Row.push_back(-1.0);
  ColIdx.push_back(StepIdx + 1);
  if (!add_constraintex(lp, Row.size(), Row.data(), ColIdx.data(), LE, 0.0))
    report_fatal_error("ILPScheduler: Can NOT add total step constraints!");
}

void ILPScheduler::buildPrecedenceConstraints(lprec *lp) {
  SmallVector<REAL, 128> Row;
  SmallVector<int, 128> ColIdx;

  for (VSchedGraph::const_iterator I = State.begin(), E = State.end();
       I != E; ++I) {
    const VSUnit *SrcU = *I;

    // Get the index of first step variable of U.
    unsigned SrcFstIdx = getFstSVIdxOf(SrcU),
    // And the total step variable count of U.
             SrcNumSVs = getTimeFrame(SrcU),
             SrcASAP = getASAPStep(SrcU);

    // Resize the row and corresponding indexes
    Row.reserve(SrcNumSVs);
    Row.set_size(SrcNumSVs);
    ColIdx.reserve(SrcNumSVs);
    ColIdx.set_size(SrcNumSVs);

    // Build the constraint for Dst_SU_Step - Src_SU_Step >= Src_Latency 
    // NOTE: add_constraintex will sort the row according column indexes!
    // Construct the step of Src schedule unit.
    for (unsigned i = 0; i < SrcNumSVs; ++i) {
      Row[i] = -(REAL)(SrcASAP + i);
      // The column index is started from 1 in lp_slove.
      ColIdx[i] = SrcFstIdx + i + 1;
    }

    for (VSUnit::const_use_iterator DI = SrcU->use_begin(),
         DE = SrcU->use_end(); DI != DE;++DI) {
      const VSUnit *DstU = *DI;
      VDEdge *UseEdge = DstU->getEdgeFrom(SrcU);

      // FIXME: Also consider back-edge for Modulo Scheduling.
      if (UseEdge->isBackEdge()) continue;

      unsigned DstFstIdx = getFstSVIdxOf(DstU),
               DstNumSVs = getTimeFrame(DstU),
               DstASAP = getASAPStep(DstU);

      // Construct the step of Dst schedule unit.
      for (unsigned i = 0; i < DstNumSVs; ++i) {
        Row.push_back(DstASAP + i);
        ColIdx.push_back(DstFstIdx + i + 1);
      }

      // Add the constraints to the model.
      if (!add_constraintex(lp, ColIdx.size(), Row.data(), ColIdx.data(),
                            GE, SrcU->getLatency()))
        report_fatal_error("ILPScheduler: Can NOT add Precedence constraints"
        " of schedule unit " + utostr_32(DstU->getIdx()) +
        " to " + utostr_32(SrcU->getIdx()));

      // Resize the row and corresponding indexes
      Row.reserve(SrcNumSVs);
      Row.set_size(SrcNumSVs);
      ColIdx.reserve(SrcNumSVs);
      ColIdx.set_size(SrcNumSVs);
    }
  }
}

void ILPScheduler::buildFUConstraints(lprec *lp) {
  VSUnit *EntryU = State.getEntryRoot(), *ExitU = State.getExitRoot();

  // Remember the bound schedule unit and corresponding function unit.
  typedef SmallVector<const VSUnit*, 16> BoundSUVec;
  typedef std::map<FuncUnitId, BoundSUVec> FU2SUMap;
  FU2SUMap FSMap;

  // Numbers of Unbound function units.
  unsigned zero = 0;
  SmallVector<unsigned, 4> MaxFUCounts(3, zero), CurFUCounts;

  SmallVector<REAL, 128> Row;
  SmallVector<int, 128> ColIdx;

  for (unsigned i = getASAPStep(EntryU), e = getALAPStep(ExitU); i != e; ++i){
    ActiveSUVec &SUs = ActiveSUs[i];
    // Clear the function unit counts.
    CurFUCounts.assign(3, 0);

    // Classify the active schedule units by its function unit id. 
    for (ActiveSUVec::const_iterator I = SUs.begin(), E = SUs.end();
         I != E; ++I) {
      const VSUnit *U = *I;

      if (U->getFUType() > VFUs::LastCommonFUType
          || U->getFUType() == VFUs::Trivial)
        continue;

      // Count the unbound function units in current step.
      if (!U->getFUId().isBound())
        ++CurFUCounts[U->getFUType() - VFUs::Shift];
      
      FSMap[U->getFUId()].push_back(U);
    }

    // Update the max function unit count.
    MaxFUCounts[0] = /*std::*/max(MaxFUCounts[0], CurFUCounts[0]);
    MaxFUCounts[1] = /*std::*/max(MaxFUCounts[1], CurFUCounts[1]);
    MaxFUCounts[2] = /*std::*/max(MaxFUCounts[2], CurFUCounts[2]);
    
    // Build the constraint for function units.
    for (FU2SUMap::const_iterator I = FSMap.begin(), E = FSMap.end();
         I != E; ++I) {
      FuncUnitId Id = I->first;
      for (BoundSUVec::const_iterator UI = I->second.begin(), 
           UE = I->second.end(); UI != UE; ++UI) {
        const VSUnit *U = *UI;

        Row.push_back(1.0);
        ColIdx.push_back(i - getASAPStep(U) + getFstSVIdxOf(U) + 1);
      }

      if (Id.isBound()) {
        if (!add_constraintex(lp, ColIdx.size(), Row.data(), ColIdx.data(),
                              LE, 1))
          report_fatal_error("ILPScheduler: Can NOT add FU constraints"
                             " at step " + utostr_32(i) +
                             " for FU "
                             + std::string(VFUDesc::getTypeName(Id.getFUType())));
      } else {
        Row.push_back(-1.0);
        ColIdx.push_back(getIdxOf(Id.getFUType()) + 1);
        if (!add_constraintex(lp, ColIdx.size(), Row.data(), ColIdx.data(),
                              LE, 0))
          report_fatal_error("ILPScheduler: Can NOT add FU constraints"
                             " at step " + utostr_32(i) +
                             " for FU "
                             + std::string(VFUDesc::getTypeName(Id.getFUType())));
      }
      
      Row.clear();
      ColIdx.clear();
    }

    // Set up the bounds for function unit count variable.
    for (VFUs::FUTypes fu = VFUs::Shift; fu <= VFUs::LastCommonFUType;
         fu = (VFUs::FUTypes)(fu + 1u))
      if (!set_bounds(lp, getIdxOf(fu) + 1, 0.0, MaxFUCounts[fu - VFUs::Shift]))
          report_fatal_error("ILPScheduler: Can NOT add FU bounds for FU "
                             + std::string(VFUDesc::getTypeName(fu)));


    FSMap.clear();
  }
}

void ILPScheduler::buildObject(lprec *lp) {
  SmallVector<REAL, 128> Row;
  SmallVector<int, 128> ColIdx;

  for (VFUs::FUTypes fu = VFUs::Shift; fu <= VFUs::LastCommonFUType;
       fu = (VFUs::FUTypes)(fu + 1u)) {
    // FIXME: Push back the cost factor for function unit and cost factor for
    // area optimization here.
    Row.push_back(1.0);
    ColIdx.push_back(getIdxOf(fu) + 1);
  }

  // FIXME: Push back the cost factor for total step here.
  Row.push_back(1.0);
  ColIdx.push_back(StepIdx + 1);

  if (!set_obj_fnex(lp, Row.size(), Row.data(), ColIdx.data()))
    report_fatal_error("ILPScheduler: Can NOT set objective!");
 
  set_minim(lp);
}

void ILPScheduler::buildSchedule(lprec *lp) {
  DEBUG(dbgs() << "ILPScheduler: Cost = " << get_objective(lp) << '\n');

   for (VSchedGraph::iterator I = State.begin(), E = State.end();
       I != E; ++I) {
    VSUnit *U = *I;

    // Get the index of first step variable of U.
    unsigned FstIdx = getFstSVIdxOf(U),
    // And the total step variable count of U.
             NumSVs = getTimeFrame(U),
             ASAP = getASAPStep(U);

    for (unsigned i = 0; i < NumSVs; ++i) {
      DEBUG(dbgs() << get_origcol_name(lp, FstIdx + i + 1) << " : "
                   << get_var_primalresult(lp, FstIdx + i + 1 + TotalRows)
                   << '\n');

      if (get_var_primalresult(lp, FstIdx + i + 1 + TotalRows) == 0.0) continue;

      U->scheduledTo(ASAP + i);
      // Schedule done, move to next schedule unit.
      break;
    }
    assert(U->isScheduled() && "ILP result broken!");
   }
}

bool ILPScheduler::scheduleState() {
  buildFDepHD(true);
  
  // Ensure there is no resource conflict in critical path.
  // FIXME: We can consider resource conflict in build FDepHD.
  if (!scheduleCriticalPath(false))
    return false;

  NumStepVars = buildSVIdx();

  // Set up the variable indexes.
  StepIdx = getIdxOf(VFUs::LastCommonFUType) + 1;

  TotalVariables = StepIdx + 1;

  lprec *lp = make_lp(0,TotalVariables);

  set_add_rowmode(lp, TRUE);

  setUpVariables(lp);

  buildOneActiveStepConstraints(lp);
  buildPrecedenceConstraints(lp);
  buildFUConstraints(lp);
  // Turn off the add rowmode and start to solve the model.
  set_add_rowmode(lp, FALSE);

  buildObject(lp);

  DEBUG(set_verbose(lp, FULL));

  set_presolve(lp, PRESOLVE_ROWS | PRESOLVE_COLS | PRESOLVE_LINDEP,
               get_presolveloops(lp));

  DEBUG(write_lp(lp, "log.lp"));

  TotalRows = get_Nrows(lp);

  int result = solve(lp);

  DEBUG(dbgs() << "ILP result is: "<< transSolveResult(result) << "\n");
  switch (result) {
  case SUBOPTIMAL:
    DEBUG(dbgs() << "Note: suboptimal schedule found!\n");
  case OPTIMAL:
  case PRESOLVED:
    break;
  default:
    report_fatal_error("ILPScheduler: Schedule fail!");
  }
  
  // Finally, Schedule the state with the ILP result.
  buildSchedule(lp);

  delete_lp(lp);

  return true;
}
