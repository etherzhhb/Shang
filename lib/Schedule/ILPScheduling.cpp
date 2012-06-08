//===---------- IPLScheduling.cpp - IPL Scheduler ---------------*- C++ -*-===//
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
#include "llvm/ADT/Twine.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/CommandLine.h"
#define DEBUG_TYPE "vbe-ilps"
#include "llvm/Support/Debug.h"

#include "lp_solve/lp_lib.h"

using namespace llvm;
static cl::opt<bool>
StopAtFirst("ilp-schedule-stop-at-first",
            cl::desc("vtm - Find the feasible but not optimal schedule"
                     " with ilp scheduler"),
            cl::Hidden, cl::init(false));

// Helper function
static const char *transSolveResult(int result) {
  if (result == -2) return "NOMEMORY";
  else if (result > 13) return "Unknown result!";

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
  : SchedulingBase(S), NumStepVars(0) {
}

unsigned ILPScheduler::buildSVIdx() {
  unsigned totalSV = 0;
  typedef VSchedGraph::sched_iterator it;
  for (it I = State.sched_begin(), E = State.sched_end(); I != E; ++I) {
    VSUnit *U = *I;
    SUnitToSV[U] = totalSV;
    totalSV += getTimeFrame(U);
  }

  return totalSV;
}

void ILPScheduler::setUpVariables(lprec *lp) {
  ActiveSUs.clear();

  // Set up the step variables.
  typedef VSchedGraph::sched_iterator it;
  for (it I = State.sched_begin(), E = State.sched_end(); I != E; ++I) {
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

      // Build the active map. Use the pair structure in order to get the VSUnits
      // and the index.
      // EP: If step 4 contains the VSUnit named sv12 and the sv12 will run
      //     two steps, then the index of sv12 in step 5 will be sv12_4
      //     instead of sv12_5.
      SUToIdx SUToIdxPair = std::make_pair(U,CurIdx);
      ActiveSUs[computeStepKey(CurStep)].push_back(SUToIdxPair);
      // If the SUnit actived at several cstep, add it to the active map at the
      // rest steps.
      for (unsigned i = CurStep + 1, e = CurStep + U->getLatency(); i < e; ++i)
		    ActiveSUs[computeStepKey(i)].push_back(SUToIdxPair);
    }
  }


  // Set up the function unit variables.
  for (VFUs::FUTypes fu = VFUs::FirstNonTrivialFUType;
       fu <= VFUs::LastPostBindFUType; fu = (VFUs::FUTypes)(fu + 1u)) {
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

  typedef VSchedGraph::sched_iterator it;
  for (it I = State.sched_begin(), E = State.sched_end(); I != E; ++I) {
    const VSUnit *U = *I;

    // Get the index of first step variable of U.
    unsigned FstIdx = getFstSVIdxOf(U);
    // And the total step variable count of U.
    unsigned NumSVs = getTimeFrame(U);

#ifdef LPSOLVE_SOS
    REAL ASAP = getASAPStep(U);
#endif

    // Resize the row and corresponding indexes
    Row.reserve(NumSVs);
    Row.set_size(NumSVs);
    ColIdx.reserve(NumSVs);
    ColIdx.set_size(NumSVs);

    for (unsigned i = 0; i < NumSVs; ++i) {
#ifdef LPSOLVE_SOS
      Row[i] = ASAP + i;
#else
      Row[i] = 1.0;
#endif
      // The column index is started from 1 in lp_slove.
      ColIdx[i] = FstIdx + i + 1;
    }

#ifdef LPSOLVE_SOS
    std::string SOSName = "sv" + utostr_32(U->getIdx());
    int SOSIdx = add_SOS(lp, const_cast<char*>(SOSName.c_str()), SOS1, 1,
                         ColIdx.size(), ColIdx.data(), Row.data());
    if (!SOSIdx)
#else
    // Only one step can be actived at a time.
    if (!add_constraintex(lp, NumSVs, Row.data(), ColIdx.data(), EQ, 1.0))
#endif
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
  SmallVector<REAL, 32> DstRow;
  SmallVector<int, 32> DstColIdx;
  SmallVector<REAL, 64> Row;
  SmallVector<int, 64> ColIdx;

  typedef VSchedGraph::sched_iterator it;
  for (it I = State.sched_begin(), E = State.sched_end(); I != E; ++I) {
    const VSUnit *DstU = *I;

    assert(DstU->isControl() && "Unexpected datapath in scheduler!");
    unsigned DstFstIdx = getFstSVIdxOf(DstU),
             DstNumSVs = getTimeFrame(DstU),
             DstASAP = getASAPStep(DstU);

    // Build the constraint for Dst_SU_Step - Src_SU_Step >= Src_Latency
    // NOTE: add_constraintex will sort the row according column indexes!
    // Construct the step of Dst schedule unit.
    DstRow.clear();
    DstColIdx.clear();

    // Construct the step of Dst schedule unit.
    for (unsigned i = 0; i < DstNumSVs; ++i) {
      DstRow.push_back((REAL)(DstASAP + i));
      DstColIdx.push_back(DstFstIdx + i + 1);
    }

    for (VSUnit::const_dep_iterator DI = DstU->edge_begin(),
         DE = DstU->dep_end(); DI != DE;++DI) {
      const VSUnit *SrcU = *DI;
      VDEdge *Edge = DI.getEdge();

      assert(SrcU->isControl() && "Unexpected datapath in scheduler!");
      // Get the index of first step variable of U.
      unsigned SrcFstIdx = getFstSVIdxOf(SrcU),
               // And the total step variable count of U.
               SrcNumSVs = getTimeFrame(SrcU),
               SrcASAP = getASAPStep(SrcU);

      Row.append(DstRow.begin(), DstRow.end());
      ColIdx.append(DstColIdx.begin(), DstColIdx.end());

      for (unsigned i = 0; i < SrcNumSVs; ++i) {
        Row.push_back(-((REAL)((SrcASAP + i))));
        // The column index is started from 1 in lp_slove.
        ColIdx.push_back(SrcFstIdx + i + 1);
      }

      // Add the constraints to the model.
      if (!add_constraintex(lp, ColIdx.size(), Row.data(), ColIdx.data(),
                            GE, int(Edge->getLatency()) - int((getMII() *  Edge->getItDst()))))
        report_fatal_error("ILPScheduler: Can NOT add Precedence constraints"
                           " of schedule unit " + utostr_32(DstU->getIdx()) +
                           " to " + utostr_32(SrcU->getIdx()));
      // Since add_constraintex sort Row and ColIdx, we have to clear them and
      // re-add new data to them.
      Row.clear();
      ColIdx.clear();
    }
  }
}

void ILPScheduler::buildFUConstraints(lprec *lp) {
  // Remember the bound schedule unit and corresponding function unit.
  typedef SmallVector<SUToIdx, 16> BoundSUVec;
  typedef std::map<FuncUnitId, BoundSUVec> FU2SUMap;
  FU2SUMap FSMap;

  // Numbers of Unbound function units.
  unsigned zero = 0;
  SmallVector<unsigned, 8> MaxFUCounts(VFUs::NumPostBindFUs, zero),
                           CurFUCounts;

  SmallVector<REAL, 128> Row;
  SmallVector<int, 128> ColIdx;

  for (ActiveSUMap::const_iterator AI = ActiveSUs.begin(), AE =ActiveSUs.end();
       AI  != AE;  ++AI) {
    unsigned CurStep = AI->first;
    ActiveSUVec SUs = AI->second;
    // Clear the function unit counts.
    CurFUCounts.assign(VFUs::NumNonTrivialCommonFUs, 0);

    // Classify the active schedule units by its function unit id.
    for (ActiveSUVec::const_iterator I = SUs.begin(), E = SUs.end();
         I != E; ++I) {
      SUToIdx SUI  = *I;
      const VSUnit* U = SUI.first;

      if (U->getFUType() > VFUs::LastCommonFUType
          || U->getFUType() < VFUs::FirstNonTrivialFUType)
        continue;

      // Count the unbound function units in current step.
      if (!U->getFUId().isBound())
        ++CurFUCounts[U->getFUType() - VFUs::FirstNonTrivialFUType];

      FSMap[U->getFUId()].push_back(SUI);
    }

    // Update the max function unit count.
    // Use std::max<unsigned> to avoid C2589 in MSVC
    for (VFUs::FUTypes fu = VFUs::FirstNonTrivialFUType;
         fu <= VFUs::LastPostBindFUType; fu = (VFUs::FUTypes)(fu + 1u)) {
      unsigned Idx = fu - VFUs::FirstNonTrivialFUType;
      MaxFUCounts[Idx] = std::max<unsigned>(MaxFUCounts[Idx], CurFUCounts[Idx]);
    }

    // Build the constraint for function units.
    for (FU2SUMap::const_iterator I = FSMap.begin(), E = FSMap.end();
         I != E; ++I) {
      FuncUnitId Id = I->first;
      for (BoundSUVec::const_iterator UI = I->second.begin(),
           UE = I->second.end(); UI != UE; ++UI) {
        SUToIdx SI  = *UI;

        Row.push_back(1.0);
        ColIdx.push_back(SI.second);
      }

      if (Id.isBound()) {
        if (!add_constraintex(lp, ColIdx.size(), Row.data(), ColIdx.data(),
                              LE, 1))
          report_fatal_error("ILPScheduler: Can NOT add FU constraints"
                             " at step " + utostr_32(CurStep) +
                             " for FU "
                             + std::string(VFUDesc::getTypeName(Id.getFUType())));
      } else {
        Row.push_back(-1.0);
        ColIdx.push_back(getIdxOf(Id.getFUType()) + 1);
        if (!add_constraintex(lp, ColIdx.size(), Row.data(), ColIdx.data(),
                              LE, 0))
          report_fatal_error("ILPScheduler: Can NOT add FU constraints"
                             " at step " + utostr_32(CurStep) +
                             " for FU "
                             + std::string(VFUDesc::getTypeName(Id.getFUType())));
      }

      Row.clear();
      ColIdx.clear();
    }

    // Set up the bounds for function unit count variable.
    for (VFUs::FUTypes fu = VFUs::FirstNonTrivialFUType;
         fu <= VFUs::LastPostBindFUType; fu = (VFUs::FUTypes)(fu + 1u))
      if (!set_bounds(lp, getIdxOf(fu) + 1, 0.0,
                      MaxFUCounts[fu - VFUs::FirstNonTrivialFUType]))
          report_fatal_error("ILPScheduler: Can NOT add FU bounds for FU "
                             + std::string(VFUDesc::getTypeName(fu)));


    FSMap.clear();
  }
}

void ILPScheduler::buildObject(lprec *lp) {
  SmallVector<REAL, 128> Row;
  SmallVector<int, 128> ColIdx;

  for (VFUs::FUTypes fu = VFUs::FirstNonTrivialFUType;
       fu <= VFUs::LastPostBindFUType; fu = (VFUs::FUTypes)(fu + 1u)) {
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

  typedef VSchedGraph::sched_iterator it;
  for (it I = State.sched_begin(), E = State.sched_end(); I != E; ++I) {
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

  if (allNodesSchedued()) return true;

  NumStepVars = buildSVIdx();

  // Set up the variable indexes.
  StepIdx = getIdxOf(VFUs::LastCommonFUType) + 1;

  TotalVariables = StepIdx + 1;

  lprec *lp = make_lp(0,TotalVariables);

  set_add_rowmode(lp, TRUE);

  setUpVariables(lp);

  // Build the constraints.
  buildOneActiveStepConstraints(lp);
  buildPrecedenceConstraints(lp);
  buildFUConstraints(lp);
  // Turn off the add rowmode and start to solve the model.
  set_add_rowmode(lp, FALSE);

  buildObject(lp);

  set_verbose(lp, CRITICAL);
  DEBUG(set_verbose(lp, FULL));

  set_presolve(lp, PRESOLVE_ROWS | PRESOLVE_COLS | PRESOLVE_LINDEP
#ifdef LPSOLVE_SOS
                   | PRESOLVE_SOS
#endif
                   | PRESOLVE_IMPLIEDFREE | PRESOLVE_REDUCEGCD
                   | PRESOLVE_PROBEFIX | PRESOLVE_PROBEREDUCE
                   | PRESOLVE_ROWDOMINATE /*| PRESOLVE_COLDOMINATE lpsolve bug*/
                   | PRESOLVE_MERGEROWS
                   | PRESOLVE_BOUNDS,
               get_presolveloops(lp));

  DEBUG(write_lp(lp, "log.lp"));

  TotalRows = get_Nrows(lp);
  DEBUG(dbgs() << "The model has " << TotalVariables
               << "x" << TotalRows << '\n');

  // TODO: Allow set the timeout with constraint.
  set_timeout(lp, 300);

  if (StopAtFirst) set_break_at_first(lp, TRUE);

  DEBUG(dbgs() << "Timeout is set to " << get_timeout(lp) << "secs.\n");

  int result = solve(lp);

  DEBUG(dbgs() << "ILP result is: "<< transSolveResult(result) << "\n");
  DEBUG(dbgs() << "Time elapsed: " << time_elapsed(lp) << "\n");

  switch (result) {
  case INFEASIBLE:
    delete_lp(lp);
    return false;
  case SUBOPTIMAL:
    DEBUG(dbgs() << "Note: suboptimal schedule found!\n");
  case OPTIMAL:
  case PRESOLVED:
    break;
  default:
    report_fatal_error(Twine("ILPScheduler Schedule fail: ")
                       + Twine(transSolveResult(result)));
  }

  // Finally, Schedule the state with the ILP result.
  buildSchedule(lp);
  delete_lp(lp);

  return true;
}
