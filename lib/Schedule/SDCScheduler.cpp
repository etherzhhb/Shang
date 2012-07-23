//===- SDCScheduler.cpp ------- SDCScheduler --------------------*- C++ -*-===//
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
// 
// 
//
//===----------------------------------------------------------------------===//

#include "SchedulingBase.h"
#include "vtm/VInstrInfo.h"
#include "lp_solve/lp_lib.h"
#define DEBUG_TYPE "sdc-scheduler"
#include "llvm/Support/Debug.h"

using namespace llvm;

void LPObjFn::setLPObj(lprec *lp) const {
  std::vector<int> Indices;
  std::vector<REAL> Coefficients;

  //Build the ASAP object function.
  typedef VSchedGraph::sched_iterator it;
  for(const_iterator I = begin(), E = end(); I != E; ++I) {
    Indices.push_back(I->first);
    Coefficients.push_back(I->second);
  }

  set_obj_fnex(lp, size(), Coefficients.data(), Indices.data());
  set_maxim(lp);
  DEBUG(write_lp(lp, "log.lp"));
}

SDCScheduler::SDCScheduler(VSchedGraph &S) : SchedulingBase(S), NumVars(0), lp(0)
{}

unsigned SDCScheduler::createLPAndVariables() {
  lp = make_lp(0, NumVars);
  unsigned Col =  1;
  typedef VSchedGraph::sched_iterator it;
  for (it I = G.sched_begin(),E = G.sched_end();I != E; ++I) {
    const VSUnit* U = *I;
    if (U->isScheduled()) continue;

    // Set up the scheduling variables for VSUnits.
    bool inserted = SUIdx.insert(std::make_pair(U, NumVars)).second;
    assert(inserted && "Index already existed!");
    (void) inserted;
    std::string SVStart = "sv" + utostr_32(U->getIdx()) + "start";
    DEBUG(dbgs() <<"Col#" << Col << " name: " <<SVStart << "\n");
    set_col_name(lp, Col, const_cast<char*>(SVStart.c_str()));
    set_int(lp, Col, TRUE);
    ++Col;
    ++NumVars;
  }

  return NumVars;
}

void SDCScheduler::addDependencyConstraints(lprec *lp, const VSUnit *U) {
  unsigned DstIdx = 0;
  int DstSlot = U->getSlot();
  if (DstSlot == 0) DstIdx = getSUIdx(U);
  SmallVector<int, 2> Col;
  SmallVector<REAL, 2> Coeff;

  // Build the constraint for Dst_SU_startStep - Src_SU_endStep >= Latency.
  typedef VSUnit::const_dep_iterator dep_it;
  for (dep_it DI = U->dep_begin(), DE = U->dep_end(); DI != DE; ++DI) {
    assert(DI.getDistance() == 0
           && "Loop carried dependencies cannot handled by SDC scheduler!");

    const VSUnit *Dep = *DI;
    unsigned SrcIdx = 0;
    int SrcSlot = Dep->getSlot();
    if (SrcSlot == 0) SrcIdx = getSUIdx(Dep);

    int RHS = DI.getLatency() - DstSlot + SrcSlot;

    // Both SU is scheduled.
    if (SrcSlot && DstSlot) {
      assert(0 >= RHS && "Bad schedule!");
      continue;
    }

    Col.clear();
    Coeff.clear();

    // Build the constraint.
    if (SrcSlot == 0) {
      Col.push_back(1 + SrcIdx);
      Coeff.push_back(-1.0);
    }

    if (DstSlot == 0) {
      Col.push_back(1 + DstIdx);
      Coeff.push_back(1.0);
    }

    int EqTy = (DI.getEdgeType() == VDEdge::edgeFixedTiming) ? EQ : GE;

    if(!add_constraintex(lp, Col.size(), Coeff.data(), Col.data(), EqTy, RHS))
      report_fatal_error("SDCScheduler: Can NOT step Dependency Constraints"
                         " at VSUnit " + utostr_32(U->getIdx()) );
  }
}

void SDCScheduler::addDependencyConstraints(lprec *lp) {
  typedef VSchedGraph::sched_iterator sched_it;
  for(sched_it I = G.sched_begin(), E = G.sched_end(); I != E; ++I)
    addDependencyConstraints(lp, *I);
}

void SDCScheduler::buildASAPObject(double weight) {
  //Build the ASAP object function.
  typedef VSchedGraph::sched_iterator it;
  for(it I = G.sched_begin(),E = G.sched_end();I != E; ++I) {
    const VSUnit* U = *I;

    if (U->isScheduled()) continue;

    unsigned Idx = getSUIdx(U);
    // Because LPObjFn will set the objective function to maxim instead of minim,
    // we should use -1.0 instead of 1.0 as coefficient
    ObjFn[1 + Idx] += - 1.0 * weight;
  }
}

void SDCScheduler::buildOptSlackObject(double weight){
  typedef VSchedGraph::sched_iterator it;
  for(it I = G.sched_begin(),E = G.sched_end();I != E; ++I) {
    const VSUnit* U = *I;

    if (U->isScheduled()) continue;

    int Indeg = U->countValDeps();
    int Outdeg = U->countValUses();
    unsigned Idx = getSUIdx(U);
    ObjFn[1 + Idx] += (Outdeg - Indeg) * weight;
  }
}

void SDCScheduler::buildSchedule(lprec *lp) {
  typedef VSchedGraph::sched_iterator it;
  for(it I = G.sched_begin(),E = G.sched_end();I != E; ++I) {
    VSUnit *U = *I;

    if (U->isScheduled()) continue;

    unsigned Offset = getSUIdx(U);
    unsigned j = get_var_primalresult(lp, TotalRows + Offset + 1);
    DEBUG(dbgs() << "At row:" << TotalRows + Offset + 1
                 << " the result is:" << j << "\n");

    assert(j && "Bad result!");
    U->scheduledTo(j);
  }
}

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

bool SDCScheduler::solveLP(lprec *lp) {
  set_verbose(lp, CRITICAL);
  DEBUG(set_verbose(lp, FULL));

  set_presolve(lp, PRESOLVE_ROWS | PRESOLVE_COLS | PRESOLVE_LINDEP
                   | PRESOLVE_IMPLIEDFREE | PRESOLVE_REDUCEGCD
                   | PRESOLVE_PROBEFIX | PRESOLVE_PROBEREDUCE
                   | PRESOLVE_ROWDOMINATE /*| PRESOLVE_COLDOMINATE lpsolve bug*/
                   | PRESOLVE_MERGEROWS
                   | PRESOLVE_BOUNDS,
               get_presolveloops(lp));

  DEBUG(write_lp(lp, "log.lp"));

  TotalRows = get_Nrows(lp);
  DEBUG(dbgs() << "The model has " << NumVars
               << "x" << TotalRows << '\n');

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

  return true;
}

bool SDCScheduler::schedule() {
  ObjFn.setLPObj(lp);

  set_add_rowmode(lp, TRUE);

  // Build the constraints.
  addDependencyConstraints(lp);

  // Turn off the add rowmode and start to solve the model.
  set_add_rowmode(lp, FALSE);

  TotalRows = get_Nrows(lp);

  if (!solveLP(lp)) return false;

  // Schedule the state with the ILP result.
  buildSchedule(lp);

  delete_lp(lp);
  lp = 0;
  SUIdx.clear();
  ObjFn.clear();
  return true;
}

bool SDCScheduler::scheduleState() {
  buildTimeFrameAndResetSchedule(true);
  BasicLinearOrderGenerator::addLinOrdEdge(*this);
  // Build the step variables, and no need to schedule at all if all SUs have
  // been scheduled.
  if (!createLPAndVariables()) return true;

  buildASAPObject(1.0);
  //buildOptSlackObject(0.0);
  return schedule();
}


