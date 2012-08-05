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

void SDCSchedulingBase::LPObjFn::setLPObj(lprec *lp) const {
  std::vector<int> Indices;
  std::vector<REAL> Coefficients;

  //Build the ASAP object function.
  for(const_iterator I = begin(), E = end(); I != E; ++I) {
    Indices.push_back(I->first);
    Coefficients.push_back(I->second);
  }

  set_obj_fnex(lp, size(), Coefficients.data(), Indices.data());
  set_maxim(lp);
  DEBUG(write_lp(lp, "log.lp"));
}

unsigned SDCSchedulingBase::createStepVariable(const VSUnit* U, unsigned Col) {
  // Set up the step variable for the VSUnit.
  bool inserted = SUIdx.insert(std::make_pair(U, Col)).second;
  assert(inserted && "Index already existed!");
  (void) inserted;
  std::string SVStart = "sv" + utostr_32(U->getIdx());
  DEBUG(dbgs() <<"Col#" << Col << " name: " <<SVStart << "\n");
  set_col_name(lp, Col, const_cast<char*>(SVStart.c_str()));
  set_int(lp, Col, TRUE);
  return Col + 1;
}

unsigned SDCSchedulingBase::createLPAndVariables(iterator I, iterator E) {
  lp = make_lp(0, 0);
  unsigned Col =  1;
  while (I != E) {
    const VSUnit* U = *I++;
    if (U->isScheduled()) continue;

    Col = createStepVariable(U, Col);
  }

  return Col - 1;
}

unsigned SDCSchedulingBase::addSoftConstraint(const VSUnit *Src, const VSUnit *Dst,
                                               unsigned Slack, double Penalty) {
  if (Src->isScheduled() && Dst->isScheduled()) return 0;

  unsigned NextCol = get_Ncolumns(lp) + 1;
  SoftConstraint C = { Penalty, Src, Dst, NextCol, Slack };
  SoftCstrs.push_back(C);
  std::string SlackName = "slack" + utostr_32(NextCol);
  DEBUG(dbgs() <<"Col#" << NextCol << " name: " <<SlackName << "\n");
  set_col_name(lp, NextCol, const_cast<char*>(SlackName.c_str()));
  set_int(lp, NextCol, TRUE);
  return NextCol;
}

void SDCSchedulingBase::addSoftConstraints(lprec *lp) {
  typedef SoftCstrVecTy::iterator iterator;
  SmallVector<int, 3> Col;
  SmallVector<REAL, 3> Coeff;

  // Build the constraint Dst - Src <= Latency - Slack
  for (iterator I = SoftCstrs.begin(), E = SoftCstrs.end(); I != E; ++I) {
    SoftConstraint &C = *I;

    unsigned DstIdx = 0;
    int DstSlot = C.Dst->getSlot();
    if (DstSlot == 0) DstIdx = getSUIdx(C.Dst);

    unsigned SrcIdx = 0;
    int SrcSlot = C.Src->getSlot();
    if (SrcSlot == 0) SrcIdx = getSUIdx(C.Src);

    int RHS = C.Slack - DstSlot + SrcSlot;

    // Both SU is scheduled.
    if (SrcSlot && DstSlot) continue;

    Col.clear();
    Coeff.clear();

    // Build the constraint.
    if (SrcSlot == 0) {
      Col.push_back(SrcIdx);
      Coeff.push_back(-1.0);
    }

    if (DstSlot == 0) {
      Col.push_back(DstIdx);
      Coeff.push_back(1.0);
    }

    // Add the slack variable.
    Col.push_back(C.SlackIdx);
    Coeff.push_back(1.0);

    if(!add_constraintex(lp, Col.size(), Coeff.data(), Col.data(), GE, RHS))
      report_fatal_error("SDCScheduler: Can NOT step soft Constraints"
                         " SlackIdx:" + utostr_32(C.SlackIdx));
  }
}

void SDCSchedulingBase::addSoftConstraintsPenalties(double weight) {
  typedef SoftCstrVecTy::iterator iterator;
  for (iterator I = SoftCstrs.begin(), E = SoftCstrs.end(); I != E; ++I) {
    const SoftConstraint &C = *I;
    // Ignore the eliminated soft constraints.
    if (C.SlackIdx == 0) continue;

    ObjFn[C.SlackIdx] -= C.Penalty * weight;
  }
}

void SDCSchedulingBase::buildASAPObject(iterator I, iterator E, double weight) {
  //Build the ASAP object function.
  while (I != E) {
    const VSUnit* U = *I++;

    if (U->isScheduled()) continue;

    unsigned Idx = getSUIdx(U);
    // Because LPObjFn will set the objective function to maxim instead of minim,
    // we should use -1.0 instead of 1.0 as coefficient
    ObjFn[Idx] += - 1.0 * weight;
  }
}

void SDCSchedulingBase::buildOptSlackObject(iterator I, iterator E, double weight) {
  while (I != E) {
    const VSUnit* U = *I++;

    if (U->isScheduled()) continue;

    int Indeg = U->countValDeps();
    int Outdeg = U->countValUses();
    unsigned Idx = getSUIdx(U);
    ObjFn[Idx] += (Outdeg - Indeg) * weight;
  }
}

void SDCSchedulingBase::buildSchedule(lprec *lp, unsigned TotalRows,
                                      iterator I, iterator E) {
  while (I != E) {
    VSUnit *U = *I++;

    if (U->isScheduled()) continue;

    unsigned Idx = getSUIdx(U);
    unsigned j = get_var_primalresult(lp, TotalRows + Idx);
    DEBUG(dbgs() << "At row:" << TotalRows + Idx
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

bool SDCSchedulingBase::solveLP(lprec *lp) {
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

  unsigned TotalRows = get_Nrows(lp), NumVars = get_Ncolumns(lp);
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

namespace {
struct ConstraintHelper {
  int SrcSlot, DstSlot;
  unsigned SrcIdx, DstIdx;

  ConstraintHelper()
    : SrcSlot(0), DstSlot(0), SrcIdx(0), DstIdx(0) {}

  void resetSrc(const VSUnit *Src, const SDCSchedulingBase *S) {
    SrcSlot = Src->getSlot();
    SrcIdx = SrcSlot == 0 ? S->getSUIdx(Src) : 0;
  }

  void resetDst(const VSUnit *Dst, const SDCSchedulingBase *S) {
    DstSlot = Dst->getSlot();
    DstIdx = DstSlot == 0 ? S->getSUIdx(Dst) : 0;
  }

  void addConstraintToLP(VDEdge Edge, lprec *lp) {
    SmallVector<int, 2> Col;
    SmallVector<REAL, 2> Coeff;

    int RHS = Edge.getLatency() - DstSlot + SrcSlot;

    // Both SU is scheduled.
    if (SrcSlot && DstSlot) {
      assert(0 >= RHS && "Bad schedule!");
      return;
    }

    // Build the constraint.
    if (SrcSlot == 0) {
      assert(SrcIdx && "Bad SrcIdx!");
      Col.push_back(SrcIdx);
      Coeff.push_back(-1.0);
    }

    if (DstSlot == 0) {
      assert(DstIdx && "Bad DstIdx!");
      Col.push_back(DstIdx);
      Coeff.push_back(1.0);
    }

    int EqTy = (Edge.getEdgeType() == VDEdge::FixedTiming) ? EQ : GE;

    // Adjust the inter-bb latency.
    RHS -= 0;

    if(!add_constraintex(lp, Col.size(), Coeff.data(), Col.data(), EqTy, RHS))
      report_fatal_error("SDCScheduler: Can NOT step Dependency Constraints"
                         " at VSUnit " + utostr_32(DstIdx));
  }
};
}

template<bool IsCtrlPath>
inline void SDCScheduler<IsCtrlPath>::addDependencyConstraints(lprec *lp) {
  for(VSchedGraph::const_iterator I = begin(), E = end(); I != E; ++I) {
    const VSUnit *U = *I;
    ConstraintHelper H;
    H.resetDst(U, this);

    // Build the constraint for Dst_SU_startStep - Src_SU_endStep >= Latency.
    for (const_dep_it DI = dep_begin(U), DE = dep_end(U); DI != DE; ++DI) {
      assert(!DI.isLoopCarried()
        && "Loop carried dependencies cannot handled by SDC scheduler!");
      const VSUnit *Src = *DI;

      H.resetSrc(Src, this);
      H.addConstraintToLP(DI.getEdge(), lp);
    }

    if (IsCtrlPath) continue;

    // The data-path scheduling units are also constrained by the control path
    // scheduling units.
    H.resetSrc(U, this);

    for (const_use_it UI = use_begin(U), UE = use_end(U); UI != UE; ++UI) {
      const VSUnit *Use = *UI;
      if (!Use->isControl()) continue;

      H.resetDst(Use, this);
      H.addConstraintToLP(Use->getEdgeFrom<IsCtrlPath>(U), lp);
    }
  }
}

template<bool IsCtrlPath>
bool SDCScheduler<IsCtrlPath>::schedule() {
  ObjFn.setLPObj(lp);

  set_add_rowmode(lp, TRUE);

  // Build the constraints.
  addDependencyConstraints(lp);
  addSoftConstraints(lp);

  // Turn off the add rowmode and start to solve the model.
  set_add_rowmode(lp, FALSE);
  unsigned TotalRows = get_Nrows(lp);

  if (!solveLP(lp)) return false;

  // Schedule the state with the ILP result.
  buildSchedule(lp, TotalRows, begin(), end());

  delete_lp(lp);
  lp = 0;
  SUIdx.clear();
  ObjFn.clear();
  return true;
}

template class SDCScheduler<false>;
template class SDCScheduler<true>;
