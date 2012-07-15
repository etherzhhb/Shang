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
#define DEBUG_TYPE "SDCdebug"
#include "llvm/Support/Debug.h"

using namespace llvm;

SDCScheduler::SDCScheduler(VSchedGraph &S)
  : SchedulingBase(S), NumVars(0), NumInst(0) {
}

void SDCScheduler::createStepVariables(lprec *lp) {
  unsigned Col =  1;
  typedef VSchedGraph::sched_iterator it;
  for (it I = State.sched_begin(),E = State.sched_end();I != E; ++I) {
    ++NumInst;
    const VSUnit* U = *I;
    // Set up the scheduling variables for VSUnits.
    SUIdx[U] = NumVars;
    for(unsigned i = 0, j = getMaxLatency(U); i <= j; ++i){
      std::string SVStart = "sv" + utostr_32(U->getIdx()) + "start" + utostr_32(i);
      DEBUG(dbgs()<<"the col is"<<Col<<"the colName is"<<SVStart<<"\n");
      set_col_name(lp, Col, const_cast<char*>(SVStart.c_str()));
      set_int(lp,Col,TRUE);
      ++Col;
      ++NumVars;
    }
  }
}

void SDCScheduler::addStepConstraints(lprec *lp){
  int Col[2];
  REAL Val[2];
  //Build the constraints for LP Variables as SVXStart1 - SVXstart0 = 1.
  for(SUIdxIt EI = SUIdx.begin(), EE = SUIdx.end(); EI != EE; ++EI){
    unsigned Idx = EI->second;
    const VSUnit* U = EI->first;
    unsigned MaxLatency = getMaxLatency(U);
    if(MaxLatency < 1) continue;
    for(unsigned i = 0, j = MaxLatency; i < j; ++i){
      Col[0] = 1 + Idx + i;
      Val[0] = -1.0;
      Col[1] = 1 + Idx + i + 1;
      Val[1] = 1.0;
      if(!add_constraintex(lp, 2, Val, Col, EQ, 1.0))
        report_fatal_error("SDCScheduler: Can NOT step Variable Constraints"
                           " at VSUnit " + utostr_32(U->getIdx()) );
    }
  }
}

void SDCScheduler::addDependencyConstraints(lprec *lp) {
  int Col[2];
  REAL Val[2];

  typedef VSchedGraph::sched_iterator sched_it;
  for(sched_it I = State.sched_begin(), E = State.sched_end(); I != E; ++I) {
    const VSUnit *U = *I;
    assert(U->isControl() && "Unexpected datapath in scheduler!");

    // Build the constraint for Dst_SU_startStep - Src_SU_endStep >= 0.
    typedef VSUnit::const_use_iterator use_it;
    for (use_it DI = U->use_begin(), DE = U->use_end(); DI != DE;++DI) {
      const VSUnit *depIn = *DI;
      const VDEdge *Edge = depIn->getEdgeFrom(U);
      unsigned SrcEndIdx =  SUIdx[U] + Edge->getLatency();
      unsigned DstStartIdx = SUIdx[depIn];

      // Build the LP.
      Col[0] = 1 + SrcEndIdx;
      Val[0] = -1.0;
      Col[1] = 1 + DstStartIdx;
      Val[1] = 1.0;
      if(!add_constraintex(lp, 2, Val, Col, GE, 0.0))
        report_fatal_error("SDCScheduler: Can NOT step Dependency Constraints"
                           " at VSUnit " + utostr_32(U->getIdx()) );
    }
  }
}

void SDCScheduler::buildASAPObject() {
  std::vector<int> Indices(NumInst);
  std::vector<REAL> Coefficients(NumInst);

  unsigned Col = 0;
  //Build the ASAP object function.
  typedef VSchedGraph::sched_iterator it;
  for(it I = State.sched_begin(),E = State.sched_end();I != E; ++I) {
      const VSUnit* U = *I;
    unsigned Idx = SUIdx[U];
    Indices[Col] = 1 + Idx;
    Coefficients[Col] = 1.0;
    ++Col;
  }

  set_obj_fnex(lp, Col, Coefficients.data(), Indices.data());
  set_minim(lp);
  DEBUG(write_lp(lp, "log.lp"));
}

void SDCScheduler::buildOptimizingSlackDistributionObject(){
  std::vector<int> Indices(NumInst);
  std::vector<REAL> Coefficients(NumInst);

  unsigned Col = 0;
  //Build the Optimizing Slack object function.
  typedef VSchedGraph::sched_iterator it;
  for(it I = State.sched_begin(),E = State.sched_end();I != E; ++I) {
    const VSUnit* U = *I;
    int Indeg = U->countValDeps();
    int Outdeg = U->countValUses();
    unsigned Idx = SUIdx[U];
    Indices[Col] = 1 + Idx;
    Coefficients[Col] = Outdeg - Indeg;
    ++Col;
  }

  set_obj_fnex(lp, Col, Coefficients.data(), Indices.data());
  set_maxim(lp);

  DEBUG(write_lp(lp, "log.lp"));

}

void SDCScheduler::buildSchedule(lprec *lp) {
  typedef VSchedGraph::sched_iterator it;
  for(it I = State.sched_begin(),E = State.sched_end();I != E; ++I) {
      VSUnit *U = *I;
    unsigned Offset = SUIdx[U];
    unsigned j = get_var_primalresult(lp, TotalRows + Offset + 1);
    DEBUG(dbgs() << "the row is:" << TotalRows + Offset + 1
                 <<"the result is:" << j << "\n");
    U->scheduledTo(j + State.EntrySlot);
  }
}

bool SDCScheduler::scheduleState() {
  buildTimeFrameAndResetSchedule(true);

  BasicLinearOrderGenerator BLOG(*this);
  BLOG.addLinOrdEdge();

  DEBUG(viewGraph());
  //Ensure there is no resource conflict in critical path.
  if (!scheduleCriticalPath(false))
    return false;

  if (allNodesSchedued()) return true;

  lp = make_lp(0, NumVars);
  set_add_rowmode(lp, TRUE);

  // Build the step variables.
  createStepVariables(lp);

  // Build the constraints.
  addStepConstraints(lp);
  addDependencyConstraints(lp);

  // Turn off the add rowmode and start to solve the model.
  set_add_rowmode(lp, FALSE);
  TotalRows = get_Nrows(lp);
  buildASAPObject();
  //buildOptimizingSlackDistributionObject();

  set_verbose(lp, CRITICAL);
  DEBUG(set_verbose(lp, FULL));

  int result = solve(lp);

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
    report_fatal_error(Twine("SDCScheduler Schedule fail: "));
  }
  // Schedule the state with the ILP result.
  buildSchedule(lp);

  DEBUG(viewGraph());

  delete_lp(lp);
  SUIdx.clear();
  return true;
}

