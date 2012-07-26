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

unsigned SDCSchedulingBase::createLPAndVariables(su_it I, su_it E) {
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

void SDCSchedulingBase::buildASAPObject(su_it I, su_it E, double weight) {
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

void SDCSchedulingBase::buildOptSlackObject(su_it I, su_it E, double weight) {
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
                                      su_it I, su_it E) {
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

unsigned SDCSchedulingBase::calculateMinSlotsFromEntry(VSUnit *BBEntry,
                                                       const B2SMapTy &Map) {
  unsigned SlotsFromEntry = BBEntry->getSlot();

  typedef VSUnit::dep_iterator dep_it;
  for (dep_it I = BBEntry->dep_begin(), E = BBEntry->dep_end(); I != E; ++I) {
    VSUnit *PredTerminator = *I;
    //DEBUG(dbgs() << "MBB#" << PredTerminator->getParentBB()->getNumber()
    //             << "->MBB#" << BBEntry->getParentBB()->getNumber() << " slack: "
    //             << (BBEntry->getSlot() - PredTerminator->getSlot()) << '\n');

    MachineBasicBlock *PredBB = PredTerminator->getParentBB();
    unsigned SlotsFromPredExit = Map[PredBB->getNumber()];
    assert(SlotsFromPredExit && "Not visiting the BBs in topological order?");
    SlotsFromEntry = std::min<unsigned>(SlotsFromEntry, SlotsFromPredExit);
  }

  return SlotsFromEntry;
}

int SDCSchedulingBase::calulateMinInterBBSlack(VSUnit *BBEntry, VSchedGraph &G,
                                               const B2SMapTy &Map,
                                               unsigned MinSlotsForEntry) {
  unsigned EntrySlot = BBEntry->getSlot();
  MachineBasicBlock *MBB = BBEntry->getParentBB();
  typedef VSUnit::use_iterator use_it;
  typedef VSUnit::dep_iterator dep_it;
  // The minimal slack from the predecessors of the BB to the entry of the BB.
  int MinInterBBSlack = 0;

  for (use_it I = BBEntry->use_begin(), E = BBEntry->use_end(); I != E; ++I) {
    VSUnit *U = *I;
    if (U->getParentBB() != MBB) continue;

    unsigned USlot = U->getSlot();
    for (dep_it DI = U->dep_begin(), DE = U->dep_end(); DI != DE; ++DI) {
      if (!DI.isCrossBB()) continue;

      MachineBasicBlock *SrcBB = DI->getParentBB();
      assert(SrcBB != MBB && "Not cross BB depenency!");
      VSUnit *SrcTerminator = G.lookUpTerminator(SrcBB);
      unsigned SrcExitSlot = Map[SrcBB->getNumber()];
      assert(SrcExitSlot && "Unexpected cross BB back-edge!");
      // Calculate the latency distributed between the SrcBB and the SnkBB.
      assert(USlot >= EntrySlot && SrcTerminator->getSlot() >= DI->getSlot()
              && "Bad schedule!");
      int InterBBLatency = DI->getLatency() - (USlot - EntrySlot)
                            - (SrcTerminator->getSlot() - DI->getSlot());
      // Calculate the minimal latency between the SrcBB and SnkBB.
      int ActualInterBBlatency = MinSlotsForEntry - SrcExitSlot;
      DEBUG(dbgs().indent(4) << "Found InterBBLatency and ActualInterBBlatency"
                              << InterBBLatency << ' '
                              << ActualInterBBlatency << '\n');
      // Calculate the slack.
      int Slack = ActualInterBBlatency - InterBBLatency;
      MinInterBBSlack = std::min<int>(MinInterBBSlack, Slack);
    }
  }

  return MinInterBBSlack;
}

void SDCSchedulingBase::fixInterBBLatency(VSchedGraph &G) {
  B2SMapTy ExitSlots(G.num_bbs(), 0);

  std::vector<std::pair<VSUnit*, VSUnit::dep_iterator> > WorkStack;
  VSUnit *ExitRoot = G.getExitRoot();
  WorkStack.push_back(std::make_pair(ExitRoot, ExitRoot->dep_begin()));

  // Visit the blocks in topological order, and ignore the pseudo exit node.
  while (WorkStack.size() > 1) {
    VSUnit *U = WorkStack.back().first;
    VSUnit::dep_iterator ChildIt = WorkStack.back().second;

    if (ChildIt == U->dep_end()) {
      WorkStack.pop_back();

      // All dependencies visited, now visit the current BBEntry.
      assert(U->isBBEntry() && "Unexpected non-entry node!");
      unsigned SlotsFromEntry = calculateMinSlotsFromEntry(U, ExitSlots);

      int MinInterBBSlack =
        calulateMinInterBBSlack(U, G, ExitSlots, SlotsFromEntry);

      DEBUG(dbgs() << "Minimal Slack: " << MinInterBBSlack << '\n');
      assert(MinInterBBSlack == 0
             && "Inserting delay block to fix negative slack is not yet"
                " implemented!");

      MachineBasicBlock *MBB = U->getParentBB();
      unsigned ExitSlot = SlotsFromEntry + G.getTotalSlot(MBB);
      assert(ExitSlot && "Bad schedule!");
      ExitSlots[MBB->getNumber()] = ExitSlot;

      continue;
    }

    VSUnit *ChildNode = *ChildIt;
    ++WorkStack.back().second;

    assert(ChildNode->isTerminator() && "Unexpected non-terminator node!");
    // Jump to the entry of the parent block of the terminator.
    ChildNode = G.lookupSUnit(ChildNode->getParentBB());
    WorkStack.push_back(std::make_pair(ChildNode, ChildNode->dep_begin()));
  }

  assert(WorkStack.back().first == ExitRoot && "Stack broken!");
}

template<bool IsCtrlPath>
void SDCScheduler<IsCtrlPath>::addDependencyConstraints(lprec *lp, const VSUnit *U) {
  unsigned DstIdx = 0;
  int DstSlot = U->getSlot();
  if (DstSlot == 0) DstIdx = getSUIdx(U);
  SmallVector<int, 2> Col;
  SmallVector<REAL, 2> Coeff;

  // Build the constraint for Dst_SU_startStep - Src_SU_endStep >= Latency.
  for (const_dep_it DI = dep_begin(U), DE = dep_end(U); DI != DE; ++DI) {
    assert(!DI.isLoopCarried()
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
      Col.push_back(SrcIdx);
      Coeff.push_back(-1.0);
    }

    if (DstSlot == 0) {
      Col.push_back(DstIdx);
      Coeff.push_back(1.0);
    }

    int EqTy = (DI.getEdgeType() == VDEdge::FixedTiming) ? EQ : GE;

    if(!add_constraintex(lp, Col.size(), Coeff.data(), Col.data(), EqTy, RHS))
      report_fatal_error("SDCScheduler: Can NOT step Dependency Constraints"
                         " at VSUnit " + utostr_32(U->getIdx()) );
  }
}

template<bool IsCtrlPath>
void SDCScheduler<IsCtrlPath>::addDependencyConstraints(lprec *lp) {
  for(VSchedGraph::sched_iterator I = su_begin(G), E = su_end(G); I != E; ++I)
    addDependencyConstraints(lp, *I);
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
  buildSchedule(lp, TotalRows, su_begin(G), su_end(G));

  delete_lp(lp);
  lp = 0;
  SUIdx.clear();
  ObjFn.clear();
  return true;
}

template class SDCScheduler<false>;
template class SDCScheduler<true>;
