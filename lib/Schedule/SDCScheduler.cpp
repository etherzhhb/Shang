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
    for(unsigned i = 0, j = getMaxLatency(U); i <= j; i++){
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
  /*const*/ REAL Val[2] = { -1.0, 1.0 };
  //Build the constraints for LP Variables as SVXStart1 - SVXstart0 = 1.
  for(SUIdxIt EI = SUIdx.begin(), EE = SUIdx.end(); EI != EE; ++EI){
    unsigned Idx = EI->second;
    const VSUnit* U = EI->first;
    unsigned MaxLatency = getMaxLatency(U);
    if(MaxLatency < 1) continue;
    for(unsigned i = 0, j = MaxLatency; i < j; ++i){
      Col[0] = 1 + Idx + i;
      Col[1] = 1 + Idx + i + 1;
      if(!add_constraintex(lp, 2, Val, Col, EQ, 1.0))
        report_fatal_error("SDCScheduler: Can NOT step Variable Constraints"
          " at VSUnit " + utostr_32(U->getIdx()) );
    }
  }
}

void SDCScheduler::addDependencyConstraints(lprec *lp) {
  int Col[2];
  /*const*/ REAL Val[2] = { -1.0, 1.0 };
  for(VSchedGraph::sched_iterator I = State.sched_begin(), E = State.sched_end();
      I != E; ++I) {
    const VSUnit *U = *I;
    assert(U->isControl() && "Unexpected datapath in scheduler!");

    // Build the constraint for Dst_SU_startStep - Src_SU_endStep >= 0.
    for (VSUnit::const_use_iterator DI = U->use_begin(),
        DE = U->use_end(); DI != DE;++DI) {
      const VSUnit *depIn = *DI;
      const VDEdge *Edge = depIn->getEdgeFrom(U);
      unsigned Latency = Edge->getLatency();
      unsigned SrcEndIdx =  SUIdx[U] + Edge->getLatency();
      unsigned DstStartIdx = SUIdx[depIn];

      // Build the LP.
      Col[0] = 1 + SrcEndIdx;
      Col[1] = 1 + DstStartIdx;
      if(!add_constraintex(lp, 2, Val, Col, GE, 0.0))
        report_fatal_error("SDCScheduler: Can NOT step Dependency Constraints"
        " at VSUnit " + utostr_32(U->getIdx()));
    }
  }
}

void SDCScheduler::addResourceConstraints(lprec *lp) {
  //The table of the ASAP and the VSUnits.
  Step2SUMap IdenticalMap;
  BoundSUVec OrderIdentity;
  Step2SUMap FUMap;
  //The table of the TimeFrame and the VSUnits.
  Step2SUMap OrderIdentityMap;

  //Get the VSUnits that need add into the resource constraints.
  for(VSchedGraph::sched_iterator I = State.sched_begin(), E = State.sched_end();
    I != E; ++I) {
    const VSUnit *SV = *I;
    if (SV->getFUType() > VFUs::LastCommonFUType
      || SV->getFUType() < VFUs::FirstNonTrivialFUType)
      continue;
      FUMap[SV->getFUType()].push_back(SV);
  }

  for(Step2SUMap::iterator FI = FUMap.begin(), FE = FUMap.end();
      FI != FE; FI++){
    bool MemType = false;
    unsigned FuType = FI->first;
    DEBUG(dbgs()<<"The FU Type is :"<<FuType<<"\n");

    if(FuType == VFUs::BRam || FuType == VFUs::MemoryBus || FuType == VFUs::CalleeFN)
      MemType = true;

    BoundSUVec Set = FI->second;
    if(Set.size()<=1) continue;

    //Map the VSUnits to their ASAPStep.
    BoundSUVec::iterator LastIt = Set.begin();
    for(BoundSUVec::iterator iB = Set.begin(),eB = Set.end(); iB != eB; iB++){
      const VSUnit *V = *iB;
      IdenticalMap[getASAPStep(V)].push_back(V);
    }

    //Sort the IdenticalMap in descending order of the TimeFrame.
    //The TimeFrame means the freedom of the VSUnits in scheduling.
    //SDCScheduler prefer to move the VSUnits that have the bigger TimeFrame
    //to next slot.
    for(Step2SUMap::iterator IS = IdenticalMap.begin(), ES = IdenticalMap.end();
      IS != ES; IS++){
        BoundSUVec V = IS->second;
        unsigned idx = IS->first;
        if(V.size()<=1) continue;
        for(BoundSUVec::iterator iU = V.begin(),eU = V.end(); iU != eU; iU++){
          const VSUnit* U = *iU;
          OrderIdentityMap [getTimeFrame(U)].push_back(U);
        }

        while(!OrderIdentityMap.empty()){
          unsigned MaxTimeFrame = 0;
          for(Step2SUMap::iterator IU = OrderIdentityMap.begin(),
              EU = OrderIdentityMap.end(); IU != EU; IU++){
            unsigned CurTF = IU->first;
            if(CurTF > MaxTimeFrame)
              MaxTimeFrame = CurTF;
          }
          Step2SUMap::iterator it = OrderIdentityMap.find(MaxTimeFrame);
          BoundSUVec B = it->second;
          typedef BoundSUVec::iterator OrderIt;
          for(OrderIt i = B.begin(),e = B.end(); i != e; i++){
            const VSUnit *O = *i;
            OrderIdentity.push_back(O);
          }
          OrderIdentityMap.erase(it);
        }//end while
        IdenticalMap[idx] = OrderIdentity;
        OrderIdentity.clear();
    }

    addTopologyResourceConstraints(lp, IdenticalMap);
  }
}

void SDCScheduler::addTopologyResourceConstraints(lprec *lp, Step2SUMap &Map){
  BoundSUVec OrderVec;
  //Sort the operations in descending order.
  while(!Map.empty()){
    unsigned MaxIdx = 0;
    for(Step2SUMap::iterator iB = Map.begin(),eB = Map.end(); iB != eB; iB++){
        unsigned CurIdx = iB->first;
      if(CurIdx>MaxIdx)
        MaxIdx = CurIdx;
    }
    Step2SUMap::iterator it = Map.find(MaxIdx);
    BoundSUVec G = it->second;
    typedef BoundSUVec::iterator orderIt;
    for(orderIt i = G.begin(),e = G.end(); i != e; i++){
      const VSUnit *O = *i;
      OrderVec.push_back(O);
    }
    Map.erase(it);
  }

  int Col[2];
  /*const*/ REAL Val[2] = { 1.0, -1.0 };
  //Build the constraints for Dst_SU_startStep - Src_SU_startStep >= Lantency
  //that means Dst_SU and Src_SU can not be scheduled in the same step.
  const VSUnit *Dst = OrderVec.front();
  unsigned DstStartIdx = SUIdx[Dst];
  for(BoundSUVec::iterator I = llvm::next(OrderVec.begin()), E = OrderVec.end();
      I != E; I++){
    const VSUnit *Src = *I;
    unsigned SrcStartIdx = SUIdx[Src];
    Col[0] = 1 + DstStartIdx;
    Col[1] = 1 + SrcStartIdx;
    if(!add_constraintex(lp, 2, Val, Col, GE, int(Dst->getLatency())))
      report_fatal_error("SDCScheduler: Can NOT stepTopology Resource Constraints"
      " at VSUnit " + utostr_32(Src->getIdx()));
    Dst = Src;
    DstStartIdx = SrcStartIdx;
  }
  OrderVec.clear();
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
    int Indeg = U->getNumDeps();
    int Outdeg = U->getNumUses();
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
    unsigned cur = State.getStartSlot();
    unsigned j = get_var_primalresult(lp, TotalRows + Offset + 1);
    DEBUG(dbgs() << "the row is:" << TotalRows + Offset + 1
                 <<"the result is:" << j << "\n");
    unsigned shedslot = j+State.getStartSlot();
    U->scheduledTo(j+State.getStartSlot());
  }
}

bool SDCScheduler::scheduleState() {
  buildFDepHD(true);
  //DEBUG(viewGraph());
  // Ensure there is no resource conflict in critical path.
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
  addResourceConstraints(lp);
  // Turn off the add rowmode and start to solve the model.
  set_add_rowmode(lp, FALSE);
  TotalRows = get_Nrows(lp);
  //buildAXAPObject();
  buildOptimizingSlackDistributionObject();
  int result = solve(lp);
  //viewGraph();
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
  delete_lp(lp);
  SUIdx.clear();
  return true;
}

