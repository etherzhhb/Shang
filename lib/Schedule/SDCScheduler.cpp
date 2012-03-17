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
      Val[0] = -1.0;
      Col[1] = 1 + DstStartIdx;
      Val[1] = 1.0;
      if(!add_constraintex(lp, 2, Val, Col, GE, 0.0))
        report_fatal_error("SDCScheduler: Can NOT step Dependency Constraints"
        " at VSUnit " + utostr_32(U->getIdx()) );   
    }
  }
}

void SDCScheduler::PreBind() {
  //The table of the ALAP and the VSUnits.
  Step2SUMap IdenticalMap;
  BoundSUVec OrderIdentity;
  BoundSUVec OrderVec;
  Step2SUMap FUMap;
  std::set<unsigned> SlotSet;
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
  
  for( Step2SUMap::iterator FI = FUMap.begin(), FE = FUMap.end(); 
    FI != FE; FI++){
    bool MemType = false;
    unsigned FuType = FI->first;
    DEBUG(dbgs()<<"The FU Type is :"<<FuType<<"\n");

    if(FuType == VFUs::BRam || FuType == VFUs::MemoryBus || FuType == VFUs::CalleeFN)
      MemType = true;

    BoundSUVec Set = FI->second;
    if(Set.size()<=1) continue;
   
    //Map the VSUnits to their ALAPStep.
    for(BoundSUVec::iterator iB = Set.begin(),eB = Set.end(); iB != eB; iB++){
      const VSUnit *V = *iB;
      for(unsigned i = getASAPStep(V), j = getALAPStep(V); i <= j; i++)
        SlotSet.insert(i);
      IdenticalMap[getALAPStep(V)].push_back(V);
    }

    //Sort the IdenticalMap in descending order of the TimeFrame.
    //The TimeFrame means the freedom of the VSUnits in scheduling. 
    //SDCScheduler prefer to move the VSUnits that have the bigger TimeFrame 
    //to next slot.
    unsigned CriticalFUNum = 0;
    for(Step2SUMap::iterator IS = IdenticalMap.begin(), ES = IdenticalMap.end();
      IS != ES; IS++){
        BoundSUVec V = IS->second;
        unsigned idx = IS->first;

        if(V.size()<=1) continue;      
        for(BoundSUVec::iterator iU = V.begin(),eU = V.end(); iU != eU; iU++){
          const VSUnit* U = *iU;
          OrderIdentityMap [getTimeFrame(U)].push_back(U);
        }

        //-------------------------------------------//
        unsigned Num = OrderIdentityMap[1].size();
        if(Num > CriticalFUNum)
          CriticalFUNum = Num;
        //-------------------------------------------//
        while(!OrderIdentityMap.empty()){
          unsigned MinTimeFrame = FindMaxOrMinValue(OrderIdentityMap, 100, false);
          Step2SUMap::iterator it = OrderIdentityMap.find(MinTimeFrame);
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

    //Compute the average number of the FU.
    double AverageRC = double(Set.size())/double(SlotSet.size());
    unsigned AverageNum = std::max<unsigned>(ceil(AverageRC), CriticalFUNum);
    DEBUG(dbgs()<<"the average resource is :"<<AverageNum<<"\n");

    if(MemType) AverageNum =1;

    ////Find the operations that have the same inputs.
    //for(Step2SUMap::iterator II = IdenticalMap.begin(), EI =IdenticalMap.end();
    //    II != EI; II++){
    //  BoundSUVec BV = II->second;
    //  if(BV.size() > 1){
    //    for(BoundSUVec::iterator iU = BV.begin(),eU = BV.end(); iU != eU; iU++){
    //      const VSUnit* U = *iU;
    //      BoundSUVec::iterator Cur = iU;
    //      ++Cur;
    //      BoundSUVec::iterator Next = Cur;
    //      if(Next == BV.end()) continue;
    //      const VSUnit* V = *Next;
    //      if(U->getIdx() != V->getIdx()){
    //        if(hasCommonInput(U,V)){
    //        CommonPair = std::make_pair(U,V);
    //        CommonVec.push_back(CommonPair);
    //      }// end if
    //    }//end for
    //  }
    //  }
    //}

    //List Scheduling
    while(!IdenticalMap.empty()){
      unsigned MaxALAP = FindMaxOrMinValue(IdenticalMap, 0, true);
      Step2SUMap::iterator it = IdenticalMap.find(MaxALAP);
      unsigned CurSlot = it->first;
      BoundSUVec BV = it->second;
      const VSUnit* U = *BV.begin();
      unsigned MoveSlot = CurSlot - U->getLatency();
      OrderVec.push_back(U);

      if(BV.size() > AverageNum){
        unsigned counter = 0;
        BoundSUVec::reverse_iterator rit = BV.rbegin();
        for(unsigned i = 0 , j = BV.size() - AverageNum; i < j; i++){
          const VSUnit* MU = *rit;
          unsigned Slot = getASAPStep(MU);
          if(Slot < CurSlot){
            IdenticalMap[MoveSlot].push_back(MU);
            ++counter;
          }
          ++rit;
        }

        //Sort the Inserted Vector in ASAP descending order
        if(counter){
          BoundSUVec Vec = IdenticalMap[MoveSlot];
          for(BoundSUVec::iterator I = Vec.begin(), E = Vec.end(); I != E; I++){
            const VSUnit* V = *I;
            OrderIdentityMap[getASAPStep(V)].push_back(V);
          }
          while(!OrderIdentityMap.empty()){
            unsigned MaxSlot = FindMaxOrMinValue(OrderIdentityMap, 0, true);
            Step2SUMap::iterator IT = OrderIdentityMap.find(MaxSlot);
            BoundSUVec B = IT->second;
            typedef BoundSUVec::iterator OrderIt;
            for(OrderIt i = B.begin(),e = B.end(); i != e; i++){
              const VSUnit *O = *i;
              OrderIdentity.push_back(O);
            }
            OrderIdentityMap.erase(IT);
          }//end while
          IdenticalMap[MoveSlot] = OrderIdentity;
          OrderIdentity.clear();
        }
      }//end if
      IdenticalMap.erase(it);
    }//end while

    // Build the Control Edges.
    for(unsigned i = 0,e = OrderVec.size(); i < e; i++){
      VSUnit* Dst = const_cast<VSUnit*>(OrderVec[i]);
      unsigned next = i + 1;
      if(next >= OrderVec.size()) continue;
      VSUnit* Src = const_cast<VSUnit*>(OrderVec[next]);
      if(isOverlap(Dst,Src))
        Dst->addDep(VDCtrlDep::CreateCtrlDep(Src, Src->getLatency()));
    }

    OrderVec.clear();
  }   
}

unsigned SDCScheduler::FindMaxOrMinValue(Step2SUMap &Map, unsigned var, bool Max){
  unsigned Idx = var;
  for(Step2SUMap::iterator iB = Map.begin(),eB = Map.end(); iB != eB; iB++){
    unsigned CurIdx = iB->first;
    if(Max){
      if(CurIdx > Idx)
        Idx = CurIdx;
    }
    else{
      if(CurIdx < Idx)
        Idx = CurIdx;
    }
  }
  return Idx;
}

unsigned SDCScheduler::getComInNum(const VSUnit* Src, const VSUnit* Dst){
  std::set<const VSUnit*> DepSet;
  unsigned DepCounter = 0;
  for(VSUnit::const_dep_iterator iS = Src->dep_begin(), eS = Src->dep_end();
      iS != eS; iS++){
    if(iS.getEdge()->getEdgeType() != VDEdge::edgeValDep) continue;
    const VSUnit* U = *iS;
    DepSet.insert(U);
    ++DepCounter;
  }

  for(VSUnit::const_dep_iterator iD = Dst->dep_begin(), eD = Dst->dep_end();
      iD != eD; iD++){
    if(iD.getEdge()->getEdgeType() != VDEdge::edgeValDep) continue;
    const VSUnit* U = *iD;
    DepSet.insert(U);
    ++DepCounter;
  }

  return DepCounter - DepSet.size();
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
  //Ensure there is no resource conflict in critical path.
  if (!scheduleCriticalPath(false))
    return false;

  //Avoid the resources conflict for the function units.
  PreBind();

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
  //buildAXAPObject();
  buildOptimizingSlackDistributionObject();
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

