
#include "SchedulingBase.h"
#include "llvm/Instructions.h"
#include "vtm/VInstrInfo.h"
#include "lp_solve/lp_lib.h"
#define DEBUG_TYPE "SDCdebug"
#include "llvm/Support/Debug.h"

using namespace llvm;

SDCScheduler::SDCScheduler(VSchedGraph &S)
  : SchedulingBase(S), numVars(0), numInst(0),shedCounter(1) {
}

void SDCScheduler::createLPVariables(lprec *lp) { 
  unsigned col =  0;
  // Set up the step variables.
  typedef VSchedGraph::sched_iterator it;
  for (it I = State.sched_begin(),E = State.sched_end();
    I != E; ++I) {     
      numInst++;
      col++;
      const VSUnit* iNode = *I;
      startVariableIndex[iNode] = numVars;
      endVariableIndex[iNode] = numVars + iNode->getLatency();
      // Set the name of the step variable.
      std::string SVStart = "sv" + utostr_32(iNode->getIdx()) + "start" ;
      std::string SVEnd = "sv" + utostr_32(iNode->getIdx()) + "end" ;
      set_col_name(lp, col, const_cast<char*>(SVStart.c_str()));
      numVars += (1 + iNode->getLatency());
      if(iNode->getLatency()){
        col++;
        set_col_name(lp, col, const_cast<char*>(SVEnd.c_str()));
      }
    } 
}

void SDCScheduler::addMulticycleConstraints(lprec *lp) {

  int col[2];
  REAL val[2];

  for (std::map<const VSUnit*, unsigned>::iterator i = startVariableIndex.begin(), 
	 e = startVariableIndex.end(); i != e; i++) {

    const VSUnit* iNode = i->first; 
    
    if (i->second == endVariableIndex[iNode] )
      continue; // not a multicycle instruction

    for (unsigned j = i->second + 1; j <= endVariableIndex[iNode]; j++) {
      col[0] = 1 + j;
      col[1] = 1 + (j-1);
      val[0] = 1.0;
      val[1] = -1.0;
      shedCounter++;
      if(iNode->getLatency())
        add_constraintex(lp, 2, val, col, GE, int(iNode->getLatency()-1));
      else
        add_constraintex(lp, 2, val, col, GE, 0.0);
    }
  }
}

void SDCScheduler::addDependencyConstraints(lprec *lp) {
  for (VSchedGraph::sched_iterator I = State.sched_begin(), E = State.sched_end();
    I != E; ++I) {     
      int col[2];
      REAL val[2];
      const VSUnit *in = *I;
      VSUnit *shedNodeIndex = *I;
      assert(in->isControl() && "Unexpected datapath in scheduler!");
      // First make sure the slot for each instruction is  >= 0.
      col[0] = 1+ startVariableIndex[in];
      val[0] = 1.0;
      add_constraintex(lp, 1, val, col, GE, 0.0);
      // Remember the constraint row of the schedule unit.
      schedTable[shedNodeIndex] = shedCounter;
      shedCounter++;
      // Build the constraint for Dst_SU_startStep - Src_SU_endStep >= Src_Latency.
      for (VSUnit::const_dep_iterator DI = in->dep_begin(),
        DE = in->dep_end(); DI != DE;++DI) {
          const VSUnit *depIn = *DI;
          VDEdge *Edge = DI.getEdge();
          col[0] = 1 + startVariableIndex[in];
          val[0] = 1.0;
          col[1] = 1 + endVariableIndex[depIn];
          val[1] = -1.0;
          add_constraintex(lp, 2, val, col, GE, Edge->getLatency());
          shedCounter++;
      }
    }
}

void SDCScheduler::addResourceConstraints(lprec *lp) {

  typedef std::vector<const VSUnit*> BoundSUVec;
  BoundSUVec orderVec;
  typedef std::map<unsigned, BoundSUVec> Step2SUMap;
  Step2SUMap SUMap;
  typedef std::map<FuncUnitId, BoundSUVec> FU2SUMap;
  FU2SUMap FSMap;

  //Get the VSUnits that need add into the resource constraints.
  for (VSchedGraph::sched_iterator I = State.sched_begin(), E = State.sched_end();
    I != E; ++I) {  
      const VSUnit *SV = *I; 
     if(SV->getFUType()== VFUs::MemoryBus || SV->getFUType()== VFUs::BRam
        || SV->getFUType()== VFUs::CalleeFN)
        FSMap[SV->getFUId()].push_back(SV);
  }

  //Map the VSUnits to their ASAPStep.
  for (FU2SUMap::iterator iF = FSMap.begin(), eF = FSMap.end(); iF != eF; iF++){
    BoundSUVec Set = iF->second;
    if(Set.size()<=1) continue;

    for (BoundSUVec::iterator iB = Set.begin(),eB = Set.end(); iB != eB; iB++){
      const VSUnit *V = *iB;
      SUMap[getASAPStep(V)].push_back(V);
    }

   //order the VSUnits according to the ASAPStep of each VSUnit from big to small.
    while(!SUMap.empty()){
      unsigned MaxIdx = 0;
      for (Step2SUMap::iterator iB = SUMap.begin(),eB = SUMap.end(); iB != eB; iB++){
        unsigned CurIdx = iB->first;
        if(CurIdx>MaxIdx)
          MaxIdx = CurIdx;
      }
      Step2SUMap::iterator it = SUMap.find(MaxIdx);
      BoundSUVec G = it->second;
      typedef BoundSUVec::iterator orderIt;
      for(orderIt i = G.begin(),e = G.end(); i != e; i++){
        const VSUnit *O = *i;
        orderVec.push_back(O);
      }
      SUMap.erase(it);
    }
    SUMap.clear();

    //Build the constraints for Dst_SU_startStep - Src_SU_startStep >= 1
    //that means Dst_SU and Src_SU can not be scheduled in the same step.
    BoundSUVec::iterator i = orderVec.begin();
    const VSUnit *front =*i;
    for (BoundSUVec::iterator OVB = ++i,OVE = orderVec.end();
         OVB != OVE; OVB++){
      int col[2];
      REAL val[2];
      const VSUnit *back = *OVB;
      col[0] = 1 + startVariableIndex[front];
      val[0] = 1.0;
      col[1] = 1 + startVariableIndex[back];
      val[1] = -1.0;
      add_constraintex(lp, 2, val, col, GE, 1.0);
      front = back;
    }
    orderVec.clear();
  }

  FSMap.clear();
  return;
}

void SDCScheduler::buildObject(bool ASAP) {
  int *variableIndices = new int[numInst];
  REAL *variableCoefficients = new REAL[numInst];

  int count = 0;
  //Build the AXAP object function.
  for (std::map<const VSUnit*, unsigned>::iterator i = startVariableIndex.begin(), 
    e = startVariableIndex.end(); i != e; i++) {

      unsigned varIndex = i->second;
      variableIndices[count] = 1 + varIndex;
      variableCoefficients[count] = 1.0;
      count++;

  }

  assert(count == numInst);  

  set_obj_fnex(lp, count, variableCoefficients, variableIndices);
  if (ASAP)
    set_minim(lp);
  else
    set_maxim(lp);

  DEBUG(write_lp(lp, "log.lp"));

  int ret = solve(lp);

  delete [] variableCoefficients;
  delete [] variableIndices;

}

void SDCScheduler::buildSchedule(lprec *lp) {
  for (std::map<VSUnit*, unsigned>::iterator i = schedTable.begin(),
    e = schedTable.end(); i != e; i++) {
    VSUnit *U = i->first;
    unsigned idx = U->getIdx();
    unsigned row = i->second;
    unsigned j = get_var_primalresult(lp, row) ;
    unsigned shedslot = j+State.getStartSlot();
    U->scheduledTo(j+State.getStartSlot());
  }
}


bool SDCScheduler::scheduleState() {
   buildFDepHD(true);
  // Ensure there is no resource conflict in critical path.
  if (!scheduleCriticalPath(false))
    return false;

  if (allNodesSchedued()) return true;
  
  //viewGraph();
  lp = make_lp(0, numVars);

  set_add_rowmode(lp, TRUE);

  // Build the step variables.
  createLPVariables(lp);

  // Build the constraints.
  addMulticycleConstraints(lp);
  addDependencyConstraints(lp);
  addResourceConstraints(lp);
  // Turn off the add rowmode and start to solve the model.
  set_add_rowmode(lp, FALSE);
  buildObject(true);
  // Schedule the state with the ILP result.
  buildSchedule(lp);
  viewGraph();
  delete_lp(lp);
  schedTable.clear();
  return true;
}

