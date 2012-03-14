//CombPathDelayAnalysis.cpp- Analysis the Path delay between registers- C++ -=//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This pass analysis the slack between two registers.
//
// The "Slack" in VAST means the extra cycles that after data appear in
// the output pin of the src register before the dst register read the data.
// i.e. if we assign reg0 at cycle 1, and the data will appear at the output
// pin of reg0 at cycle 2, and now reg1 can read the data. In this case
// becasue the data appear at cycle 2 and we read the data at the same cycle,
// the slack is 0. But if we read the data at cycle 3, the slack is 1.
//
//===----------------------------------------------------------------------===//

#include "CFGShortestPath.h"
#include "RtlSSAAnalysis.h"

#include "vtm/VerilogAST.h"
#include "vtm/Passes.h"
#include "vtm/VFInfo.h"
#include "vtm/Utilities.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/Target/TargetData.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/Support/SourceMgr.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/CommandLine.h"
#define DEBUG_TYPE "vtm-comb-path-delay"
#include "llvm/Support/Debug.h"
using namespace llvm;

static cl::opt<bool>
DisableTimingScriptGeneration("vtm-disable-timing-script",
                              cl::desc("Disable timing script generation"),
                              cl::init(false));

namespace{
struct CombPathDelayAnalysis : public MachineFunctionPass {
  CFGShortestPath *CFGSP;
  RtlSSAAnalysis *RtlSSA;

  static char ID;

  void getAnalysisUsage(AnalysisUsage &AU) const {
    MachineFunctionPass::getAnalysisUsage(AU);
    AU.addRequired<TargetData>();
    AU.addRequired<CFGShortestPath>();
    AU.addRequired<RtlSSAAnalysis>();
    AU.setPreservesAll();
  }

  void writeConstraintsForDstReg(VASTRegister *DstReg);

  void extractTimingPathsFor(ValueAtSlot *DstVAS,
                             DenseMap<VASTValue*, int> &Distances);

  bool runOnMachineFunction(MachineFunction &MF);

  bool doInitialization(Module &) {
    SMDiagnostic Err;
    // Get the script from script engine.
    const char *HeaderScriptPath[] = { "Misc",
                                       "TimingConstraintsHeaderScript" };
    if (!runScriptStr(getStrValueFromEngine(HeaderScriptPath), Err))
      report_fatal_error("Error occur while running timing header script:\n"
                         + Err.getMessage());
    return false;
  }

  CombPathDelayAnalysis() : MachineFunctionPass(ID) {
    initializeCombPathDelayAnalysisPass(*PassRegistry::getPassRegistry());
  }
};
}

// The first node of the path is the use node and the last node of the path is
// the define node.
static void bindPath2ScriptEngine(ArrayRef<VASTUse> Path,
                                  unsigned Slack) {
  assert(Path.size() >= 2 && "Path vector have less than 2 nodes!");
  // Path table:
  // Datapath: {
  //  unsigned Slack,
  //  table NodesInPath
  // }
  SMDiagnostic Err;

  if (!runScriptStr("RTLDatapath = {}\n", Err))
    llvm_unreachable("Cannot create RTLDatapath table in scripting pass!");

  std::string Script;
  raw_string_ostream SS(Script);
  SS << "RTLDatapath.Slack = " << Slack;
  SS.flush();
  if (!runScriptStr(Script, Err))
    llvm_unreachable("Cannot create slack of RTLDatapath!");

  Script.clear();

  SS << "RTLDatapath.Nodes = {'" << Path[0].get()->getName();
  for (unsigned i = 1; i < Path.size(); ++i) {
    // Skip the unnamed nodes.
    const char *Name = Path[i].get()->getName();
    if (Name) SS << "', '" << Name;
  }
  SS << "'}";

  SS.flush();
  if (!runScriptStr(Script, Err))
    llvm_unreachable("Cannot create node table of RTLDatapath!");

  // Get the script from script engine.
  const char *DatapathScriptPath[] = { "Misc", "DatapathScript" };
  if (!runScriptStr(getStrValueFromEngine(DatapathScriptPath), Err))
    report_fatal_error("Error occur while running datapath script:\n"
                       + Err.getMessage());
}

bool CombPathDelayAnalysis::runOnMachineFunction(MachineFunction &MF) {
  // No need to write timing script at all.
  if (DisableTimingScriptGeneration) return false;

  bindFunctionInfoToScriptEngine(MF, getAnalysis<TargetData>());
  VASTModule *VM = MF.getInfo<VFInfo>()->getRtlMod();
  CFGSP = &getAnalysis<CFGShortestPath>();
  RtlSSA = &getAnalysis<RtlSSAAnalysis>();

  //Write the timing constraints.
  typedef VASTModule::reg_iterator reg_it;
  for (reg_it I = VM->reg_begin(), E = VM->reg_end(); I != E; ++I)
    writeConstraintsForDstReg(*I);

  return false;
}

void CombPathDelayAnalysis::writeConstraintsForDstReg(VASTRegister *DstReg) {
  DenseMap<VASTValue*, int> DistanceMap;

  typedef VASTRegister::assign_itertor assign_it;
  for (assign_it I = DstReg->assign_begin(), E = DstReg->assign_end();
       I != E; ++I) {
    VASTSlot *S = I->first->getSlot();
    ValueAtSlot *DstVAS = RtlSSA->getValueASlot(DstReg, S);

    extractTimingPathsFor(DstVAS, DistanceMap);
  }

  typedef DenseMap<VASTValue*, int>::iterator distance_it;
  for (distance_it DI = DistanceMap.begin(), DE = DistanceMap.end();
       DI != DE; ++DI) {
    // Path from Src to Dst
    VASTUse Path[] = { DstReg, DI->first };
    bindPath2ScriptEngine(Path, DI->second);
  }
}

void CombPathDelayAnalysis::extractTimingPathsFor(ValueAtSlot *DstVAS,
                                                  DenseMap<VASTValue*, int>
                                                  &Distances) {
  typedef ValueAtSlot::iterator dep_it;
  for (dep_it DI = DstVAS->dep_begin(), DE = DstVAS->dep_end();DI != DE;++DI){
    ValueAtSlot *SrcVAS = *DI;

    int D = CFGSP->getSlotDistance(SrcVAS->getSlot(), DstVAS->getSlot());
    assert(D > 0 && "SrcSlot should able to reach DstSlot!");
    int &Distance = Distances[SrcVAS->getValue()];

    // Update the distance.
    if (Distance == 0 || Distance > D) Distance = D;
  }
}

char CombPathDelayAnalysis::ID = 0;
INITIALIZE_PASS_BEGIN(CombPathDelayAnalysis, "CombPathDelayAnalysis",
                      "CombPathDelayAnalysis", false, false)
INITIALIZE_PASS_END(CombPathDelayAnalysis, "CombPathDelayAnalysis",
                    "CombPathDelayAnalysis", false, false)
Pass *llvm::createCombPathDelayAnalysisPass() {
  return new CombPathDelayAnalysis();
}
