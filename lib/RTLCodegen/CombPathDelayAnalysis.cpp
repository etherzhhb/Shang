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
struct TimingPath {
  int Delay;
  VASTValue **Path;
  unsigned PathSize;

  void bindPath2ScriptEngine();
};

struct CombPathDelayAnalysis : public MachineFunctionPass {
  CFGShortestPath *CFGSP;
  RtlSSAAnalysis *RtlSSA;
  BumpPtrAllocator Allocator;

  static char ID;

  void getAnalysisUsage(AnalysisUsage &AU) const {
    MachineFunctionPass::getAnalysisUsage(AU);
    AU.addRequired<TargetData>();
    AU.addRequired<CFGShortestPath>();
    AU.addRequired<RtlSSAAnalysis>();
    AU.setPreservesAll();
  }

  void writeConstraintsForDstReg(VASTRegister *DstReg);

  void extractTimingPaths(ValueAtSlot *DstVAS, VASTUse DepTree,
                          SmallVectorImpl<TimingPath*> &Paths);

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

  TimingPath *createTimingPath(ValueAtSlot *Dst, ArrayRef<VASTUse> Path);
};
}

// The first node of the path is the use node and the last node of the path is
// the define node.
void TimingPath::bindPath2ScriptEngine() {
  assert(PathSize >= 2 && "Path vector have less than 2 nodes!");
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
  SS << "RTLDatapath.Slack = " << Delay;
  SS.flush();
  if (!runScriptStr(Script, Err))
    llvm_unreachable("Cannot create slack of RTLDatapath!");

  Script.clear();

  SS << "RTLDatapath.Nodes = {'" << Path[0]->getName();
  for (unsigned i = 1; i < PathSize; ++i) {
    // Skip the unnamed nodes.
    const char *Name = Path[i]->getName();
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

// Helper class
struct TimgPathBuilder {
  CombPathDelayAnalysis &A;
  ValueAtSlot *DstVAS;
  SmallVectorImpl<TimingPath*> &Paths;

  TimgPathBuilder(CombPathDelayAnalysis &a, ValueAtSlot *V,
                  SmallVectorImpl<TimingPath*> &P) : A(a), DstVAS(V), Paths(P){}

  void operator() (ArrayRef<VASTUse> PathArray) {
    VASTUse SrcUse = PathArray.back();
    if (VASTRegister *Src = dyn_cast_or_null<VASTRegister>(SrcUse.getOrNull()))
      Paths.push_back(A.createTimingPath(DstVAS, PathArray));
  }
};

TimingPath *CombPathDelayAnalysis::createTimingPath(ValueAtSlot *Dst,
                                                    ArrayRef<VASTUse> Path) {
  TimingPath *P = new (Allocator.Allocate<TimingPath>()) TimingPath();
  VASTSlot *DstSlot = Dst->getSlot();
  P->Delay = 0;

  // The Path should include the Dst.
  P->PathSize = Path.size() + 1;
  P->Path = Allocator.Allocate<VASTValue*>(P->PathSize);

  int BlockBoxesDelay = 0;

  P->Path[0] = Dst->getValue();
  for (unsigned i = 0; i < Path.size(); ++i) {
    VASTValue *V = Path[i].get();
    P->Path[i + 1] = V;

    // Accumulates the block box latency.
    if (VASTWire *W = dyn_cast<VASTWire>(V))
      if (W->getOpcode() == VASTWire::dpVarLatBB)
        BlockBoxesDelay += W->getLatency();
  }

  // Add the end slots.
  VASTRegister *SrcReg = cast<VASTRegister>(Path.back().get());

  int PathDelay = CFGShortestPath::Infinite;

  typedef VASTRegister::assign_itertor assign_it;
  for (assign_it I = SrcReg->assign_begin(), E = SrcReg->assign_end();
       I != E; ++I) {
    VASTSlot *SrcSlot = I->first->getSlot();
    ValueAtSlot *SrcVAS = RtlSSA->getValueASlot(SrcReg, SrcSlot);

    // Update the PathDelay if the source VAS reaches DstSlot.
    if (Dst->isDependOn(SrcVAS)) {
      int D = CFGSP->getSlotDistance(SrcSlot, DstSlot);
      // Note that getSlotDistance is possible, because the define may reach
      // the dst slot via a non-shortest path. And getSlotDistance returns a
      // shortest distance so the result maybe invalid.
      assert(D >= 0 && "Reaching define reaches an unreachable slot?");
      if (D) PathDelay = std::min(PathDelay, D);
    }
  }

  assert(PathDelay != CFGShortestPath::Infinite && "All path invalid?");
  // The path delay is the sum of minimum distance between source/destinate slot
  // and the delay of block boxes.
  P->Delay = PathDelay + BlockBoxesDelay;

  return P;
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

static bool path_delay_is_bigger(const TimingPath *LHS, const TimingPath *RHS) {
  return LHS > RHS;
}

void CombPathDelayAnalysis::writeConstraintsForDstReg(VASTRegister *DstReg) {
  SmallVector<TimingPath*, 8> Paths;

  typedef VASTRegister::assign_itertor assign_it;
  for (assign_it I = DstReg->assign_begin(), E = DstReg->assign_end();
       I != E; ++I) {
    VASTSlot *S = I->first->getSlot();
    ValueAtSlot *DstVAS = RtlSSA->getValueASlot(DstReg, S);

    // Paths for the assigning value
    extractTimingPaths(DstVAS, I->first, Paths);
    // Paths for the condition.
    extractTimingPaths(DstVAS, *I->second, Paths);
  }

  // Sort the delay so we write big delay constraints first, if the loose
  // constraint is not applied, we may always apply the tighter constraint,
  // so the design can synthesized correctly.
  std::sort(Paths.begin(), Paths.end(), path_delay_is_bigger);

  typedef SmallVector<TimingPath*, 8>::iterator path_it;
  for (path_it DI = Paths.begin(), DE = Paths.end(); DI != DE; ++DI)
    (*DI)->bindPath2ScriptEngine();

  Allocator.Reset();
}

void CombPathDelayAnalysis::extractTimingPaths(ValueAtSlot *DstVAS,
                                               VASTUse DepTree,
                                               SmallVectorImpl<TimingPath*>
                                               &Paths) {
  VASTValue *SrcValue = DepTree.getOrNull();

  // If Define Value is immediate or symbol, skip it.
  if (!SrcValue) return;

  // Trivial case: register to register path.
  if (VASTRegister *SrcReg = dyn_cast<VASTRegister>(SrcValue)){
    VASTUse Path[] = { VASTUse(SrcReg) };
    Paths.push_back(createTimingPath(DstVAS, Path));
    return;
  }

  TimgPathBuilder B(*this, DstVAS, Paths);
  DepthFirstTraverseDepTree(DepTree, B);
}

char CombPathDelayAnalysis::ID = 0;
INITIALIZE_PASS_BEGIN(CombPathDelayAnalysis, "CombPathDelayAnalysis",
                      "CombPathDelayAnalysis", false, false)
INITIALIZE_PASS_END(CombPathDelayAnalysis, "CombPathDelayAnalysis",
                    "CombPathDelayAnalysis", false, false)
Pass *llvm::createCombPathDelayAnalysisPass() {
  return new CombPathDelayAnalysis();
}
