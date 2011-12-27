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

#include "vtm/VerilogAST.h"
#include "vtm/Passes.h"
#include "vtm/VFInfo.h"
#include "vtm/Utilities.h"
#define DEBUG_TYPE "CombPathDelayAnalysis"
#include "llvm/Support/Debug.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/ADT/DenseMap.h"
#include "FindMBBShortestPath.h"
#include "llvm/Support/SourceMgr.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

namespace{
class CombPathDelayAnalysis : public MachineFunctionPass {
  MachineFunction *MF;
  FindShortestPath *FindSP;

  // define a DenseMap to record the Slack Info between Registers.
  typedef std::pair<VASTRegister*, VASTRegister*> RegisterMapTy;
  typedef DenseMap<RegisterMapTy, unsigned> RegPathDelayTy;
  RegPathDelayTy RegPathDelay;

  // Initial the slack between the registers.
  void InitRegPath(VASTModule *VM);

  // Compute the Path Slack between two register.
  void computePathSlack(VASTRegister* UseReg);

  // Get Slack through Define register.
  void computeSlackThrough(VASTUse DefUse, VASTRegister* UseReg, VASTSlot *UseSlot);

  // get nearest Define slot Distance.
  unsigned getNearestSlotDistance(VASTRegister *DefReg, VASTSlot *UseSlot);

  void DepthFristTraverseDataPathUseTree(VASTUse DefUse,
                                         VASTRegister* UseReg,
                                         VASTSlot *UseSlot);

public:
  static char ID;

  // get the slack of Combination Path between two registers.
  unsigned getCombPathSlack(VASTRegister *DefReg, VASTRegister *UseReg);

  void updateCombPathSlack(VASTRegister *Def, VASTRegister *Use,
                           unsigned NewSlack) {
    assert(RegPathDelay.count(std::make_pair(Def, Use)) && "Pair not exist!");
    RegPathDelayTy::value_type &v =
      RegPathDelay.FindAndConstruct(std::make_pair(Def, Use));
    v.second = std::min(v.second, NewSlack);
  }

  void getAnalysisUsage(AnalysisUsage &AU) const {
    MachineFunctionPass::getAnalysisUsage(AU);
    AU.addRequired<TargetData>();
    AU.addRequired<FindShortestPath>();
    AU.addPreserved<FindShortestPath>();
  }

  bool runOnMachineFunction(MachineFunction &MF);

  void releaseMemory() {
    RegPathDelay.clear();
  }

  bool doInitialization(Module &) {
    SMDiagnostic Err;
    // Get the script from script engine.
    const char *HeaderScriptPath[] = { "Misc",
                                       "TimingConstraintsHeaderScript" };
    if (!runScriptStr(getStrValueFromEngine(HeaderScriptPath), Err))
      report_fatal_error("Error occur while running timing header script:\n"
                         + Err.getMessage());
  }

  CombPathDelayAnalysis() : MachineFunctionPass(ID) {
    initializeCombPathDelayAnalysisPass(*PassRegistry::getPassRegistry());
  }
};
}

bool CombPathDelayAnalysis::runOnMachineFunction(MachineFunction &F) {
  bindFunctionInfoToScriptEngine(F, getAnalysis<TargetData>());
  MF = &F;
  VASTModule *VM = MF->getInfo<VFInfo>()->getRtlMod();
  FindSP = &getAnalysis<FindShortestPath>();
  InitRegPath(VM);
  return false;
}

static void bindPath2ScriptEngine(ArrayRef<VASTRegister*> Path,
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

  SS << "RTLDatapath.Nodes = {'" << Path[0]->getName();
  for (unsigned i = 1; i < Path.size(); ++i) {
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

void CombPathDelayAnalysis::InitRegPath(VASTModule *VM) {
  //Initial all the path with infinite.
  for (VASTModule::reg_iterator I = VM->reg_begin(), E = VM->reg_end();
       I != E; ++I){
    VASTRegister *DefReg = *I;
    for (VASTModule::reg_iterator I = VM->reg_begin(), E = VM->reg_end();
         I != E; ++I){
      VASTRegister *UseReg = *I;
      RegPathDelay[std::make_pair(DefReg, UseReg)] = FindShortestPath::Infinite;
    }
  }

  //Assign the path delay to path between two register.
  for (VASTModule::reg_iterator I = VM->reg_begin(), E = VM->reg_end();
       I != E; ++I) {
    VASTRegister *UseReg = *I;
    computePathSlack(UseReg);
  }

  typedef RegPathDelayTy::const_iterator RegPairIt;
  for (RegPairIt I = RegPathDelay.begin(), E = RegPathDelay.end(); I != E; ++I){
    unsigned Slack = I->second;
    if (Slack != FindShortestPath::Infinite) {
      VASTRegister* Path[] = { (I->first).first, (I->first).second };
      bindPath2ScriptEngine(Path, Slack);
    }
  }
}

void CombPathDelayAnalysis::computePathSlack(VASTRegister* UseReg) {
  typedef DenseMap<VASTWire*, VASTUse*> AssignMapTy;
  AssignMapTy Assigns = UseReg->getAssignments();
  // Do we have any assignment information?
  if (Assigns.empty()) return;

  for (AssignMapTy::const_iterator I = Assigns.begin(), E = Assigns.end();
       I != E; ++I) {
    VASTUse &Def = *I->second;
    VASTWire *AssignCnds = I->first;
    VASTSlot *UseSlot = AssignCnds->getSlot();

    //Get Slack from the Define Value.
    computeSlackThrough(Def, UseReg, UseSlot);

    //Get Slack from the condition of the assignment.
    typedef VASTWire::op_iterator it;
    for (it I = AssignCnds->op_begin(), E = AssignCnds->op_end(); I != E; ++I) {
      computeSlackThrough(*I, UseReg, UseSlot);
    }
  }
}

void CombPathDelayAnalysis::computeSlackThrough(VASTUse DefUse,
                                                VASTRegister *UseReg,
                                                VASTSlot *UseSlot) {
  VASTValue *DefValue = DefUse.getOrNull();

  // If Define Value is immediate or symbol, skip it.
  if (!DefValue) return;

  if (VASTRegister *DefReg = dyn_cast<VASTRegister>(DefValue)) {
    unsigned Slack = getNearestSlotDistance(DefReg, UseSlot);
    // If the Define register and Use register already have a slack, compare the
    // slacks and assign the smaller one to the RegPathDelay Map.
    updateCombPathSlack(DefReg, UseReg, Slack);
    return;
  }

  DepthFristTraverseDataPathUseTree(DefUse, UseReg, UseSlot);
}

unsigned CombPathDelayAnalysis::getNearestSlotDistance(VASTRegister *DefReg,
                                                       VASTSlot *UseSlot) {

  int NearestSlotDistance = FindShortestPath::Infinite;
  typedef std::set<VASTSlot*, less_ptr<VASTSlot> >::const_iterator SlotIt;

  // FIXME: We can perform a binary search.
  for (SlotIt I = DefReg->slots_begin(), E = DefReg->slots_end(); I != E; ++I) {
    VASTSlot *DefSlot = *I;
    int SlotDistance = FindSP->getSlotDistance(DefSlot, UseSlot);

    // if SlotDistance == 0, abandon this result.
    if (SlotDistance <= 0) continue;

    if (SlotDistance < NearestSlotDistance) {
      NearestSlotDistance = SlotDistance;
    }
  }

  return NearestSlotDistance;
}

// Traverse the use tree in datapath, stop when we meet a register or other
// leaf node.
void
  CombPathDelayAnalysis::DepthFristTraverseDataPathUseTree(VASTUse DefUse,
                                                           VASTRegister* UseReg,
                                                           VASTSlot *UseSlot) {
  typedef VASTUse::iterator ChildIt;
  // Use seperate node and iterator stack, so we can get the path vector.
  typedef SmallVector<VASTUse, 16> NodeStackTy;
  typedef SmallVector<ChildIt, 16> ItStackTy;
  NodeStackTy NodeWorkStack;
  ItStackTy ItWorkStack;
  // Remember what we had visited.
  std::set<VASTUse> VisitedUses;

  // Put the current node into the node stack, so it will appears in the path.
  NodeWorkStack.push_back(UseReg);

  // Put the root.
  NodeWorkStack.push_back(DefUse);
  ItWorkStack.push_back(DefUse.dp_src_begin());

  while (!ItWorkStack.empty()) {
    VASTUse Node = NodeWorkStack.back();
    ChildIt It = ItWorkStack.back();

    // Do we reach the leaf?
    if (Node.is_dp_leaf()) {
      if (VASTValue *V = Node.getOrNull()) {
        DEBUG_WITH_TYPE("rtl-slack-info",
          dbgs() << "Datapath:\t";
        for (NodeStackTy::iterator I = NodeWorkStack.begin(),
          E = NodeWorkStack.end(); I != E; ++I) {
            dbgs() << ", ";
            I->print(dbgs());
        });

        if (VASTRegister *R = dyn_cast<VASTRegister>(V)) {
          unsigned Slack = 0;
          // Data-path has timing information available.
          for (NodeStackTy::iterator I = NodeWorkStack.begin(),
            E = NodeWorkStack.end(); I != E; ++I)
            if (VASTWire *W = dyn_cast<VASTWire>(I->get()))
              if (W->getOpcode() == VASTWire::dpVarLatBB)
                Slack += W->getLatency();

          Slack = std::max(Slack, getNearestSlotDistance(R, UseSlot));

          DEBUG_WITH_TYPE("rtl-slack-info",
            dbgs() << " Slack: " << int(Slack));

          // If the Define register and Use register already have a slack,
          // compare the slacks and assign the smaller one to the RegPathDelay
          // Map.
          updateCombPathSlack(R, UseReg, Slack);
        }

        DEBUG_WITH_TYPE("rtl-slack-info", dbgs() << '\n');
      }

      NodeWorkStack.pop_back();
      ItWorkStack.pop_back();
      continue;
    }

    // All sources of this node is visited.
    if (It == Node.dp_src_end()) {
      NodeWorkStack.pop_back();
      ItWorkStack.pop_back();
      continue;
    }

    // Depth first traverse the child of current node.
    VASTUse ChildNode = *It;
    ++ItWorkStack.back();

    // Had we visited this node?
    if (!VisitedUses.insert(ChildNode).second) continue;

    // If ChildNode is not visit, go on visit it and its childrens.
    NodeWorkStack.push_back(ChildNode);
    ItWorkStack.push_back(ChildNode.dp_src_begin());
  }

  assert(NodeWorkStack.back().get() == UseReg && "Node stack broken!");
}

unsigned CombPathDelayAnalysis::getCombPathSlack(VASTRegister *Def,
                                                 VASTRegister *Use) {
  RegPathDelayTy::iterator at = RegPathDelay.find(std::make_pair(Def, Use));
  assert(at != RegPathDelay.end() && "Cannot found pair!");
  return at->second;
}

char CombPathDelayAnalysis::ID = 0;
INITIALIZE_PASS_BEGIN(CombPathDelayAnalysis, "CombPathDelayAnalysis",
                      "CombPathDelayAnalysis", false, false)
INITIALIZE_PASS_END(CombPathDelayAnalysis, "CombPathDelayAnalysis",
                    "CombPathDelayAnalysis", false, false)
Pass *llvm::createCombPathDelayAnalysisPass() {
  return new CombPathDelayAnalysis();
}