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

#include "FindMBBShortestPath.h"
#include "vtm/VerilogAST.h"
#include "vtm/Passes.h"
#include "vtm/VFInfo.h"
#include "vtm/Utilities.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/Support/SourceMgr.h"
#include "llvm/Support/ErrorHandling.h"

#define DEBUG_TYPE "vtm-comb-path-delay"
#include "llvm/Support/Debug.h"
using namespace llvm;

namespace{
class CombPathDelayAnalysis : public MachineFunctionPass {
  FindShortestPath *FindSP;

  // Define a DenseMap to record the Slack Info between Registers.
  typedef std::pair<VASTRegister*, VASTRegister*> RegisterMapTy;
  typedef DenseMap<RegisterMapTy, unsigned> RegPathDelayTy;
  RegPathDelayTy RegPathDelay;

  typedef SmallVector<VASTSlot*, 4> SlotVec;

  // Compute the Path Slack between two register.
  void computePathSlack(VASTRegister* UseReg);

  // Get Slack through Define register.
  void computeSlackThrough(VASTUse DefUse, VASTRegister* UseReg,
                           SlotVec &UseSlots);

  // get nearest Define slot Distance.
  unsigned getNearestSlotDistance(VASTRegister *DefReg, SlotVec &UseSlots);

  void DepthFristTraverseDataPathUseTree(VASTUse DefUse,
                                         VASTRegister *UseReg,
                                         SlotVec &UseSlots);

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
    return false;
  }

  CombPathDelayAnalysis() : MachineFunctionPass(ID) {
    initializeCombPathDelayAnalysisPass(*PassRegistry::getPassRegistry());
  }
};
}

// The first node of the path is the use node and the last node of the path is
// the define node.
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

bool CombPathDelayAnalysis::runOnMachineFunction(MachineFunction &MF) {
  bindFunctionInfoToScriptEngine(MF, getAnalysis<TargetData>());
  VASTModule *VM = MF.getInfo<VFInfo>()->getRtlMod();
  FindSP = &getAnalysis<FindShortestPath>();

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
      VASTRegister* Path[] = { (I->first).second, (I->first).first };
      bindPath2ScriptEngine(Path, Slack);
    }
  }

  return false;
}

void CombPathDelayAnalysis::computePathSlack(VASTRegister* UseReg) {
  typedef DenseMap<VASTWire*, VASTUse*> AssignMapTy;

  typedef std::map<VASTUse, SlotVec> CSEMapTy;
  CSEMapTy SrcCSEMap;
  typedef VASTRegister::assign_itertor assign_it;
  for (assign_it I = UseReg->assign_begin(), E = UseReg->assign_end();
       I != E; ++I) {
    VASTSlot *S = I->first->getSlot();
    assert(S && "Unexpected empty slot!");
    //Get Slack from the Define Value.
    SrcCSEMap[*I->second].push_back(S);
    //Get Slack from the condition of the assignment.
    SrcCSEMap[I->first].push_back(S);
  }

  typedef CSEMapTy::iterator it;
  for (it I = SrcCSEMap.begin(), E = SrcCSEMap.end(); I != E; ++I)
    computeSlackThrough(I->first, UseReg, I->second);
}

void CombPathDelayAnalysis::computeSlackThrough(VASTUse DefUse,
                                                VASTRegister *UseReg,
                                                SlotVec &UseSlots) {
  VASTValue *DefValue = DefUse.getOrNull();

  // If Define Value is immediate or symbol, skip it.
  if (!DefValue) return;

  if (VASTRegister *DefReg = dyn_cast<VASTRegister>(DefValue)) {
    unsigned Slack = getNearestSlotDistance(DefReg, UseSlots);
    // If the Define register and Use register already have a slack, compare the
    // slacks and assign the smaller one to the RegPathDelay Map.
    updateCombPathSlack(DefReg, UseReg, Slack);
    return;
  }

  DepthFristTraverseDataPathUseTree(DefUse, UseReg, UseSlots);
}

unsigned CombPathDelayAnalysis::getNearestSlotDistance(VASTRegister *DefReg,
                                                       SlotVec &UseSlots) {
  int NearestSlotDistance = FindShortestPath::Infinite;
  typedef VASTRegister::slot_iterator SlotIt;
  typedef SlotVec::iterator UseSlotIt;

  for (UseSlotIt UI = UseSlots.begin(), UE = UseSlots.end(); UI != UE; ++UI) {
    VASTSlot *UseSlot = *UI;
    for (SlotIt DI = DefReg->slots_begin(), DE = DefReg->slots_end();
         DI != DE; ++DI) {
      VASTSlot *DefSlot = *DI;

      int SlotDistance = FindSP->getSlotDistance(DefSlot, UseSlot);
      DEBUG(dbgs() << "\n(" << DefSlot->getSlotNum()
                   << "->" << UseSlot->getSlotNum()
                   << ") #" << SlotDistance << '\n');
      // if SlotDistance == 0, abandon this result.
      if (SlotDistance <= 0) continue;

      if (SlotDistance < NearestSlotDistance) {
        NearestSlotDistance = SlotDistance;
      }
    }
  }

  return NearestSlotDistance;
}

// Traverse the use tree in datapath, stop when we meet a register or other
// leaf node.
void
CombPathDelayAnalysis::DepthFristTraverseDataPathUseTree(VASTUse DefUse,
                                                         VASTRegister *UseReg,
                                                         SlotVec &UseSlots) {
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

    // Had we visited this node? If the Use slots are same, the same subtree
    // will lead to a same slack, and we do not need to compute the slack agian.
    if (!VisitedUses.insert(Node).second) {
      NodeWorkStack.pop_back();
      ItWorkStack.pop_back();
      continue;
    }

    ChildIt It = ItWorkStack.back();

    // Do we reach the leaf?
    if (Node.is_dp_leaf()) {
      if (VASTValue *V = Node.getOrNull()) {
        DEBUG(dbgs() << "Datapath:\t";
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

          Slack = std::max(Slack, getNearestSlotDistance(R, UseSlots));

          DEBUG(dbgs() << " Slack: " << int(Slack));

          // If the Define register and Use register already have a slack,
          // compare the slacks and assign the smaller one to the RegPathDelay
          // Map.
          updateCombPathSlack(R, UseReg, Slack);
        }

        DEBUG(dbgs() << '\n');
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
