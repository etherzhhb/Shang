//===- RtlSSAAnalysis.cpp - Analyse the dependency between registers - C++ --=//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This pass collect the slots information of register and map them into a map
// vector. then it will analyse dependency between registers.
//
//
//===----------------------------------------------------------------------===//

#include "CFGShortestPath.h"
#include "RtlSSAAnalysis.h"

#include "vtm/Passes.h"

#include "llvm/Target/TargetData.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallPtrSet.h"
#include "llvm/ADT/SetOperations.h"
#include "llvm/Support/SourceMgr.h"
#define DEBUG_TYPE "vtm-reg-dependency"
#include "llvm/Support/Debug.h"
#include "llvm/Support/GraphWriter.h"

#include <map>

using namespace llvm;

namespace llvm {
template<>
struct DOTGraphTraits<RtlSSAAnalysis*> : public DefaultDOTGraphTraits{
  typedef VASTSlot NodeTy;
  typedef RtlSSAAnalysis GraphTy;

  DOTGraphTraits(bool isSimple=false) : DefaultDOTGraphTraits(isSimple) {}

  std::string getNodeLabel(const NodeTy *Node, const GraphTy *Graph) {
    std::string Str;
    raw_string_ostream ss(Str);
    SlotInfo * SI = Graph->getSlotInfo(Node);
    typedef SlotInfo::vasset_it it;

    ss << Node->getName() << "\nGen:\n";
    for (it I = SI->gen_begin(), E = SI->gen_end(); I != E; ++I) {
      ValueAtSlot *VAS = *I;
      ss.indent(2) << VAS->getName() << "\n";
    }

    ss << "\n\nIn:\n";
    for (it I = SI->in_begin(), E = SI->in_end(); I != E; ++I) {
      ValueAtSlot *VAS = *I;
      ss.indent(2) << VAS->getName() << "\n";
    }
    ss << "\n\n";

    return ss.str();
  }

  static std::string getNodeAttributes(const NodeTy *Node,
                                       const GraphTy *Graph) {
      return "shape=Mrecord";
  }
};
}

void RtlSSAAnalysis::viewGraph() {
  ViewGraph(this, "CompatibilityGraph" + utostr_32(ID));
}


// Helper class
struct VASDepBuilder {
  RtlSSAAnalysis &A;
  VASDepBuilder(RtlSSAAnalysis &RtlSSA) : A(RtlSSA) {}

  void operator() (ValueAtSlot *VAS, VASTRegister *DefReg) {
    A.addVASDep(VAS, DefReg);
  }
};

void ValueAtSlot::print(raw_ostream &OS) const {
  OS << getValue()->getName() << '@' << getSlot()->getSlotNum()
    << "\t <= {";

  typedef VASVecTy::iterator it;
  for (it I = DepVAS.begin(), E = DepVAS.end(); I != E; ++I)
    OS << (*I)->getName() << ',';

  OS << "}\n";
}

void ValueAtSlot::dump() const {
  print(dbgs());
}

// FIXME: Move this to a separate pass!
// The first node of the path is the use node and the last node of the path is
// the define node.
static void bindPath2ScriptEngine(ArrayRef<VASTUse> Path,
                                  unsigned Slack) {
  assert(Path.size() >= 2 && "Path vector have less than 2 nodes!");
  assert(Slack && "Unexpected zero slack!");
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

// FIXME: Move this to a separate pass!
bool RtlSSAAnalysis::doInitialization(Module &) {
  SMDiagnostic Err;
  // Get the script from script engine.
  const char *HeaderScriptPath[] = { "Misc",
    "TimingConstraintsHeaderScript" };
  if (!runScriptStr(getStrValueFromEngine(HeaderScriptPath), Err))
    report_fatal_error("Error occur while running timing header script:\n"
    + Err.getMessage());
  return false;
}

bool RtlSSAAnalysis::runOnMachineFunction(MachineFunction &MF) {
  VASTModule *VM = MF.getInfo<VFInfo>()->getRtlMod();

  // Push back all the slot into the SlotVec for the purpose of view graph.
  typedef VASTModule::slot_iterator slot_it;
  for (slot_it I = VM->slot_begin(), E = VM->slot_end(); I != E; ++I) {
    VASTSlot *S = *I;
    // If the VASTslot is void, abandon it.
    if (!S) continue;

    SlotVec.push_back(S);
    // Create a new SlotInfo if it is not defined before.
    SlotInfo *SI = new (Allocator) SlotInfo(S);
    bool inserted = SlotInfos.insert(std::make_pair(S, SI)).second;
    assert(inserted && "SlotInfo inserted?");
    (void) inserted;
  }

  // Define the VAS.
  buildAllVAS(VM);

  ComputeReachingDefinition();

  // Build the VAS dependence graph with reaching define information.
  buildVASGraph(VM);

  DEBUG(viewGraph());

  // FIXME: Move this to a separate pass!
  bindFunctionInfoToScriptEngine(MF, getAnalysis<TargetData>());

  CFGShortestPath &CFGSP = getAnalysis<CFGShortestPath>();
  DenseMap<VASTValue*, int> DistanceMap;
  // Dirty Hack: Write timing script according to the VAS Graph.
  // Collect the generated statements to the SlotGenMap.
  for (vas_iterator I = vas_begin(), E = vas_end(); I != E; ++I) {
    ValueAtSlot *DstVAS = *I;
    DistanceMap.clear();

    typedef ValueAtSlot::iterator dep_it;
    for (dep_it DI = DstVAS->dep_begin(), DE = DstVAS->dep_end();DI != DE;++DI){
      ValueAtSlot *SrcVAS = *DI;

      int D = CFGSP.getSlotDistance(SrcVAS->getSlot(), DstVAS->getSlot());
      assert(D > 0 && "SrcSlot should able to reach DstSlot!");
      int &Distance = DistanceMap[SrcVAS->getValue()];
      if (Distance == 0) Distance = CFGShortestPath::Infinite;
      Distance = std::min(Distance, D);
    }

    typedef DenseMap<VASTValue*, int>::iterator distance_it;
    for (distance_it DI = DistanceMap.begin(), DE = DistanceMap.end();
         DI != DE; ++DI) {
      // Path from Src to Dst
      VASTUse Path[] = { DstVAS->getValue(), DI->first };
      bindPath2ScriptEngine(Path, DI->second);
    }
  }

  return false;
}

ValueAtSlot *RtlSSAAnalysis::getValueASlot(VASTValue *V, VASTSlot *S){
  ValueAtSlot *VAS = UniqueVASs.lookup(std::make_pair(V, S));
  assert(VAS && "VAS not exist!");
  return VAS;
}

SlotInfo *RtlSSAAnalysis::getSlotInfo(const VASTSlot *S) const {
  slotinfo_it It = SlotInfos.find(S);
  assert(It != SlotInfos.end() && "SlotInfo not exist!");
  return It->second;
}

void RtlSSAAnalysis::addVASDep(ValueAtSlot *VAS, VASTRegister *DepReg) {
  VASTSlot *UseSlot = VAS->getSlot();
  SlotInfo *UseSI = getSlotInfo(UseSlot);
  assert(UseSI && "SlotInfo missed!");

  for (assign_it I = DepReg->assign_begin(), E = DepReg->assign_end();
       I != E; ++I){
    VASTSlot *DefSlot = I->first->getSlot();
    ValueAtSlot *DefVAS = getValueASlot(DepReg, DefSlot);

    // VAS is only depends on DefVAS if it can reach this slot.
    if (UseSI->isLiveIn(DefVAS)) VAS->addDepVAS(DefVAS);
  }
}

void RtlSSAAnalysis::buildAllVAS(VASTModule *VM) {
  typedef VASTModule::reg_iterator reg_it;
  for (reg_it I = VM->reg_begin(), E = VM->reg_end(); I != E; ++I){
      VASTRegister *Reg = *I;

    typedef VASTRegister::assign_itertor assign_it;
    for (assign_it I = Reg->assign_begin(), E = Reg->assign_end(); I != E; ++I){
      VASTSlot *S = I->first->getSlot();

      // Create the origin VAS.
      ValueAtSlot *VAS = new (Allocator) ValueAtSlot(Reg, S);
      UniqueVASs.insert(std::make_pair(std::make_pair(Reg, S), VAS));
    }
  }
}

void RtlSSAAnalysis::buildVASGraph(VASTModule *VM) {
  typedef VASTModule::reg_iterator it;
  for (VASTModule::reg_iterator I = VM->reg_begin(), E = VM->reg_end(); I != E;
       ++I){
    VASTRegister *R = *I;

    typedef VASTRegister::assign_itertor assign_it;
    for (assign_it I = R->assign_begin(), E = R->assign_end(); I != E; ++I) {
      VASTSlot *S = I->first->getSlot();
      // Create the origin VAS.
      ValueAtSlot *VAS = getValueASlot(R, S);
      // Build dependence for conditions
      visitDepTree(I->first, VAS);
      // Build dependence for the assigning value.
      visitDepTree(*I->second, VAS);
    }
  }
}

void RtlSSAAnalysis::visitDepTree(VASTUse DepTree, ValueAtSlot *VAS){
  VASTValue *DefValue = DepTree.getOrNull();

  // If Define Value is immediate or symbol, skip it.
  if (!DefValue) return;

  // If the define Value is register, add the dependent VAS to the
  // dependentVAS.
  if (VASTRegister *DepReg = dyn_cast<VASTRegister>(DefValue)){
    addVASDep(VAS, DepReg);
    return;
  }

  VASDepBuilder B(*this);
  // If the define Value is wire, traverse the use tree to get the
  // ultimate registers.
  DepthFirstTraverseDepTree(DepTree, VAS, B);
}

template<typename Func>
void RtlSSAAnalysis::DepthFirstTraverseDepTree(VASTUse DepTree, ValueAtSlot *VAS,
                                               Func F) {
  typedef VASTUse::iterator ChildIt;
  // Use seperate node and iterator stack, so we can get the path vector.
  typedef SmallVector<VASTUse, 16> NodeStackTy;
  typedef SmallVector<ChildIt, 16> ItStackTy;
  NodeStackTy NodeWorkStack;
  ItStackTy ItWorkStack;
  // Remember what we had visited.
  std::set<VASTUse> VisitedUses;

  // Put the root.
  NodeWorkStack.push_back(DepTree);
  ItWorkStack.push_back(DepTree.dp_src_begin());

  while (!ItWorkStack.empty()) {
    VASTUse Node = NodeWorkStack.back();

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
          // Add dependent VAS. Use the function pointer to get the desired
          // function.
          F(VAS, R);
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

    // Had we visited this node? If the Use slots are same, the same subtree
    // will lead to a same slack, and we do not need to compute the slack agian.
    if (!VisitedUses.insert(ChildNode).second) continue;

    // If ChildNode is not visit, go on visit it and its childrens.
    NodeWorkStack.push_back(ChildNode);
    ItWorkStack.push_back(ChildNode.dp_src_begin());
  }

  assert(NodeWorkStack.empty() && "Node stack broken!");
}

void RtlSSAAnalysis::ComputeReachingDefinition() {
  ComputeGenAndKill();
  // TODO: Simplify the data-flow, some slot may neither define new VAS nor
  // kill any VAS.

  bool Change = false;

  do {
    Change = false;

    for (slot_vec_it I = SlotVec.begin(), E = SlotVec.end(); I != E; ++I) {
      VASTSlot *S =*I;
      assert(S && "Unexpected null slot!");
      // No need to update the out set of Slot 0 according its incoming value.
      // It is the first slot of the FSM.
      if (S->getSlotNum() == 0) continue;

      SlotInfo *SI = getSlotInfo(S);
      assert(SI && "Slot information not existed?");

      // Compute the out set.
      typedef VASTSlot::pred_it pred_it;
      for (pred_it PI = S->pred_begin(), PE = S->pred_end(); PI != PE; ++PI) {
        VASTSlot *PS = *PI;
        SlotInfo *PSI = getSlotInfo(PS);
        assert(PSI && "Slot information not existed?");
        typedef SlotInfo::vasset_it it;
        for (it II = PSI->out_begin(), IE = PSI->out_end(); II != IE; ++II) {
          ValueAtSlot *PredOut = *II;
          SI->insertIn(PredOut);
          // Do not let the killed VASs go out
          if (!SI->isVASKilled(PredOut))
            // New out occur.
            Change |= SI->insertOut(PredOut).second;
        }
      }
    }
  } while (Change);
}

void RtlSSAAnalysis::ComputeGenAndKill(){
  // Collect the generated statements to the SlotGenMap.
  for (vas_iterator I = vas_begin(), E = vas_end(); I != E; ++I) {
    ValueAtSlot *VAS = *I;
    SlotInfo *SI = getSlotInfo(VAS->getSlot());
    SI->insertGen(VAS);
  }

  // Build the Out set from Gen set.
  for (slot_vec_it I = SlotVec.begin(), E = SlotVec.end(); I != E; ++I) {
    VASTSlot *S =*I;
    assert(S && "Unexpected null slot!");
    SlotInfo *SI = getSlotInfo(S);
    typedef SlotInfo::vasset_it vas_it;
    for (vas_it VI = SI->gen_begin(), VE = SI->gen_end(); VI != VE; ++VI) {
      ValueAtSlot *VAS = *VI;
      SI->insertOut(VAS);
      DEBUG(dbgs() << "    Gen: " << VAS->getValue()->getName() << "   "
                    << VAS->getSlot()->getName() << "\n");
    }
  }
}

char RtlSSAAnalysis::ID = 0;
INITIALIZE_PASS_BEGIN(RtlSSAAnalysis, "RtlSSAAnalysis",
                      "RtlSSAAnalysis", false, false)
INITIALIZE_PASS_END(RtlSSAAnalysis, "RtlSSAAnalysis",
                    "RtlSSAAnalysis", false, false)

Pass *llvm::createRtlSSAAnalysisPass() {
  return new RtlSSAAnalysis();
}
