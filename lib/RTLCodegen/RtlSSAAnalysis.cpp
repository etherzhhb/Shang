//RtlSSAAnalysis.cpp---- Analyse the dependency between registers---- C++ ---=//
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

#include "vtm/VerilogAST.h"
#include "vtm/Passes.h"
#include "vtm/VFInfo.h"

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallPtrSet.h"
#include "llvm/ADT/FoldingSet.h"
#include "llvm/ADT/SetOperations.h"
#define DEBUG_TYPE "vtm-reg-dependency"
#include "llvm/Support/Debug.h"
#include "llvm/Support/GraphWriter.h"

#include <map>

using namespace llvm;

namespace llvm {
  class ValueAtSlot;
  template<> struct FoldingSetTrait<ValueAtSlot>;

  class ValueAtSlot : public FoldingSetNode {
    friend struct FoldingSetTrait<ValueAtSlot>;

    VASTValue *V;
    VASTSlot *Slot;

    /// FastID - A reference to an Interned FoldingSetNodeID for this node.
    /// The ScalarEvolution's BumpPtrAllocator holds the data.
    const FoldingSetNodeIDRef FastID;

    // Vector for the dependent ValueAtSlots which is a Predecessor VAS.
    typedef SmallPtrSet<ValueAtSlot*, 8> VASVecTy;
    VASVecTy DepVAS;

    // Vector for the successor ValueAtSlot.
    VASVecTy UseVAS;

  public:
    explicit ValueAtSlot(const FoldingSetNodeIDRef ID, VASTValue *v,
                         VASTSlot *slot) : V(v), Slot(slot), FastID(ID) {}

    void addDepVAS(ValueAtSlot *VAS){
      DepVAS.insert(VAS);
      VAS->UseVAS.insert(this);
    }

    VASTValue *getValue() const { return V; }

    VASTSlot *getSlot() const { return Slot; }


    typedef VASVecTy::iterator iterator;
    iterator dep_begin() { return DepVAS.begin(); }
    iterator dep_end() { return DepVAS.end(); }

    iterator use_begin() { return UseVAS.begin(); }
    iterator use_end() { return UseVAS.end(); }

    bool operator==(ValueAtSlot &RHS) const {
      return V == RHS.getValue() && Slot == RHS.getSlot();
    }
  };

  template<> struct GraphTraits<ValueAtSlot*> {
    typedef ValueAtSlot NodeType;
    typedef NodeType::iterator ChildIteratorType;
    static NodeType *getEntryNode(NodeType* N) { return N; }
    static inline ChildIteratorType child_begin(NodeType *N) {
      return N->use_begin();
    }
    static inline ChildIteratorType child_end(NodeType *N) {
      return N->use_end();
    }
  };

  // Specialize FoldingSetTrait for ValueAtSlot to avoid needing to compute
  // temporary FoldingSetNodeID values.
  template<>
    struct FoldingSetTrait<ValueAtSlot> : DefaultFoldingSetTrait<ValueAtSlot> {
    static void Profile(const ValueAtSlot &X, FoldingSetNodeID& ID) {
      ID = X.FastID;
    }
    static bool Equals(const ValueAtSlot &X, const FoldingSetNodeID &ID,
      FoldingSetNodeID &TempID) {
        return ID == X.FastID;
    }
    static unsigned ComputeHash(const ValueAtSlot &X,
                                FoldingSetNodeID &TempID) {
      return X.FastID.ComputeHash();
    }
  };

  class SlotInfo {
  public:
    // Define the VAS set for the reaching definition dense map.
    typedef std::set<ValueAtSlot*> VASSetTy;
    typedef VASSetTy::iterator vasset_it;
  private:
    const VASTSlot *S;
    // Define Set for the reaching definition.
    VASSetTy SlotGen;
    typedef std::set<VASTValue*> ValueSet;
    ValueSet OverWrittenValue;
    VASSetTy SlotIn;
    VASSetTy SlotOut;
  public:
    SlotInfo(const VASTSlot *s) : S(s) {}
    // get the iterator of the defining map of reaching definition.
    vasset_it gen_begin() const { return SlotGen.begin(); }
    vasset_it gen_end() const { return SlotGen.end(); }
    vasset_it in_begin() const { return SlotIn.begin(); }
    vasset_it in_end() const { return SlotIn.end(); }
    vasset_it out_begin() const { return SlotOut.begin(); }
    vasset_it out_end() const { return SlotOut.end(); }

    // Get different VAS sets.
    VASSetTy &getGenVASSet() { return SlotGen; }
    VASSetTy &getInVASSet() { return SlotIn; }
    VASSetTy &getOutVASSet() { return SlotOut; }

    // Any VAS whose value is generated at this slot, but its slot is not equal
    // to this slot is killed.
    bool isVASKilled(const ValueAtSlot *VAS) const {
      return VAS->getSlot() != S && OverWrittenValue.count(VAS->getValue());
    }

    // Insert VAS into different set.
    void insertGen(ValueAtSlot *VAS) {
      SlotGen.insert(VAS);
      OverWrittenValue.insert(VAS->getValue());
    }

    void insertIn(ValueAtSlot *VAS) { SlotIn.insert(VAS); }

    // If the given slot lives in this slot?
    bool isLiveIn(ValueAtSlot *VAS) const { return SlotIn.count(VAS); }

    std::pair<SlotInfo::vasset_it, bool> insertOut(ValueAtSlot *VAS) {
      return SlotOut.insert(VAS);
    }

    // Clear the SlotOut set.
    void clearOutSet() { SlotOut.clear(); }

    // Get Slot pointer.
    const VASTSlot *getSlot() { return S; }
  };
}

namespace llvm{
class RtlSSAAnalysis : public MachineFunctionPass {
public:
  // define VASVec for the ValueAtSlot.
  typedef SmallVector<ValueAtSlot*, 4> VASVec;
  typedef VASVec::iterator vasvec_it;

  // Define small vector for the slots.
  typedef SmallVector<VASTSlot*, 4> SlotVecTy;
  typedef SlotVecTy::iterator slot_vec_it;

  typedef std::map<const VASTSlot* ,SlotInfo*> SlotInfoTy;
  typedef SlotInfoTy::const_iterator slotinfo_it;


private:
  typedef SlotInfo::VASSetTy VASSet;

  SlotInfoTy SlotInfos;

  // This vector is for the ValueAtSlot.
  VASVec AllVASs;

  SlotVecTy SlotVec;

  typedef FoldingSet<ValueAtSlot>::iterator fs_vas_it;
  FoldingSet<ValueAtSlot> UniqueVASs;
  BumpPtrAllocator Allocator;

  // define VAS assign iterator.
  typedef VASTRegister::assign_itertor assign_it;

public:
  static char ID;

  // All nodes (except exit node) are successors of the entry node.
  vasvec_it vas_begin() { return AllVASs.begin(); }
  vasvec_it vas_end() { return AllVASs.end(); }

  slot_vec_it slot_begin() { return SlotVec.begin(); }
  slot_vec_it slot_end() { return SlotVec.end(); }

  // Get SlotInfo from the existing SlotInfos set.
  SlotInfo* getSlotInfo(const VASTSlot *S) const;

  // Get SlotInfo from the existing SlotInfos set or create a slotInfo and
  // insert it to the SlotInfos set.
  SlotInfo *getOrCreateSlotInfo(const VASTSlot *S) ;

  ValueAtSlot *getOrCreateVAS(VASTValue *V, VASTSlot *S);

  // Traverse every register to define the ValueAtSlots.
  void buildAllVAS(VASTModule *VM);

  // Traverse every register to define the ValueAtSlots.
  void buildVASGraph(VASTModule *VM);

  // Add dependent ValueAtSlot.
  void addVASDep(ValueAtSlot *VAS, VASTRegister *DepReg);

  // Traverse the dependent VASTUse to get the registers.
  void visitDepTree(VASTUse DepTree, ValueAtSlot *VAS);

  // Traverse the use tree to get the registers.
  template<typename Func>
  void DepthFirstTraverseDepTree(VASTUse DefUse, ValueAtSlot *VAS, Func F);

  // Using the reaching definition algorithm to sort out the ultimate
  // relationship of registers.
  // Dirty hack: maybe there are two same statements is a slot, and we can use
  // bit vector to implement the algorithm similar to the compiler principle.
  void ComputeReachingDefinition();

  // collect the Generated and Killed statements of the slot.
  void ComputeGenAndKill();

  void viewGraph();

  void releaseMemory() {
    UniqueVASs.clear();
    Allocator.Reset();
    SlotVec.clear();
    AllVASs.clear();
    SlotInfos.clear();
  }

  void getAnalysisUsage(AnalysisUsage &AU) const {
    MachineFunctionPass::getAnalysisUsage(AU);
  }

  bool runOnMachineFunction(MachineFunction &MF);

  RtlSSAAnalysis() : MachineFunctionPass(ID) {
    initializeRtlSSAAnalysisPass(*PassRegistry::getPassRegistry());
  }
};

// Helper class
struct VASDepBuilder {
  RtlSSAAnalysis &A;
  VASDepBuilder(RtlSSAAnalysis &RtlSSA) : A(RtlSSA) {}

  void operator() (ValueAtSlot *VAS, VASTRegister *DefReg) {
    A.addVASDep(VAS, DefReg);
  }
};


template <> struct GraphTraits<RtlSSAAnalysis*>
: public GraphTraits<VASTSlot*> {

  typedef RtlSSAAnalysis::slot_vec_it nodes_iterator;
  static nodes_iterator nodes_begin(RtlSSAAnalysis *G) {
    return G->slot_begin();
  }
  static nodes_iterator nodes_end(RtlSSAAnalysis *G) {
    return G->slot_end();
  }
};

template<>
struct DOTGraphTraits<RtlSSAAnalysis*> : public DefaultDOTGraphTraits{
  typedef VASTSlot NodeTy;
  typedef RtlSSAAnalysis GraphTy;

  DOTGraphTraits(bool isSimple=false) : DefaultDOTGraphTraits(isSimple) {}

  static std::string getEdgeSourceLabel(const NodeTy *Node,
                                        NodeTy::succ_iterator I){
    std::string Str;
    raw_string_ostream ss(Str);
    ss << Node->getName();
    return ss.str();
  }

  std::string getNodeLabel(const NodeTy *Node, const GraphTy *Graph) {
    std::string Str;
    raw_string_ostream ss(Str);
    SlotInfo * SI = Graph->getSlotInfo(Node);
    for (SlotInfo::vasset_it I = SI->gen_begin(), E = SI->gen_end();
         I != E; ++I) {
      ValueAtSlot *VAS = *I;
        ss<<"    Gen: "<< VAS->getValue()->getName() << "   "
          << VAS->getSlot()->getName() << "\n";
    }
    ss << "\n\n";

    /*for (SlotInfo::vasset_it I = SI->getVASKillBegin(), E = SI->getVASKillEnd();
         I != E; ++I) {
      ValueAtSlot *VAS = *I;
      ss<<"    kill: "<< VAS->getValue()->getName() << "   "
        << VAS->getSlot()->getName() << "\n";
    }
    ss << "\n\n";*/
    /*for (RegDependencyAnalysis::vasset_it I = Graph->vas_in_begin(Node),
      E = Graph->vas_in_end(Node); I != E; ++I) {
        ValueAtSlot *VAS = *I;
        ss<<"    In: "<< VAS->getValue()->getName() << "   "
          << VAS->getSlot()->getName() << "\n";
    }
    ss << "\n\n";*/
    /*for (SlotInfo::vasset_it I = Graph->getVASOutBegin(Node),
         E = Graph->getVASOutEnd(Node); I != E; ++I) {
      ValueAtSlot *VAS = *I;
      ss<<"    Out: "<< VAS->getValue()->getName() << "   "
        << VAS->getSlot()->getName() << "\n";
    }*/
    return ss.str();
  }

  static std::string getNodeAttributes(const NodeTy *Node,
    const GraphTy *Graph) {
      return "shape=Mrecord";
  }
};

void RtlSSAAnalysis::viewGraph() {
  ViewGraph(this, "CompatibilityGraph" + utostr_32(ID));
}
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
  }

  // Define the VAS.
  buildAllVAS(VM);

  ComputeReachingDefinition();

  // Build the VAS dependence graph with reaching define information.
  buildVASGraph(VM);

  DEBUG(viewGraph());

  return false;
}

ValueAtSlot *RtlSSAAnalysis::getOrCreateVAS(VASTValue *V, VASTSlot *S){
  FoldingSetNodeID ID;
  ID.AddPointer(V);
  ID.AddPointer(S);
  void *IP = 0;
  if (ValueAtSlot *VAS = UniqueVASs.FindNodeOrInsertPos(ID, IP)) {
    //assert(cast<ValueAtSlot>(VAS)->getValue() == V &&
           //"Stale ValueAtSlot in uniquing map!");
    return VAS;
  }

  ValueAtSlot *VAS = new (Allocator) ValueAtSlot(ID.Intern(Allocator), V, S);
  UniqueVASs.InsertNode(VAS, IP);
  AllVASs.push_back(VAS);

  return VAS;
}

SlotInfo *RtlSSAAnalysis::getSlotInfo(const VASTSlot *S) const {
  slotinfo_it It = SlotInfos.find(S);
  return It == SlotInfos.end()? 0 : It->second;
}

SlotInfo *RtlSSAAnalysis::getOrCreateSlotInfo(const VASTSlot *S) {
  // Get SlotInfo if there exist in the SlotInfos map.
  SlotInfo *SIPointer = getSlotInfo(S);
  if (SIPointer) return SIPointer;

  // Create a new SlotInfo if it is not defined before.
  SIPointer = new (Allocator) SlotInfo(S);
  SlotInfos.insert(std::make_pair(S, SIPointer));
  return SIPointer;
}

void RtlSSAAnalysis::addVASDep(ValueAtSlot *VAS, VASTRegister *DepReg) {
  SlotInfo *SI = getSlotInfo(VAS->getSlot());
  assert(SI && "SlotInfo missed!");

  for (assign_it I = DepReg->assign_begin(), E = DepReg->assign_end();
       I != E; ++I){
    VASTSlot *DefSlot = I->first->getSlot();
    ValueAtSlot *DefVAS = getOrCreateVAS(DepReg, DefSlot);
    // VAS is only depends on DefVAS if it can reach this slot.
    if (SI->isLiveIn(DefVAS)) VAS->addDepVAS(DefVAS);
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
      (void) getOrCreateVAS(Reg, S);
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
      ValueAtSlot *VAS = getOrCreateVAS(R, S);
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
  typedef SlotInfo::vasset_it vas_it;
  // Collect the generated statements to the SlotGenMap.
  for (vasvec_it I = AllVASs.begin(), E = AllVASs.end(); I != E; ++I) {
    ValueAtSlot *VAS = *I;
    SlotInfo *SI = getOrCreateSlotInfo(VAS->getSlot());
    SI->insertGen(VAS);
  }

  // Build the Out set from Gen set.
  for (slot_vec_it I = SlotVec.begin(), E = SlotVec.end(); I != E; ++I) {
    VASTSlot *S =*I;
    assert(S && "Unexpected null slot!");
    SlotInfo *SI = getOrCreateSlotInfo(S);
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
