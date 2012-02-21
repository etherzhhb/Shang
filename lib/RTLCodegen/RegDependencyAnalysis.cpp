//RegDependencyAnalysis.cpp-- Analyse the dependency between registers- C++ -=//
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
#include "llvm/ADT/FoldingSet.h"

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
    FoldingSetNodeIDRef FastID;

    // Vector for the dependent ValueAtSlots which is a Predecessor VAS.
    typedef SmallVector<ValueAtSlot*, 4> VASVecTy;
    VASVecTy PredVAS;

    // Vector for the successor ValueAtSlot.
    VASVecTy SuccVAS;

  public:
    explicit ValueAtSlot(const FoldingSetNodeIDRef ID, VASTValue *v,
                         VASTSlot *slot) : FastID(ID), V(v), Slot(slot) {}

    void addPredValueAtSlot(ValueAtSlot *VAS){ PredVAS.push_back(VAS); }

    void addSuccValueAtSlot(ValueAtSlot *VAS) { SuccVAS.push_back(VAS); }

    VASTValue *getValue() const { return V; }

    VASTSlot *getSlot() const { return Slot; }


    typedef VASVecTy::iterator DVASIt;
    DVASIt pred_vas_begin() { return PredVAS.begin(); }
    DVASIt pred_vas_end() { return PredVAS.end(); }

    DVASIt succ_vas_begin() { return SuccVAS.begin(); }
    DVASIt succ_vas_end() { return SuccVAS.end(); }

    bool operator==(ValueAtSlot &RHS) const {
      if ((V == RHS.getValue()) && (Slot == RHS.getSlot())) return true;
      return false;
    }

  };

  template<> struct GraphTraits<ValueAtSlot*> {
    typedef ValueAtSlot NodeType;
    typedef NodeType::DVASIt ChildIteratorType;
    static NodeType *getEntryNode(NodeType* N) { return N; }
    static inline ChildIteratorType child_begin(NodeType *N) {
      return N->succ_vas_begin();
    }
    static inline ChildIteratorType child_end(NodeType *N) {
      return N->succ_vas_end();
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
}

namespace llvm{
class RegDependencyAnalysis : public MachineFunctionPass {
public:
  // define VASVec for the ValueAtSlot.
  typedef SmallVector<ValueAtSlot*, 4> VASVec;
  typedef VASVec::iterator vasvec_it;

  // Define small vector for the slots.
  typedef SmallVector<VASTSlot*, 4> SlotVecTy;
  typedef SlotVecTy::iterator slot_vec_it;

  // Define the VAS set for the reaching definition dense map.
  typedef std::set<ValueAtSlot*> VASSetTy;
  typedef VASSetTy::iterator vasset_it;

private:
  // This vector is for the ValueAtSlot.
  VASVec VASVector;

  SlotVecTy SlotVec;

  // ChangeSet is used to record the changes in the SlotOutMap.
  VASSetTy ChangeSet;
  // EraseSet is used to record the VAS from SlotKillMap which will be erase
  // from the SlotInMap.
  VASSetTy EraseSet;

  // Define Map for the reaching definition.
  typedef DenseMap<const VASTSlot*, VASSetTy> Slot2VecTy;
  Slot2VecTy SlotGenMap;
  Slot2VecTy SlotKillMap;
  Slot2VecTy SlotInMap;
  Slot2VecTy SlotOutMap;
  Slot2VecTy OldSlotOutMap;

  MachineFunction *MF;

  typedef FoldingSet<ValueAtSlot>::iterator fs_vas_it;
  FoldingSet<ValueAtSlot> UniqueVASs;
  BumpPtrAllocator VASAllocator;

  // define VAS assign iterator.
  typedef VASTRegister::assign_itertor assign_it;

  // define a function pointer which is use to add dependent VAS.
  typedef void(RegDependencyAnalysis::* addDependentVASFuncTy)(ValueAtSlot*,
                                                               VASTRegister*);

public:
  static char ID;

  // get the iterator of the defining map of reaching definition.
  vasset_it getVASGenBegin(const VASTSlot *S) const {
    return SlotGenMap.find(S)->second.begin();
  }
  vasset_it getVASGenEnd(const VASTSlot *S) const {
    return SlotGenMap.find(S)->second.end();
  }
  vasset_it getVASKillBegin(const VASTSlot *S) const {
    return SlotKillMap.find(S)->second.begin();
  }
  vasset_it getVASKillEnd(const VASTSlot *S) const {
    return SlotKillMap.find(S)->second.end();
  }
  vasset_it getVASInBegin(const VASTSlot *S) const {
    return SlotInMap.find(S)->second.begin();
  }
  vasset_it getVASInEnd(const VASTSlot *S) const {
    return SlotInMap.find(S)->second.end();
  }
  vasset_it getVASOutBegin(const VASTSlot *S) const {
    return SlotOutMap.find(S)->second.begin();
  }
  vasset_it getVASOutEnd(const VASTSlot *S) const {
    return SlotOutMap.find(S)->second.end();
  }

  // All nodes (except exit node) are successors of the entry node.
  vasvec_it vas_begin() { return VASVector.begin(); }
  vasvec_it vas_end() { return VASVector.end(); }

  slot_vec_it slot_begin() { return SlotVec.begin(); }
  slot_vec_it slot_end() { return SlotVec.end(); }

  ValueAtSlot *getOrCreateVAS(VASTValue *V, VASTSlot *S);

  // Traverse every register to define the ValueAtSlots.
  void DefineVAS(VASTModule *VM);

  // Add dependent ValueAtSlot.
  void addDependentVAS(ValueAtSlot *VAS, VASTRegister *DefReg);

  // Traverse the dependent VASTUse to get the registers.
  void TraverseDependentRegister(VASTUse *DefUse, ValueAtSlot *VAS);

  // Traverse the use tree to get the registers.
  void DepthFirstTraverseUseTree(addDependentVASFuncTy F, VASTUse DefUse,
                                 ValueAtSlot *VAS);

  // Using the reaching definition algorithm to sort out the ultimate
  // relationship of registers.
  // Dirty hack: maybe there are two same statements is a slot, and we can use
  // bit vector to implement the algorithm similar to the compiler principle.
  void ComputeReachingDefinition(MachineFunction &F, VASTModule *VM);

  // collect the Generated and Killed statements of the slot.
  void ComputeGenAndKill(VASTModule *VM);

  void viewGraph();

  void releaseMemory() {
    UniqueVASs.clear();
    VASAllocator.Reset();
  }

  void getAnalysisUsage(AnalysisUsage &AU) const {
    MachineFunctionPass::getAnalysisUsage(AU);
   // AU.addRequired<FindShortestPath>();
   // AU.addPreserved<FindShortestPath>();
  }

  bool runOnMachineFunction(MachineFunction &MF);

  RegDependencyAnalysis() : MachineFunctionPass(ID) {
    initializeRegDependencyAnalysisPass(*PassRegistry::getPassRegistry());
  }
};

template <> struct GraphTraits<RegDependencyAnalysis*>
: public GraphTraits<VASTSlot*> {

  typedef RegDependencyAnalysis::slot_vec_it nodes_iterator;
  static nodes_iterator nodes_begin(RegDependencyAnalysis *G) {
    return G->slot_begin();
  }
  static nodes_iterator nodes_end(RegDependencyAnalysis *G) {
    return G->slot_end();
  }
};

template<>
struct DOTGraphTraits<RegDependencyAnalysis*> : public DefaultDOTGraphTraits{
  typedef VASTSlot NodeTy;
  typedef RegDependencyAnalysis GraphTy;

  DOTGraphTraits(bool isSimple=false) : DefaultDOTGraphTraits(isSimple) {}

  static std::string getEdgeSourceLabel(const NodeTy *Node, NodeTy::succ_slot_it I){
    std::string Str;
    raw_string_ostream ss(Str);
    ss << Node->getName();
    return ss.str();
  }

  std::string getNodeLabel(const NodeTy *Node, const GraphTy *Graph) {
    std::string Str;
    raw_string_ostream ss(Str);
    /*for (RegDependencyAnalysis::vasset_it I = Graph->vas_gen_begin(Node),
         E = Graph->vas_gen_end(Node); I != E; ++I) {
      ValueAtSlot *VAS = *I;
        ss<<"    Gen: "<< VAS->getValue()->getName() << "   "
          << VAS->getSlot()->getName() << "\n";
    }
    ss << "\n\n";*/
    for (RegDependencyAnalysis::vasset_it I = Graph->vas_kill_begin(Node),
      E = Graph->vas_kill_end(Node); I != E; ++I) {
        ValueAtSlot *VAS = *I;
        ss<<"    kill: "<< VAS->getValue()->getName() << "   "
          << VAS->getSlot()->getName() << "\n";
    }
    ss << "\n\n";
    /*for (RegDependencyAnalysis::vasset_it I = Graph->vas_in_begin(Node),
      E = Graph->vas_in_end(Node); I != E; ++I) {
        ValueAtSlot *VAS = *I;
        ss<<"    In: "<< VAS->getValue()->getName() << "   "
          << VAS->getSlot()->getName() << "\n";
    }
    ss << "\n\n";*/
    for (RegDependencyAnalysis::vasset_it I = Graph->vas_out_begin(Node),
      E = Graph->vas_out_end(Node); I != E; ++I) {
        ValueAtSlot *VAS = *I;
        ss<<"    Out: "<< VAS->getValue()->getName() << "   "
          << VAS->getSlot()->getName() << "\n";
    }
    return ss.str();
  }

  static std::string getNodeAttributes(const NodeTy *Node,
    const GraphTy *Graph) {
      return "shape=Mrecord";
  }
};

void RegDependencyAnalysis::viewGraph() {
  ViewGraph(this, "CompatibilityGraph" + utostr_32(ID));
}

}

bool RegDependencyAnalysis::runOnMachineFunction(MachineFunction &F) {
  MF = &F;
  VASTModule *VM = MF->getInfo<VFInfo>()->getRtlMod();

  for (VASTModule::slot_iterator I = VM->slot_begin(), E = VM->slot_end();
    I != E; ++I) {
      VASTSlot *S = *I;
      // If the VASTslot is void, abandon it.
      if (!S) continue;

      SlotVec.push_back(S);
  }

  // Define the VAS.
  DefineVAS(VM);

  ComputeReachingDefinition(*MF, VM);

  viewGraph();

  return false;
}

ValueAtSlot *RegDependencyAnalysis::getOrCreateVAS(VASTValue *V,
                                                   VASTSlot *S){
  FoldingSetNodeID ID;
  ID.AddPointer(V);
  ID.AddPointer(S);
  void *IP = 0;
  if (ValueAtSlot *VAS = UniqueVASs.FindNodeOrInsertPos(ID, IP)) {
    //assert(cast<ValueAtSlot>(VAS)->getValue() == V &&
           //"Stale ValueAtSlot in uniquing map!");
    return VAS;
  }

  ValueAtSlot *VAS =
    new (VASAllocator)ValueAtSlot(ID.Intern(VASAllocator), V, S);
  UniqueVASs.InsertNode(VAS, IP);
  return VAS;
}

void RegDependencyAnalysis::addDependentVAS(ValueAtSlot *VAS,
                                            VASTRegister *DefReg) {
  for (assign_it I = DefReg->assign_begin(), E = DefReg->assign_end();
    I != E; ++I){
      VASTSlot *DefS = I->first->getSlot();
      ValueAtSlot *PredVAS = getOrCreateVAS(DefReg, DefS);
      VAS->addPredValueAtSlot(PredVAS);

      // Add the VAS to the successor VAS vector.
      PredVAS->addSuccValueAtSlot(VAS);
  }
}

void RegDependencyAnalysis::DefineVAS(VASTModule *VM) {
  for (VASTModule::reg_iterator I = VM->reg_begin(), E = VM->reg_end(); I != E;
    ++I){
      VASTRegister *UseReg = *I;

      typedef VASTRegister::assign_itertor assign_it;
      for (assign_it I = UseReg->assign_begin(), E = UseReg->assign_end();
        I != E; ++I) {
          VASTSlot *S = I->first->getSlot();
          // Create the origin VAS.
          ValueAtSlot *VAS = getOrCreateVAS(UseReg,S);
          VASTUse *DefUse = I->second;
          // Traverse the dependent VAS.
          TraverseDependentRegister(DefUse, VAS);
      }
  }
}

void RegDependencyAnalysis::TraverseDependentRegister(VASTUse *DefUse,
                                                      ValueAtSlot *VAS){

  VASTValue *DefValue = DefUse->getOrNull();

  // If Define Value is immediate or symbol, skip it.
  if (!DefValue) return;

  // If the define Value is register, add the dependent VAS to the
  // dependentVAS.
  if (VASTRegister *DefReg = dyn_cast<VASTRegister>(DefValue)){
    addDependentVAS(VAS, DefReg);
    return;
  }

  // If the define Value is wire, traverse the use tree to get the
  // ultimate registers.
  DepthFirstTraverseUseTree(&RegDependencyAnalysis::addDependentVAS, *DefUse,
                            VAS);
}

void RegDependencyAnalysis::DepthFirstTraverseUseTree(addDependentVASFuncTy F,
                                                      VASTUse DefUse,
                                                      ValueAtSlot *VAS) {
  typedef VASTUse::iterator ChildIt;
  // Use seperate node and iterator stack, so we can get the path vector.
  typedef SmallVector<VASTUse, 16> NodeStackTy;
  typedef SmallVector<ChildIt, 16> ItStackTy;
  NodeStackTy NodeWorkStack;
  ItStackTy ItWorkStack;
  // Remember what we had visited.
  std::set<VASTUse> VisitedUses;

  // Put the current node into the node stack, so it will appears in the path.
  NodeWorkStack.push_back(VAS->getValue());

  // Put the root.
  NodeWorkStack.push_back(DefUse);
  ItWorkStack.push_back(DefUse.dp_src_begin());

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
          (this->*F)(VAS, R);
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

  assert(NodeWorkStack.back().get() == VAS->getValue() && "Node stack broken!");
}

void RegDependencyAnalysis::ComputeReachingDefinition(MachineFunction &F,
                                                      VASTModule *VM) {
  ComputeGenAndKill(VM);

  do {
    ChangeSet.clear();

    for (VASTModule::slot_iterator I = VM->slot_begin(), E = VM->slot_end();
         I != E; ++I) {
      VASTSlot *S =*I;

      // If the VASTslot is void, abandon it.
      if (!S) continue;

      // Return true If this slot jump from some other slots conditionally
      // instead of inherits from SlotNum - 1 slot. If false, compute the
      // SlotInMap.
      if (!S->hasExplicitPredSlots()) {
        VASTSlot *PS = VM->getSlot(S->getSlotNum()-1);
        for (vasset_it I = SlotOutMap[PS].begin(), E = SlotOutMap[PS].end();
              I != E; ++I) {
          ValueAtSlot *OutVAS = *I;
          SlotInMap[S].insert(OutVAS);
        }
      }

      // Compute the SlotInMap.
      for (VASTSlot::pred_iterator I = S->pred_begin(), E = S->pred_end();
            I != E; ++I) {
        VASTSlot *PS = VM->getSlot(*I);
        for (vasset_it I = SlotOutMap[PS].begin(), E = SlotOutMap[PS].end();
              I != E; ++I) {
          ValueAtSlot *OutVAS = *I;
          SlotInMap[S].insert(OutVAS);
        }
      }

      // Compute the SlotInMap subtract the SlotKillMap. Push all the VAS
      // which should be erased to the EraseSet.
      for (vasset_it I = SlotInMap[S].begin(), E = SlotInMap[S].end();
            I != E; ++I) {
        ValueAtSlot *InVAS = *I;
        if (SlotKillMap[S].count(InVAS)) {
          EraseSet.insert(InVAS);
        }
      }

      // Erase the VAS from SlotInMap.
      for (vasset_it I = EraseSet.begin(), E = EraseSet.end(); I != E; ++I) {
        ValueAtSlot *VAS = *I;
        SlotInMap[S].erase(VAS);
      }

      EraseSet.clear();
      SlotOutMap[S].clear();

      // Compute the SlotOutMap. insert the VAS from the SlotGenMap.
      for (vasset_it I = SlotGenMap[S].begin(), E = SlotGenMap[S].end();
            I != E; ++I) {
        ValueAtSlot *GenVAS = *I;
        SlotOutMap[S].insert(GenVAS);
      }

      // Compute the SlotOutMap. Insert the VAS from the SlotInMap.
      for (vasset_it I = SlotInMap[S].begin(), E = SlotInMap[S].end(); I != E;
            ++I) {
        ValueAtSlot *InVAS = *I;
        SlotOutMap[S].insert(InVAS);
      }

      // Compare the SlotOutMap and OldSlotOutMap, findout whether there are
      // changes in the SlotOut.
      for (vasset_it I = SlotOutMap[S].begin(), E = SlotOutMap[S].end();
            I != E; ++I) {
        ValueAtSlot *NewVAS = *I;
        if (!OldSlotOutMap[S].count(NewVAS)) {
          ChangeSet.insert(NewVAS);
        }
      }

      // clear the old SlotOutMap and replace with the new SlotOutMap.
      OldSlotOutMap[S].clear();
      for (vasset_it I = SlotOutMap[S].begin(), E = SlotOutMap[S].end();
            I != E; ++I) {
        ValueAtSlot *VAS = *I;
        OldSlotOutMap[S].insert(VAS);
      }
    }
  } while (!ChangeSet.empty());
}

void RegDependencyAnalysis::ComputeGenAndKill(VASTModule *VM){
    // Push all the VAS from the UniqueVASs to the VASVector.
  for (fs_vas_it I = UniqueVASs.begin(), E = UniqueVASs.end(); I != E; ++I) {
    ValueAtSlot *VAS = &*I;
    VASVector.push_back(VAS);
  }

  // Collect the generated statements to the SlotGenMap, and collect the killed
  // statements to the SlotKillMap.
  for (VASTModule::slot_iterator I = VM->slot_begin(), E = VM->slot_end();
    I != E; ++I) {
    VASTSlot *S = *I;
    // If the VASTslot is void, abandon it.
    if (!S) continue;

    DEBUG(dbgs()<<"origin slot: "<< S->getName()<< "\n";);

    // Collect the generated statements to the SlotGenMap.
    for (vasvec_it I = VASVector.begin(), E = VASVector.end(); I != E; ++I) {
      ValueAtSlot *GenVAS = *I;

      // If the VAS have the same slot with S, then its a Generated VAS to this
      // slot.
      if (GenVAS->getSlot() == S) (SlotGenMap[S]).insert(GenVAS);
    }

    // Collect the killed statements to the SlotGenMap.
    for (vasset_it I = SlotGenMap[S].begin(), E = SlotGenMap[S].end(); I != E;
      ++I) {
      ValueAtSlot *GenVAS = *I;

      for (vasvec_it VI = VASVector.begin(), VE = VASVector.end(); VI != VE;
        ++VI) {
        ValueAtSlot *KillVAS = *VI;

        // abandon the same VAS.
        if (GenVAS == KillVAS) continue;

        // If the KillVAS have the same value with the GenVAS, then it's killed.
        if (GenVAS->getValue() == KillVAS->getValue())
          SlotKillMap[S].insert(KillVAS);
      }
    }

    DEBUG(
      for (vasset_it I = SlotGenMap[S].begin(), E = SlotGenMap[S].end(); I != E;
        ++I) {
          ValueAtSlot *VAS = *I;
          if (S->getSlotNum() == 2) {
            dbgs()<<"    Gen: "<< VAS->getValue()->getName() << "   "
                  << VAS->getSlot()->getName() << "\n";
          }
      }
      for (vasset_it I = SlotKillMap[S].begin(), E = SlotKillMap[S].end();
        I != E; ++I) {
          ValueAtSlot *VAS = *I;
          if (S->getSlotNum() == 2) {
            dbgs()<<"    Kill: "<< VAS->getValue()->getName() << "   "
                  << VAS->getSlot()->getName()  << "\n";
          }
      }
    );
  }
}

char RegDependencyAnalysis::ID = 0;
INITIALIZE_PASS_BEGIN(RegDependencyAnalysis, "RegDependencyAnalysis",
                      "RegDependencyAnalysiss", false, false)
INITIALIZE_PASS_END(RegDependencyAnalysis, "RegDependencyAnalysis",
                    "RegDependencyAnalysis", false, false)

Pass *llvm::createRegDependencyAnalysisPass() {
  return new RegDependencyAnalysis();
}
