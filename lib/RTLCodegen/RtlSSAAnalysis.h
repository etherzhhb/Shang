//===- RtlSSAAnalysis.h - Analyse the dependency between registers - C++ ----=//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This define the RtlSSAAnalysis pass, which construct SSA form on register
// transfer level.
//
//===----------------------------------------------------------------------===//


#include "vtm/VerilogAST.h"
#include "vtm/Utilities.h"

#include "llvm/ADT/GraphTraits.h"
#include "llvm/ADT/SmallPtrSet.h"
#include "llvm/Support/Allocator.h"

#ifndef RTL_SSA_ANALYSIS_H
#define RTL_SSA_ANALYSIS_H


namespace llvm {
// ValueAtSlot, represent the value that is defined at a specific slot.
class ValueAtSlot {
  VASTValue *V;
  VASTSlot *Slot;

  // Vector for the dependent ValueAtSlots which is a Predecessor VAS.
  typedef SmallPtrSet<ValueAtSlot*, 8> VASVecTy;
  VASVecTy DepVAS;

  // Vector for the successor ValueAtSlot.
  VASVecTy UseVAS;

public:
  ValueAtSlot(VASTValue *v, VASTSlot *slot) : V(v), Slot(slot){}

  void addDepVAS(ValueAtSlot *VAS){
    DepVAS.insert(VAS);
    VAS->UseVAS.insert(this);
  }

  VASTValue *getValue() const { return V; }
  VASTSlot *getSlot() const { return Slot; }
  std::string getName() const {
    return std::string(getValue()->getName()) + "@"
      + utostr_32(getSlot()->getSlotNum());
  }

  void print(raw_ostream &OS) const;

  void dump() const;

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

// SlotInfo, store the data-flow information of a slot.
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

  // Get Slot pointer.
  const VASTSlot *getSlot() { return S; }
};

// The RtlSSAAnalysis that construct the SSA form.
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
  SlotVecTy SlotVec;

  typedef DenseMap<std::pair<VASTValue*, VASTSlot*>, ValueAtSlot*> VASMapTy;
  VASMapTy UniqueVASs;
  // Use mapped_iterator which is a simple iterator adapter that causes a
  // function to be dereferenced whenever operator* is invoked on the iterator.
  typedef
  std::pointer_to_unary_function<std::pair<std::pair<VASTValue*, VASTSlot*>,
                                                     ValueAtSlot*>,
                                 ValueAtSlot*>
  vas_getter;

  typedef mapped_iterator<VASMapTy::iterator, vas_getter> vas_iterator;

  BumpPtrAllocator Allocator;

  // define VAS assign iterator.
  typedef VASTRegister::assign_itertor assign_it;

public:
  static char ID;

  // All nodes (except exit node) are successors of the entry node.
  vas_iterator vas_begin() {
    return vas_iterator(UniqueVASs.begin(),
      vas_getter(pair_second<std::pair<VASTValue*, VASTSlot*>,
      ValueAtSlot*>));
  }

  vas_iterator vas_end() {
    return vas_iterator(UniqueVASs.end(),
      vas_getter(pair_second<std::pair<VASTValue*, VASTSlot*>,
      ValueAtSlot*>));
  }

  slot_vec_it slot_begin() { return SlotVec.begin(); }
  slot_vec_it slot_end() { return SlotVec.end(); }

  // Get SlotInfo from the existing SlotInfos set.
  SlotInfo* getSlotInfo(const VASTSlot *S) const;

  ValueAtSlot *getValueASlot(VASTValue *V, VASTSlot *S);

  // Traverse every register to define the ValueAtSlots.
  void buildAllVAS(VASTModule *VM);

  // Traverse every register to define the ValueAtSlots.
  void buildVASGraph(VASTModule *VM);

  // Add dependent ValueAtSlot.
  void addVASDep(ValueAtSlot *VAS, VASTRegister *DepReg);

  // Traverse the dependent VASTUse to get the registers.
  void visitDepTree(VASTUse DepTree, ValueAtSlot *VAS);

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
    SlotInfos.clear();
  }

  void getAnalysisUsage(AnalysisUsage &AU) const {
    MachineFunctionPass::getAnalysisUsage(AU);
    AU.addRequired<CFGShortestPath>();
    AU.addRequired<TargetData>();
    AU.setPreservesAll();
  }

  bool runOnMachineFunction(MachineFunction &MF);

  RtlSSAAnalysis() : MachineFunctionPass(ID) {
    initializeRtlSSAAnalysisPass(*PassRegistry::getPassRegistry());
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
}
#endif
