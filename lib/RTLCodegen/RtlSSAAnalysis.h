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

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/ADT/GraphTraits.h"
#include "llvm/ADT/SmallPtrSet.h"
#include "llvm/Support/Allocator.h"

#ifndef RTL_SSA_ANALYSIS_H
#define RTL_SSA_ANALYSIS_H


namespace llvm {
class RtlSSAAnalysis;
class SlotInfo;

// ValueAtSlot, represent the value that is defined at a specific slot.
class ValueAtSlot {
  struct LiveInInfo {
    union {
      struct {
        uint32_t Cycles : 31;
        uint32_t FromOtherBB : 1;
      } I;

      uint32_t D;
    } U;

    LiveInInfo() { U.D = 0; }

    uint32_t getCycles() const { return U.I.Cycles; }
    bool isFromOtherBB() const { return U.I.FromOtherBB; }
    uint32_t getData() const { return U.D; }
  };

  VASTSignal *V;
  VASTSlot *Slot;

  // Vector for the dependent ValueAtSlots which is a Predecessor VAS.
  typedef DenseMap<ValueAtSlot*, LiveInInfo> VASCycMapTy;
  VASCycMapTy DepVAS;

  typedef SmallPtrSet<ValueAtSlot*, 8> VASSetTy;
  // Vector for the successor ValueAtSlot.
  VASSetTy UseVAS;

  void addDepVAS(ValueAtSlot *VAS, unsigned CyclesFormDef, bool FromOtherBB){
    assert(CyclesFormDef && "Expect non-zero distance!");

    LiveInInfo &Info = DepVAS[VAS];

    if (Info.getCycles() == 0) VAS->UseVAS.insert(this);

    if (Info.getCycles() == 0 || Info.getCycles() > CyclesFormDef) {
      // Try to take the shortest path.
      Info.U.I.Cycles = CyclesFormDef;
      Info.U.I.FromOtherBB = FromOtherBB;
    }
  }

  ValueAtSlot(VASTSignal *v, VASTSlot *slot) : V(v), Slot(slot){}
  ValueAtSlot(const ValueAtSlot&); // Do not implement.

  LiveInInfo getDepInfo(ValueAtSlot *VAS) const {
    VASCycMapTy::const_iterator at = DepVAS.find(VAS);
    return at == DepVAS.end() ? LiveInInfo() : at->second;
  }

  friend class SlotInfo;
  friend class RtlSSAAnalysis;
public:
  VASTSignal *getValue() const { return V; }
  VASTSlot *getSlot() const { return Slot; }
  std::string getName() const {
    return std::string(getValue()->getName()) + "@"
      + utostr_32(getSlot()->getSlotNum());
  }

  unsigned getCyclesFromDef(ValueAtSlot *VAS) const {
    return getDepInfo(VAS).getCycles();
  }

  bool isFromOtherBB(ValueAtSlot *VAS) const {
    return getDepInfo(VAS).isFromOtherBB();
  }

  void verify() const;

  void print(raw_ostream &OS) const;

  void dump() const;

  typedef VASSetTy::iterator iterator;
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
  // Define the VAS set for the reaching definition dense map.
  typedef std::set<ValueAtSlot*> VASSetTy;
  typedef std::map<ValueAtSlot*, ValueAtSlot::LiveInInfo> VASCycMapTy;
  const VASTSlot *S;
  // Define Set for the reaching definition.
  VASSetTy SlotGen;
  typedef std::set<VASTValue*> ValueSet;
  ValueSet OverWrittenValue;
  // In/Out set with cycles form define information.
  VASCycMapTy SlotIn;
  VASCycMapTy SlotOut;

  typedef VASSetTy::iterator gen_iterator;
  // get the iterator of the defining map of reaching definition.
  gen_iterator gen_begin() const { return SlotGen.begin(); }
  gen_iterator gen_end() const { return SlotGen.end(); }

  typedef
  std::pointer_to_unary_function<std::pair<ValueAtSlot*, unsigned>,
                                 ValueAtSlot*>
  vas_getter;

  static bool updateSet(ValueAtSlot *VAS, unsigned LiveInCycle,
                        bool FromOtherBB, VASCycMapTy &S) {
    assert(LiveInCycle && "It takes at least a cycle to live in!");

    ValueAtSlot::LiveInInfo &Info = S[VAS];

    unsigned OldInfo = Info.getData();

    if (Info.U.I.Cycles == 0 || Info.U.I.Cycles > LiveInCycle) {
      // Try to take the shortest path.
      Info.U.I.Cycles = LiveInCycle;
      Info.U.I.FromOtherBB = FromOtherBB;
    }

    // Updated?
    return OldInfo != Info.getData();
  }

  ValueAtSlot::LiveInInfo getLiveIn(ValueAtSlot *VAS) const {
    vascyc_iterator at = SlotIn.find(VAS);
    return at == SlotIn.end() ? ValueAtSlot::LiveInInfo() : at->second;
  }

  // Initialize the out set by simply copying the gen set, and initialize the
  // cycle counter to 0.
  void initOutSet();

  // Insert VAS into different set.
  void insertGen(ValueAtSlot *VAS) {
    SlotGen.insert(VAS);
    OverWrittenValue.insert(VAS->getValue());
  }

  bool insertIn(ValueAtSlot *VAS, unsigned LiveInCycle, bool FromOtherBB) {
    return updateSet(VAS, LiveInCycle, FromOtherBB, SlotIn);
  }

  bool insertOut(ValueAtSlot *VAS, unsigned LiveInCycle, bool FromOtherBB) {
    return updateSet(VAS, LiveInCycle, FromOtherBB, SlotOut);
  }

  friend class RtlSSAAnalysis;
public:
  SlotInfo(const VASTSlot *s) : S(s) {}

  typedef VASCycMapTy::const_iterator vascyc_iterator;
  typedef mapped_iterator<VASCycMapTy::iterator, vas_getter> iterator;

  vascyc_iterator in_begin() const { return SlotIn.begin(); }
  vascyc_iterator in_end() const { return SlotIn.end(); }
  vascyc_iterator out_begin() const { return SlotOut.begin(); }
  vascyc_iterator out_end() const { return SlotOut.end(); }

  bool isVASKilled(const ValueAtSlot *VAS) const;

  // Get the distance (in cycles) from the define slot of the VAS to this slot.
  unsigned getCyclesFromDef(ValueAtSlot *VAS) const {
    return getLiveIn(VAS).getCycles();
  }

  unsigned isFromOtherBB(ValueAtSlot *VAS) const {
    return getLiveIn(VAS).isFromOtherBB();
  }

  // Get Slot pointer.
  const VASTSlot *getSlot() { return S; }

  void print(raw_ostream &OS) const;
  void dump() const;
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
  typedef mapped_iterator<VASMapTy::const_iterator, vas_getter>
          const_vas_iterator;

  BumpPtrAllocator Allocator;

  // define VAS assign iterator.
  typedef VASTRegister::assign_itertor assign_it;

public:
  static char ID;

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

  const_vas_iterator vas_begin() const {
    return const_vas_iterator(UniqueVASs.begin(),
                        vas_getter(pair_second<std::pair<VASTValue*, VASTSlot*>,
                                               ValueAtSlot*>));
  }

  const_vas_iterator vas_end() const {
    return const_vas_iterator(UniqueVASs.end(),
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

  bool addLiveIns(SlotInfo *From, SlotInfo *To, bool OnlyUndefTiming);
  bool addLiveInFromAliasSlots(VASTSlot *From, SlotInfo *To, VASTModule *VM);

  // Using the reaching definition algorithm to sort out the ultimate
  // relationship of registers.
  // Dirty hack: maybe there are two same statements is a slot, and we can use
  // bit vector to implement the algorithm similar to the compiler principle.
  void ComputeReachingDefinition(VASTModule *VM);

  // collect the Generated and Killed statements of the slot.
  void ComputeGenAndKill();

  void verifyRTLDependences() const;

  void viewGraph();

  void releaseMemory() {
    UniqueVASs.clear();
    Allocator.Reset();
    SlotVec.clear();
    SlotInfos.clear();
  }

  void getAnalysisUsage(AnalysisUsage &AU) const {
    MachineFunctionPass::getAnalysisUsage(AU);
    AU.setPreservesAll();
  }

  bool runOnMachineFunction(MachineFunction &MF);

  RtlSSAAnalysis();
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
