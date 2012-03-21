//===- RtlSSAAnalysis.cpp - Analyse the dependency between registers - C++ --=//
//
// Copyright: 2011 by SYSU EDA Group. all rights reserved.
// IMPORTANT: This software is supplied to you by Hongbin Zheng in consideration
// of your agreement to the following terms, and your use, installation,
// modification or redistribution of this software constitutes acceptance
// of these terms.  If you do not agree with these terms, please do not use,
// install, modify or redistribute this software. You may not redistribute,
// install copy or modify this software without written permission from
// Hongbin Zheng.
//
//===----------------------------------------------------------------------===//
//
// This pass collect the slots information of register and map them into a map
// vector. then it will analyse dependency between registers.
//
//
//===----------------------------------------------------------------------===//

#include "RtlSSAAnalysis.h"
#include "vtm/Passes.h"
#include "vtm/VFInfo.h"

#include "llvm/Target/TargetData.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallPtrSet.h"
#include "llvm/ADT/SetOperations.h"
#include "llvm/Support/SourceMgr.h"
#define DEBUG_TYPE "vtm-rtl-ssa"
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
    ss << Node->getName();
    DEBUG(
      SlotInfo * SI = Graph->getSlotInfo(Node);
      SI->print(ss););

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
  ValueAtSlot *DstVAS;
  VASDepBuilder(RtlSSAAnalysis &RtlSSA, ValueAtSlot *V) : A(RtlSSA), DstVAS(V) {}

  void operator() (ArrayRef<VASTUse> PathArray) {
    VASTUse SrcUse = PathArray.back();
    if (VASTRegister *Src = dyn_cast_or_null<VASTRegister>(SrcUse.unwrap()))
      A.addVASDep(DstVAS, Src);
  }
};

void ValueAtSlot::print(raw_ostream &OS) const {
  OS << getValue()->getName() << '@' << getSlot()->getSlotNum()
    << "\t <= {";

  typedef VASCycMapTy::const_iterator it;
  for (it I = DepVAS.begin(), E = DepVAS.end(); I != E; ++I)
    OS << I->first->getName() << '[' << I->second.getCycles() << ']' << ',';

  OS << "}\n";
}

void ValueAtSlot::verify() const {
  typedef VASCycMapTy::const_iterator it;
  VASTSlot *UseSlot = getSlot();
  for (it I = DepVAS.begin(), E = DepVAS.end(); I != E; ++I) {
    VASTSlot *DefSlot = I->first->getSlot();
    LiveInInfo LI = I->second;
    if (DefSlot->getParentIdx() == UseSlot->getParentIdx() &&
        UseSlot->hasAliasSlot() && !LI.isFromOtherBB() &&
        LI.getCycles() > DefSlot->alias_ii())
      llvm_unreachable("Broken RTL dependence!");
  }
}

void ValueAtSlot::dump() const {
  print(dbgs());
}

void SlotInfo::dump() const {
  print(dbgs());
}

void SlotInfo::print(raw_ostream &OS) const {
  OS << S->getName() << "\nGen:\n";
  for (gen_iterator I = gen_begin(), E = gen_end(); I != E; ++I) {
    ValueAtSlot *VAS = *I;
    OS.indent(2) << VAS->getName() << "\n";
  }

  OS << "\n\nIn:\n";
  for (VASCycMapTy::const_iterator I = in_begin(), E = in_end(); I != E; ++I) {
    ValueAtSlot *VAS = I->first;
    OS.indent(2) << VAS->getName() << '[' << I->second.getCycles() << "]\n";
  }

  OS << "\n\n";
}

// Any VAS whose value is overwritten at this slot is killed.
bool SlotInfo::isVASKilled(const ValueAtSlot *VAS) const {
  return OverWrittenValue.count(VAS->getValue());
}

void SlotInfo::initOutSet() {
  // Build the initial out set ignoring the kill set.
  for (gen_iterator I = gen_begin(), E = gen_end(); I != E; ++I)
    SlotOut.insert(std::make_pair(*I, ValueAtSlot::LiveInInfo()));
}

RtlSSAAnalysis::RtlSSAAnalysis() : MachineFunctionPass(ID) {
  initializeRtlSSAAnalysisPass(*PassRegistry::getPassRegistry());
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

  ComputeReachingDefinition(VM);

  // Build the VAS dependence graph with reaching define information.
  buildVASGraph(VM);

  DEBUG(viewGraph());

  verifyRTLDependences();

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
       I != E; ++I) {
    VASTSlot *DefSlot = I->first->getSlot();
    ValueAtSlot *DefVAS = getValueASlot(DepReg, DefSlot);

    ValueAtSlot::LiveInInfo LI = UseSI->getLiveIn(DefVAS);

    // VAS is only depends on DefVAS if it can reach this slot.
    if (unsigned Distance = LI.getCycles())
      VAS->addDepVAS(DefVAS, LI);
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

void RtlSSAAnalysis::verifyRTLDependences() const {
  for (const_vas_iterator I = vas_begin(), E = vas_end(); I != E; ++I)
    (*I)->verify();
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
  VASTValue *DefValue = *DepTree;

  // If Define Value is immediate or symbol, skip it.
  if (!DefValue) return;

  // If the define Value is register, add the dependent VAS to the
  // dependentVAS.
  if (VASTRegister *DepReg = dyn_cast<VASTRegister>(DefValue)){
    addVASDep(VAS, DepReg);
    return;
  }

  VASDepBuilder B(*this, VAS);
  // If the define Value is wire, traverse the use tree to get the
  // ultimate registers.
  DepthFirstTraverseDepTree(DepTree, B);
}

bool RtlSSAAnalysis::addLiveIns(SlotInfo *From, SlotInfo *To,
                                bool OnlyUndefTiming) {
  bool Changed = false;
  typedef SlotInfo::vascyc_iterator it;
  unsigned CurBBIdx = To->getSlot()->getParentIdx();

  for (it II = From->out_begin(), IE = From->out_end(); II != IE; ++II) {
    ValueAtSlot *PredOut = II->first;
    // Are we only add live-ins with undefine timing?
    if (OnlyUndefTiming && ! PredOut->getValue()->isTimingUndef())
      continue;

    unsigned DefBBIdx = PredOut->getSlot()->getParentIdx();
    
    ValueAtSlot::LiveInInfo LI = II->second;
    // Increase the cycles by 1 after the value lives to next slot.
    LI.incCycles();

    // Trace the propagation path.
    LI.liveThroughOtherBB(DefBBIdx != CurBBIdx);

    Changed |= To->insertIn(PredOut, LI);
    // Do not let the killed VASs go out
    if (!To->isVASKilled(PredOut))
      // New out occur.
      Changed |= To->insertOut(PredOut, LI);
  }

  return Changed;
}

bool RtlSSAAnalysis::addLiveInFromAliasSlots(VASTSlot *From, SlotInfo *To,
                                             VASTModule *VM) {
  bool Changed = false;
  unsigned FromSlotNum = From->getSlotNum();

  for (unsigned i = From->alias_start(), e = From->alias_end(),
       ii = From->alias_ii(); i < e; i += ii) {
    SlotInfo * PredSI = getSlotInfo(VM->getSlot(i));
    if (i == FromSlotNum) continue;

    // From the view of signals with undefined timing, all alias slot is the
    // same slot.
    Changed |= addLiveIns(PredSI, To, i > FromSlotNum);
  }

  return Changed;
}

void RtlSSAAnalysis::ComputeReachingDefinition(VASTModule *VM) {
  ComputeGenAndKill();
  // TODO: Simplify the data-flow, some slot may neither define new VAS nor
  // kill any VAS.

  bool Changed = false;

  do {
    Changed = false;

    for (slot_vec_it I = SlotVec.begin(), E = SlotVec.end(); I != E; ++I) {
      VASTSlot *S =*I;
      assert(S && "Unexpected null slot!");

      SlotInfo *CurSI = getSlotInfo(S);

      // Compute the out set.
      typedef VASTSlot::pred_it pred_it;
      for (pred_it PI = S->pred_begin(), PE = S->pred_end(); PI != PE; ++PI) {
        VASTSlot *PredSlot = *PI;

        // No need to update the out set of Slot 0 according its incoming value.
        // It is the first slot of the FSM.
        if (S->getSlotNum() == 0 && PredSlot->getSlotNum() != 0) continue;

        SlotInfo *PredSI = getSlotInfo(PredSlot);

        Changed |= addLiveIns(PredSI, CurSI, false);

        if (PredSlot->getParentIdx() == S->getParentIdx() &&
            PredSlot->hasAliasSlot())
          Changed |= addLiveInFromAliasSlots(PredSlot, CurSI, VM);
      }
    }
  } while (Changed);
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
    SI->initOutSet();
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
