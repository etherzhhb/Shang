//===-- MemOpsFusing - Fuse MemOps to Boost Speed Performance  --*- C++ -*-===//
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
// This file implement the MemOpsFusing Pass, which fully utilize the bandwidth
// of memory bus to boost the speed performance of the design.
//
// You can find the original description of the clique partitioning algorithm
// in paper:
//   New Efficient Clique Partitioning Algorithms for Register-Transfer Synthesis
//   of Data Paths
//   Jong Tae Kim and Dong Ryeol Shin, 2001
//===----------------------------------------------------------------------===//
#include "CompGraphTraits.h"
#include "CompGraph.h"
#include "CompGraphDOT.h"

#include "vtm/Passes.h"
#include "vtm/VerilogBackendMCTargetDesc.h"
#include "vtm/VInstrInfo.h"
#include "vtm/Utilities.h"

#include "llvm/Analysis/ScalarEvolution.h"
#include "llvm/Analysis/ScalarEvolutionExpressions.h"
#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/ADT/PointerIntPair.h"
#include "llvm/ADT/SetOperations.h"
#include "llvm/ADT/Statistic.h"
#define DEBUG_TYPE "vtm-memop-fusing"
#include "llvm/Support/Debug.h"

using namespace llvm;

STATISTIC(MemOpFused, "Number of memory operations fused");
namespace {
struct AccessInfo {
  typedef PointerIntPair<const Value*, 1, bool> BaseAddrPtrTy;
  /*const */MachineInstr *Inst;
  /*const */unsigned Id;

  operator MachineInstr*() const { return Inst; }
  MachineInstr *operator->() const { return Inst; }

  MachineMemOperand *getAddr() const { return *Inst->memoperands_begin(); }

  uint64_t getSize() const { return getAddr()->getSize(); }
  uint64_t getAlignment() const { return getAddr()->getAlignment(); }
  bool isStore() const { return getAddr()->isStore(); }

  AccessInfo(MachineInstr *RHS = 0, bool isDummy = false)
    : Inst(RHS), Id(isDummy ? 0 : ~0) {}

  AccessInfo(MachineInstr *MI, unsigned Id)
    : Inst(MI), Id(Id) {}

  bool operator==(const AccessInfo &RHS) const {
    return Inst == RHS.Inst;
  }

  bool operator!=(const AccessInfo &RHS) const { return !operator==(RHS); }

  static AccessInfo Create(MachineInstr *MI, unsigned Id) {
    assert(MI->getOpcode() == VTM::VOpMemTrans && "Unexpected opcode!");
    return AccessInfo(MI, Id);
  }
};
}

namespace llvm {
template<> struct DenseMapInfo<AccessInfo> {
  static inline AccessInfo getEmptyKey() {
    return AccessInfo(DenseMapInfo<MachineInstr*>::getEmptyKey(), true);
  }

  static inline AccessInfo getTombstoneKey() {
    return AccessInfo(DenseMapInfo<MachineInstr*>::getTombstoneKey(), true);
  }

  static unsigned getHashValue(const AccessInfo &Val) {
    return DenseMapInfo<MachineInstr*>::getHashValue(Val);
  }

  static bool isEqual(const AccessInfo &LHS, const AccessInfo &RHS) {
    return DenseMapInfo<MachineInstr*>::isEqual(LHS, RHS);
  }
};

template<> struct CompGraphTraits<AccessInfo> {
  static bool isEarlier(const AccessInfo &LHS, const AccessInfo &RHS) {
    return LHS.Id < RHS.Id;
  }

  static bool compatible(AccessInfo LHS, AccessInfo RHS) {
    return LHS.isStore() == RHS.isStore();
  }

  static bool isTrivial(const AccessInfo &LHS) {
    return LHS.Inst == 0;
  }

  static std::string getNodeLabel(const AccessInfo &LHS) {
    std::string Str;
    raw_string_ostream ss(Str);
    ss << **LHS->memoperands_begin();
    return ss.str();
  }
};
}

namespace {
typedef CompGraph<AccessInfo> MemOpCompGraph;

class InstGraphBase {
public:
  typedef SmallPtrSet<MachineInstr*, 8> InstSetTy;
  typedef DenseMap<MachineInstr*, InstSetTy> InstGraphTy;
protected:
  InstGraphTy Graph;

public:
  bool IsConnected(MachineInstr *SrcMI, MachineInstr *DstMI) const {
    InstGraphTy::const_iterator at = Graph.find(DstMI);
    return at == Graph.end() ? false : at->second.count(SrcMI);
  }

  bool IsDstDepsConnectedToSrc(MachineInstr *SrcMI, MachineInstr *DstMI) const {
    InstGraphTy::const_iterator at = Graph.find(DstMI);

    if (at == Graph.end()) return false;

    const InstSetTy DepSet = at->second;
    typedef InstSetTy::const_iterator iterator;
    for (iterator I = DepSet.begin(), E = DepSet.end(); I != E; ++I)
      if (IsConnected(SrcMI, *I)) return true;

    return false;
  }
};

struct MemDepGraph : public InstGraphBase {
  ScalarEvolution &SE;
  AliasAnalysis &AA;

  MemDepGraph(Pass *P)
    : SE(P->getAnalysis<ScalarEvolution>()), AA(P->getAnalysis<AliasAnalysis>())
  {}

  void addMemInstr(MachineInstr *Inst) {
    unsigned Opcode = Inst->getOpcode();
    bool ForceDstDep =  Opcode == VTM::VOpInternalCall;
    bool IsDstMemTrans = Opcode == VTM::VOpMemTrans;
    IsDstMemTrans |= Opcode == VTM::VOpBRAMTrans;

    if (!IsDstMemTrans && !ForceDstDep) return;

    InstSetTy &DepSet = Graph[Inst];

    // Already visited.
    if (!DepSet.empty()) return;

    bool DstMayStore = VInstrInfo::mayStore(Inst);
    MachineMemOperand *DstMO = IsDstMemTrans ? *Inst->memoperands_begin() : 0;

    typedef MachineBasicBlock::instr_iterator it;
    MachineBasicBlock *MBB = Inst->getParent();
    for (it I = MBB->instr_begin(), E = Inst; I != E; ++I) {
      MachineInstr *SrcMI = I;
      // Do not add loop to dependent graph.
      if (SrcMI == Inst) continue;

      unsigned SrcOpcode = SrcMI->getOpcode();
      if (SrcOpcode != VTM::VOpMemTrans && SrcOpcode != VTM::VOpBRAMTrans
          && SrcOpcode != VTM::VOpInternalCall)
        continue;

      if (VInstrInfo::isPredicateMutex(SrcMI, Inst)) continue;

      // Handle force dependency.
      if (ForceDstDep || SrcOpcode == VTM::VOpInternalCall) {
        DepSet.insert(SrcMI);
        continue;
      }

      bool SrcMayStore = VInstrInfo::mayStore(SrcMI);

      // Ignore RAR dependency.
      if (!SrcMayStore && ! DstMayStore) continue;

      MachineMemOperand *SrcMO = *SrcMI->memoperands_begin();

      // Is DstMI depends on SrcMI?
      if (isMachineMemOperandAlias(SrcMO, DstMO, &AA, &SE))
        DepSet.insert(SrcMI);
    }
  }

  void mergeDepSet(MachineInstr *From, MachineInstr *To) {
    InstGraphTy::iterator at = Graph.find(From);

    assert(at != Graph.end() && "From not exist!");

    // Merge the set.
    InstSetTy &ToDepSet = Graph[To];
    ToDepSet.insert(at->second.begin(), at->second.end());
    ToDepSet.erase(From);

    Graph.erase(at);
  }
};

// Transitive Closure of dependent relation
struct UseTransClosure : public InstGraphBase {
  MemDepGraph MDG;
  MachineBasicBlock &MBB;
  MachineRegisterInfo &MRI;

  UseTransClosure(Pass *P, MachineBasicBlock &MBB)
    : MDG(P), MBB(MBB), MRI(MBB.getParent()->getRegInfo()) {}

  void addMemInstr(MachineInstr *Inst) {
    MDG.addMemInstr(Inst);
  }

  void mergeMemDepSet(MachineInstr *From, MachineInstr *To) {
    MDG.mergeDepSet(From, To);
  }

  void clearUseCache() { Graph.clear(); }

  bool isIdenticalMemTrans(MachineInstr *LHS, MachineInstr *RHS) {
    if (LHS->getOpcode() != VTM::VOpMemTrans) return false;
    if (RHS->getOpcode() != VTM::VOpMemTrans) return false;

    for (unsigned i = 1, e = VInstrInfo::mayStore(LHS) ? 5 : 4; i < e; ++i)
      if (!LHS->getOperand(i).isIdenticalTo(RHS->getOperand(i)))
        return false;

    return true;
  }

  bool trackUsesOfSrc(InstSetTy &UseSet, MachineInstr *Src, MachineInstr *Dst) {
    assert(!Src->isPHI() && !Dst->isPHI() && "Unexpected PHI!");

    if (UseSet.count(Dst))
      return true;

    // Check the memory dependence, note that we can move the identical accesses
    // across each other.
    if ((MDG.IsConnected(Src, Dst) && !isIdenticalMemTrans(Dst, Src))
        || MDG.IsDstDepsConnectedToSrc(Src, Dst)) {
      // Src is (indirectly) used by Dst.
      UseSet.insert(Dst);
      return true;
    }

    // Iterate from use to define.
    for (unsigned i = 0, e = Dst->getNumOperands(); i != e; ++i) {
      const MachineOperand &MO = Dst->getOperand(i);

      // Only care about a use register.
      if (!MO.isReg() || MO.isDef() || MO.getReg() == 0)
        continue;

      unsigned SrcReg = MO.getReg();
      MachineInstr *DefMI = MRI.getVRegDef(SrcReg);
      if (DefMI->getParent() != &MBB || DefMI == Dst || DefMI->isPHI())
        continue;

      if (DefMI == Src || UseSet.count(DefMI)) {
        // Src is (indirectly) used by Dst.
        UseSet.insert(Dst);
        return true;
      }
    }

    return false;
  }

  bool trackUsesOfSrc(MachineInstr *Src, MachineInstr *Dst) {
    return trackUsesOfSrc(Graph[Src], Src, Dst);
  }

  int operator()(AccessInfo LHS, AccessInfo RHS) {
    MachineMemOperand *LHSAddr = LHS.getAddr(), *RHSAddr = RHS.getAddr();

    const SCEVConstant *DeltaSCEV
      = dyn_cast<SCEVConstant>(getAddressDeltaSCEV(RHSAddr, LHSAddr, &MDG.SE));

    // Cannot fuse two memory access with unknown distance.
    if (!DeltaSCEV) return CompGraphWeights::HUGE_NEG_VAL;

    int64_t Delta = DeltaSCEV->getValue()->getSExtValue();
    // Make sure LHS is in the lower address.
    if (Delta < 0) {
      Delta = -Delta;
      std::swap(LHS, RHS);
    }

    assert(LHSAddr->isStore() == RHSAddr->isStore()
           && "Unexpected different access type!");
    uint64_t FusedWidth = std::max(LHS.getSize(), Delta + RHS.getSize());

    // Do not generate unaligned memory access.
    if (FusedWidth > LHS.getAlignment()) return CompGraphWeights::HUGE_NEG_VAL;

    if (LHSAddr->isStore()) {
      // Cannot store with irregular byteenable at the moment.
      if (!isPowerOf2_64(FusedWidth)) return CompGraphWeights::HUGE_NEG_VAL;

      // For the stores, we must make sure the higher address is just next to
      // the lower address.
      if (Delta && uint64_t(Delta) != LHS.getSize())
        return CompGraphWeights::HUGE_NEG_VAL;

      // LHS and RHS have the same address.
      if (Delta == 0 &&
        // Need to check if the two access are writing the same data, and writing
        // the same size.
          (!LHS->getOperand(2).isIdenticalTo(RHS->getOperand(2))
            || LHSAddr->getSize() != RHSAddr->getSize()))
          return CompGraphWeights::HUGE_NEG_VAL;
    }
    
    uint64_t BusWidth = getFUDesc<VFUMemBus>()->getDataWidth() / 8;

    // Don't exceed the width of data port of MemBus.
    if (FusedWidth > BusWidth) return CompGraphWeights::HUGE_NEG_VAL;

    // Ensure LHS is before than RHS in execution order.
    if (RHS.Id < LHS.Id) std::swap(LHS, RHS);

    // Is there any dependence between LHS and RHS?
    typedef MachineBasicBlock::instr_iterator it;
    for (it I = llvm::next(it(LHS)), E = it(RHS); I != E; ++I)
      trackUsesOfSrc(LHS, I);

    if (trackUsesOfSrc(LHS, RHS)) return CompGraphWeights::HUGE_NEG_VAL;

    return (2 * BusWidth - FusedWidth) * CompGraphWeights::TINY_VAL;
  }
};

struct MemOpsFusing : public MachineFunctionPass {
  static char ID;
  ScalarEvolution *SE;
  MachineRegisterInfo *MRI;
  const TargetInstrInfo *TII;

  MemOpsFusing() : MachineFunctionPass(ID), SE(0), MRI(0), TII(0) {}

  void getAnalysisUsage(AnalysisUsage &AU) const {
    AU.addRequired<ScalarEvolution>();
    AU.addPreserved<ScalarEvolution>();
    AU.addRequired<AliasAnalysis>();
    AU.addPreserved<AliasAnalysis>();
    MachineFunctionPass::getAnalysisUsage(AU);
  }

  bool runOnMachineFunction(MachineFunction &MF) {
    bool changed = false;
    SE = &getAnalysis<ScalarEvolution>();
    MRI = &MF.getRegInfo();
    TII = MF.getTarget().getInstrInfo();

    for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I)
      changed |= runOnMachineBasicBlock(*I);

    MF.verify(this, "After Memory operations fusion.");

    return changed;
  }

  bool runOnMachineBasicBlock(MachineBasicBlock &MBB);

  void updateQ(MemOpCompGraph::NodeTy *N, MemOpCompGraph::NodeTy *P,
               MemOpCompGraph::NodeTy *&Q, unsigned &MaxCommonNeighbors);

  MemOpCompGraph::NodeTy *getNeighborToCombine(MemOpCompGraph::NodeTy *P,
                                               MemOpCompGraph &MemOps);
  bool fuseMemOp(MemOpCompGraph &MemOps, UseTransClosure &UseClosure);

  void moveUsesAfter(AccessInfo &MergeFrom, AccessInfo &MergeTo,
                     UseTransClosure &UseClosure, MemOpCompGraph &MemOps);

  void fuseMachineInstr(MachineInstr *From, MachineInstr *To);
};
}

char MemOpsFusing::ID = 0;

bool MemOpsFusing::runOnMachineBasicBlock(MachineBasicBlock &MBB) {
  MemOpCompGraph MemOps;
  unsigned Id = 0;
  UseTransClosure UseClosure(this, MBB);

  // 1. Collect the fusing candidates.
  typedef MachineBasicBlock::instr_iterator instr_it;
  for (instr_it I = MBB.instr_begin(), E = MBB.instr_end(); I != E; ++I) {
    UseClosure.addMemInstr(I);

    // Only fuse the accesses via memory bus
    if (I->getOpcode() != VTM::VOpMemTrans)
      continue;

    AccessInfo Info = AccessInfo::Create(I, ++Id);

    MemOps.GetOrCreateNode(Info);
  }

  // Nothing to merge.
  if (!MemOps.hasMoreThanOneNode()) return false;

  bool Changed = false;
  Changed |= fuseMemOp(MemOps, UseClosure);

  return Changed;
}

bool MemOpsFusing::fuseMemOp(MemOpCompGraph &MemOps,
                             UseTransClosure &UseClosure) {
  typedef MemOpCompGraph::iterator allnodes_it;
  typedef MemOpCompGraph::NodeTy NodeTy;
  bool changed = false;

  for(;;) {
    MemOps.recomputeCompatibility();
    MemOps.updateEdgeWeight(UseClosure);

    // 1. Pick a node with neighbor weight and call it P.
    NodeTy *P = 0;
    // Initialize MaxNeighborWeight to a nozero value so we can ignore the
    // trivial nodes.
    int MaxNeighborWeight = 1;

    for (allnodes_it I = MemOps.begin(), E = MemOps.end(); I != E; ++I) {
      MemOpCompGraph::NodeTy *N = *I;
      // Ignore the Nodes that has only virtual neighbors.
      if (MemOpCompGraph::Traits::isTrivial(N->get())) continue;

      int CurrentNeighborWeight = N->computeNeighborWeight();

      // Do not update P if N has smaller neighbor weight.
      if (CurrentNeighborWeight < MaxNeighborWeight) continue;

      // If N and P have the same neighbor weight, pick the one with bigger Id.
      if (P && CurrentNeighborWeight == MaxNeighborWeight
          && N->get().Id < P->get().Id)
        continue;

      P = N;
      MaxNeighborWeight = CurrentNeighborWeight;
    }

    if (P == 0) break;

    DEBUG(MemOps.viewGraph());

    changed |= true;

    // If P has any no-virtual neighbor.
    while (P->degree() > 2) {
      DEBUG(
        typedef MachineBasicBlock::instr_iterator instr_it;
        dbgs() << "MemOp order:\n";
        for (instr_it I = UseClosure.MBB.instr_begin(),
             E = UseClosure.MBB.instr_end(); I != E; ++I)
          if (NodeTy *N = MemOps[AccessInfo(I)])
            dbgs() << '[' << N->get().Id << ']' << *I;
      );

      typedef NodeTy::iterator neighbor_it;
      NodeTy *NodeToMerge = getNeighborToCombine(P, MemOps);

      // Combine P and Q and call it P, Make sure Q is before P.
      if (!MemOpCompGraph::Traits::isEarlier(NodeToMerge->get(), P->get())) {
        std::swap(P, NodeToMerge);
      }
      P->deleteUncommonEdges(NodeToMerge);

      // Now the instruction "MergeFrom" is before the instruction "MergeTo".
      AccessInfo &MergeFrom = NodeToMerge->get(), &MergeTo = P->get();
      assert(MergeFrom.Id < MergeTo.Id && "Bad merge order!");

      // Move all use of "MergeFrom" between "MergeFrom" and "MergeTo" after
      // "MergeTo", and update the ID of the memory access instructions.
      moveUsesAfter(MergeFrom, MergeTo, UseClosure, MemOps);

      // Merge LHS into RHS.
      fuseMachineInstr(MergeFrom.Inst, MergeTo.Inst);

      // Update the use closure.
      UseClosure.mergeMemDepSet(MergeFrom.Inst, MergeTo.Inst);

      // Remove LHS.
      MergeFrom.Inst->eraseFromParent();
      MemOps.deleteNode(NodeToMerge);
    }
  }

  return changed;
}

void MemOpsFusing::moveUsesAfter(AccessInfo &MergeFrom, AccessInfo &MergeTo,
                                 UseTransClosure &UseClosure,
                                 MemOpCompGraph &MemOps) {
  MachineInstr *MergeFromMI = MergeFrom, *MergeToMI = MergeTo;
  assert(MergeFromMI && MergeToMI && "Unexpected virtual node!");
  unsigned CurId = MergeFrom.Id;

  DEBUG(dbgs() << "Going to merge:\n[" << MergeFrom.Id << ']'
        << *MergeFromMI << "into\n[" << MergeTo.Id << ']'
        << *MergeToMI);

  typedef MachineBasicBlock::instr_iterator instr_it;
  instr_it L = MergeFromMI, InsertPos = MergeToMI;

  // Skip MergeFromMI.
  ++L;
  // Insert moved instruction after MergeToMI.
  ++InsertPos;

  typedef MemOpCompGraph::NodeTy NodeTy;
  SmallVector<NodeTy*, 8> NeedToUpdateID;
  UseTransClosure::InstSetTy InstSet;

  while (L != instr_it(MergeToMI)) {
    assert(L != UseClosure.MBB.instr_end()
           && "Iterator pass the end of the list!");
    MachineInstr *MIToMove = L++;
    NodeTy *N = MemOps[AccessInfo(MIToMove)];

    DEBUG(if (N) dbgs() << "Get [" << N->get().Id << ']' << *MIToMove;);

    assert((N == 0 || N->get().Id < MergeTo.Id) && "MemOp has bad order!");

    if (!UseClosure.trackUsesOfSrc(InstSet, MergeFromMI, MIToMove)) {
      if (N) N->get().Id = ++CurId;
      continue;
    }

    // We cannot determinate the id of nodes that moved after MergeToMI
    // right now, we need to update their id after all nodes are moved.
    if (N) NeedToUpdateID.push_back(N);
    MIToMove->removeFromParent();
    UseClosure.MBB.insert(InsertPos, MIToMove);
  }

  MergeTo.Id = ++CurId;
  // Update the moved nodes' ID.
  for (unsigned i = 0, e = NeedToUpdateID.size(); i != e; ++i)
    NeedToUpdateID[i]->get().Id = ++CurId;
}



void MemOpsFusing::fuseMachineInstr(MachineInstr *From, MachineInstr *To) {
  assert(From->getOpcode() == VTM::VOpMemTrans
         && To->getOpcode() == VTM::VOpMemTrans
         && "Unexpected type of MachineInstr to merge!!");

  // Get the new address, i.e. the lower address which has a bigger
  // alignment.
  unsigned LowerReg = From->getOperand(0).getReg();
  MachineOperand LowerAddr = From->getOperand(1);
  MachineOperand LowerData = From->getOperand(2);
  MachineMemOperand *LowerMemOp = *From->memoperands_begin();
  unsigned LowerByteEn = getBitSlice64(From->getOperand(4).getImm(), 8);
  LowerAddr.clearParent();
  LowerData.clearParent();
  unsigned HigherReg = To->getOperand(0).getReg();
  MachineOperand HigherAddr = To->getOperand(1);
  MachineOperand HigherData = To->getOperand(2);
  MachineMemOperand *HigherMemOp = *To->memoperands_begin();
  unsigned HigherByteEn = getBitSlice64(To->getOperand(4).getImm(), 8);
  HigherAddr.clearParent();
  HigherData.clearParent();
  int64_t Delta = getAddressDelta(HigherMemOp, LowerMemOp, SE);
  // Make sure lower address is actually lower.
  if (Delta < 0) {
    Delta = - Delta;
    std::swap(LowerAddr, HigherAddr);
    std::swap(LowerData, HigherData);
    std::swap(LowerMemOp, HigherMemOp);
    std::swap(LowerReg, HigherReg);
    std::swap(LowerByteEn, HigherByteEn);
  }

  int64_t NewSize = std::max(LowerMemOp->getSize(),
                             Delta + HigherMemOp->getSize());
  NewSize = NextPowerOf2(NewSize - 1);
  MachineBasicBlock *CurMBB = To->getParent();
  MachineFunction *MF = CurMBB->getParent();
  // Get the new address from the lower address.
  MachineMemOperand **NewMO = MF->allocateMemRefsArray(1);
  NewMO[0] = MF->getMachineMemOperand(LowerMemOp, 0, NewSize);
  To->setMemRefs(NewMO, NewMO + 1);

  // Get the Byte enable.
  unsigned ByteEn = getByteEnable(NewSize);
  assert((((LowerByteEn | (HigherByteEn << Delta)) == ByteEn)
         || !NewMO[0]->isStore()) && "New Access writing extra bytes!");
  (void) LowerByteEn;
  (void) HigherByteEn;

  assert((!NewMO[0]->isStore()
          || From->getOperand(3).isIdenticalTo(To->getOperand(3)))
         && "Cannot mixing load and store!");

  // Merge the predicate and get the trace operand.
  MachineOperand *FromPred = VInstrInfo::getPredOperand(From),
                 *ToPred = VInstrInfo::getPredOperand(To);

  // Build the new data to store by concatenating them together.
  if (NewMO[0]->isStore()) {
    unsigned NewReg = MRI->createVirtualRegister(&VTM::DRRegClass);
    unsigned NewDataSizeInBit = VInstrInfo::getBitWidth(LowerData)
                                + VInstrInfo::getBitWidth(HigherData);
    BuildMI(*CurMBB, To, DebugLoc(), VInstrInfo::getDesc(VTM::VOpBitCat))
      .addOperand(VInstrInfo::CreateReg(NewReg, NewDataSizeInBit, true))
      .addOperand(HigherData).addOperand(LowerData)
      .addOperand(VInstrInfo::CreatePredicate())
      .addOperand(VInstrInfo::CreateTrace());

    LowerData = VInstrInfo::CreateReg(NewReg, NewDataSizeInBit);
  }

  unsigned NewPred =
    VInstrInfo::MergePred(*FromPred, *ToPred, *To->getParent(),
                          To, MRI, TII, VTM::VOpOr).getReg();


  if (!FromPred[1].isIdenticalTo(ToPred[1]))
    VInstrInfo::ResetTrace(To);

  MachineOperand TraceOperand = *VInstrInfo::getTraceOperand(To);
  TraceOperand.clearParent();

  // Refresh the machine operand.
  To->RemoveOperand(6);
  To->RemoveOperand(5);
  To->RemoveOperand(4);
  To->RemoveOperand(3);
  To->RemoveOperand(2);
  To->RemoveOperand(1);

  // Add the new address operand.
  To->addOperand(LowerAddr);
  // Add the data to store.
  To->addOperand(LowerData);
  To->addOperand(VInstrInfo::CreateImm(NewMO[0]->isStore(), 1));
  To->addOperand(VInstrInfo::CreateImm(ByteEn, 8));
  To->addOperand(VInstrInfo::CreatePredicate(NewPred));
  To->addOperand(TraceOperand);

  // Update the result registers.
  if (NewMO[0]->isLoad()) {
    const TargetRegisterClass *RC = MRI->getRegClass(LowerReg);
    // Get the new higher data from the higher part of the result of the fused
    // memory access.
    unsigned NewHigherReg = MRI->createVirtualRegister(RC);
    unsigned HigherRegSizeInBits = HigherMemOp->getSize() * 8;
    unsigned DataSizeInBit = getFUDesc<VFUMemBus>()->getDataWidth() ;
    unsigned LB = Delta * 8;
    unsigned UB = LB + HigherRegSizeInBits;
    // Insert the bitslice after the load.
    MachineBasicBlock::instr_iterator IP = To;
    ++IP;
    BuildMI(*CurMBB, IP, DebugLoc(), VInstrInfo::getDesc(VTM::VOpBitSlice))
      .addOperand(VInstrInfo::CreateReg(NewHigherReg, HigherRegSizeInBits, true))
      .addOperand(VInstrInfo::CreateReg(LowerReg, DataSizeInBit))
      .addOperand(VInstrInfo::CreateImm(UB, 8))
      .addOperand(VInstrInfo::CreateImm(LB, 8))
      .addOperand(VInstrInfo::CreatePredicate())
      .addOperand(VInstrInfo::CreateTrace());

    // Update the result register.
    MRI->replaceRegWith(HigherReg, NewHigherReg);
    To->getOperand(0).ChangeToRegister(LowerReg, true);
  }
  
  ++MemOpFused;
}

MemOpCompGraph::NodeTy *
MemOpsFusing::getNeighborToCombine(MemOpCompGraph::NodeTy *P,
                                   MemOpCompGraph &MemOps) {
  typedef MemOpCompGraph::NodeTy NodeTy;
  typedef NodeTy::iterator neighbor_it;

  unsigned MaxCommonNeighbors = 0;
  MemOpCompGraph::NodeTy *Q = 0;

  for (neighbor_it I = P->pred_begin(), E = P->pred_end(); I != E; ++I) {
    if (MemOpCompGraph::Traits::isTrivial((*I)->get())) continue;

    updateQ(*I, P, Q, MaxCommonNeighbors);
  }

  for (neighbor_it I = P->succ_begin(), E = P->succ_end(); I != E; ++I) {
    if (MemOpCompGraph::Traits::isTrivial((*I)->get())) continue;

    updateQ(*I, P, Q, MaxCommonNeighbors);
  }

  assert(Q && Q->get() && "Unexpected Q is null!");

  return Q;
}

void MemOpsFusing::updateQ(MemOpCompGraph::NodeTy *N, MemOpCompGraph::NodeTy *P,
                           MemOpCompGraph::NodeTy *&Q, unsigned &MaxCommon) {
  unsigned NCommonNeighbors = N->computeNeighborWeight(P);
  // 2. Pick a neighbor of p, q, such that the number of common neighbor is
  //    maximum
  if (NCommonNeighbors == 0 || NCommonNeighbors < MaxCommon) return;

  // Tie-breaking: Select q such that the node degree of q is minimum.
  if (Q && NCommonNeighbors == MaxCommon && Q->degree() < N->degree())
    return;

  // If they have the same neighbors weight, pick the node with bigger Id.
  if (Q && Q->degree() < P->degree() && Q->get().Id > N->get().Id) return;

  MaxCommon = NCommonNeighbors;
  Q = N;
}

Pass *llvm::createMemOpsFusingPass() {
  return new MemOpsFusing();
}
