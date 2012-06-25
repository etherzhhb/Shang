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
//===----------------------------------------------------------------------===//
#include "CompGraphTraits.h"
#include "CompGraph.h"
#include "CompGraphDOT.h"

#include "vtm/Passes.h"
#include "vtm/VerilogBackendMCTargetDesc.h"
#include "vtm/VInstrInfo.h"
#include "vtm/Utilities.h"

#include "llvm/Analysis/ScalarEvolution.h"
#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineInstr.h"
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
  const Value *BaseAddr;
  /*const */int64_t Offset;
  /*const */int64_t Size;
  /*const */unsigned Alignment;
  /*const */unsigned Id : 31;
  /*const */bool     IsWrite : 1;

  operator MachineInstr*() const { return Inst; }
  MachineInstr *operator->() const { return Inst; }

  BaseAddrPtrTy getBaseAddr() const {
    return BaseAddrPtrTy(BaseAddr, IsWrite);
  }

  AccessInfo(MachineInstr *RHS = 0, bool isDummy = false)
    : Inst(RHS), Offset(0), Size(0), Alignment(0), Id(0),
    IsWrite((!isDummy && RHS) ? VInstrInfo::mayStore(RHS) : false) {}

  AccessInfo(MachineInstr *MI, bool IsWrite, const Value *BaseAddr,
             int64_t Offset, int64_t Size, unsigned Alignment, unsigned Id)
    : Inst(MI), BaseAddr(BaseAddr), Offset(Offset), Size(Size),
      Alignment(0), Id(Id), IsWrite(IsWrite) {}

  bool operator==(const AccessInfo &RHS) const {
    return Inst == RHS.Inst;
  }

  bool operator!=(const AccessInfo &RHS) const { return !operator==(RHS); }

  static AccessInfo Create(MachineInstr *MI, unsigned Id) {
    assert(MI->getOpcode() == VTM::VOpMemTrans && "Unexpected opcode!");
    bool IsWrite = VInstrInfo::mayStore(MI);

    MachineMemOperand *MemOperand = *MI->memoperands_begin();
    const Value *BaseAddr;
    int64_t Offset;
    tie(BaseAddr, Offset) = extractPointerAndOffset(MemOperand->getValue(),
                                                    MemOperand->getOffset());

    int64_t Size = MemOperand->getSize();
    return AccessInfo(MI, IsWrite, BaseAddr, Offset, Size,
                      MemOperand->getAlignment(), Id);
  }

  static int64_t getRangeBetween(const AccessInfo LHS, const AccessInfo RHS) {
    return std::max(LHS.Offset - RHS.Offset + RHS.Size, LHS.Size);
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
    if (!LHS || !RHS) return true;

    // Don't fuse read/write together.
    if (LHS.getBaseAddr() != RHS.getBaseAddr())
      return false;

    // Make sure LHS is Earlier than RHS.
    if (isEarlier(RHS, LHS)) std::swap(LHS, RHS);

    // Don't exceed the width of data port of MemBus.
    int64_t FusedWidth = AccessInfo::getRangeBetween(RHS, LHS);

    // LHS and RHS have the same address.
    if (FusedWidth == std::max(LHS.Size, RHS.Size)) {
      // Need to check if the two access are writing the same data, and writing
      // The same size.
      if (LHS.IsWrite)
        return LHS->getOperand(2).isIdenticalTo(RHS->getOperand(2))
               && LHS->getOperand(4).isIdenticalTo(RHS->getOperand(4));

      // Two operation reading the same address are always compatible.
      return true;
    }

    // if (FusedWidth <= getFUDesc<VFUMemBus>()->getDataWidth() / 8)
    //  return true;

    return false;
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
    for (it I = MBB->instr_begin(), E = MBB->instr_end(); I != E; ++I) {
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
    Graph[To].insert(at->second.begin(), at->second.end());

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

    for (unsigned i = 1, e = VInstrInfo::mayStore(LHS) ? 5 : 4; i < 5; ++i)
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
    if (MDG.IsConnected(Src, Dst) && !isIdenticalMemTrans(Dst, Src)) {
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
    // Do not merge the Instruction with difference base address.
    if (LHS.getBaseAddr() != RHS.getBaseAddr())
      return CompGraphWeights::HUGE_NEG_VAL;

    // Ensure LHS is before than RHS in execution order.
    if (RHS.Id < LHS.Id) std::swap(LHS, RHS);

    // Is there any dependence between LHS and RHS?
    typedef MachineBasicBlock::instr_iterator it;
    for (it I = llvm::next(it(LHS)), E = it(RHS); I != E; ++I)
      trackUsesOfSrc(LHS, I);

    return trackUsesOfSrc(LHS, RHS) ? CompGraphWeights::HUGE_NEG_VAL
                                    : CompGraphWeights::TINY_VAL;
  }
};

struct MemOpsFusing : public MachineFunctionPass {
  static char ID;

  MachineRegisterInfo *MRI;
  const TargetInstrInfo *TII;

  MemOpsFusing() : MachineFunctionPass(ID), MRI(0), TII(0) {}

  void getAnalysisUsage(AnalysisUsage &AU) const {
    AU.addRequired<ScalarEvolution>();
    AU.addPreserved<ScalarEvolution>();
    AU.addRequired<AliasAnalysis>();
    AU.addPreserved<AliasAnalysis>();
    MachineFunctionPass::getAnalysisUsage(AU);
  }

  bool runOnMachineFunction(MachineFunction &MF) {
    bool changed = false;
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
    MemOps.updateEdgeWeight(UseClosure);

    // 1. Pick a node with smallest degree and call it P.
    NodeTy *P = 0;
    int MaxNeighborWeight = 0;

    for (allnodes_it I = MemOps.begin(), E = MemOps.end(); I != E; ++I) {
      MemOpCompGraph::NodeTy *N = *I;
      // Ignore the Nodes that has only virtual neighbors.
      if (MemOpCompGraph::Traits::isTrivial(N->get())) continue;

      int CurrentNeighborWeight = N->computeNeighborWeight();

      if (CurrentNeighborWeight > MaxNeighborWeight) {
        P = N;
        MaxNeighborWeight = CurrentNeighborWeight;
      }
    }

    if (P == 0) break;

    DEBUG(MemOps.viewGraph());

    changed |= true;

    UseTransClosure::InstSetTy InstSet;
    SmallVector<NodeTy*, 8> NeedToUpdateID;

    typedef MachineBasicBlock::instr_iterator instr_it;

    // If P has any no-virtual neighbor.
    while (P->degree() > 2) {
      DEBUG(dbgs() << "MemOp order:\n";
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

      AccessInfo &MergeFrom = NodeToMerge->get(), &MergeTo = P->get();
      assert(MergeFrom.Id < MergeTo.Id && "Bad merge order!");
      unsigned OriginalMergeToId = MergeTo.Id;

      // Move all use between LHS and RHS after RHS.
      MachineInstr *MergeFromMI = MergeFrom, *MergeToMI = MergeTo;
      assert(MergeFromMI && MergeToMI && "Unexpected virtual node!");
      unsigned CurId = MergeFrom.Id;

      DEBUG(dbgs() << "Going to merge:\n[" << MergeFrom.Id << ']'
                   << *MergeFromMI << "into\n[" << MergeTo.Id << ']'
                   << *MergeToMI);
      // Move the user of LHSMI between LHSMI and RHSMI after RHSMI, and update
      // the ID of the memory access instructions.
      instr_it L = MergeFromMI, InsertPos = MergeToMI;

      // Skip LHSMI.
      ++L;
      // Insert moved instruction after RHSMI.
      ++InsertPos;

      InstSet.clear();
      NeedToUpdateID.clear();
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

        // We can determinate the id of nodes that moved after LHSMI, we need to
        // update their id after all nodes are moved.
        if (N) NeedToUpdateID.push_back(N);
        MIToMove->removeFromParent();
        UseClosure.MBB.insert(InsertPos, MIToMove);
      }

      MergeTo.Id = ++CurId;
      // Update the moved nodes' ID.
      for (unsigned i = 0, e = NeedToUpdateID.size(); i != e; ++i)
        NeedToUpdateID[i]->get().Id = ++CurId;

      // Merge LHS into RHS.
      fuseMachineInstr(MergeFromMI, MergeToMI);

      // Update the use closure.
      UseClosure.mergeMemDepSet(MergeFromMI, MergeToMI);

      // Remove LHS.
      MergeFromMI->eraseFromParent();
      MemOps.deleteNode(NodeToMerge);
    }
  }

  return changed;
}

void MemOpsFusing::fuseMachineInstr(MachineInstr *From, MachineInstr *To) {
  assert(From->getOpcode() == VTM::VOpMemTrans
         && To->getOpcode() == VTM::VOpMemTrans
         && "Unexpected type of MachineInstr to merge!!");

  // Check if the store data, read/write, and byte enable are identical.
  for (unsigned i = 2, e = VInstrInfo::mayStore(To) ? 5 : 4; i < e; ++i) {
    assert(From->getOperand(i).isIdenticalTo(To->getOperand(i))
           && "Can only merge identical Memory access at the moment!");
  }

  // Only merge the predicate.
  MachineOperand *FromPred = VInstrInfo::getPredOperand(From),
                 *ToPred = VInstrInfo::getPredOperand(To);

  unsigned NewPred =
    VInstrInfo::MergePred(*FromPred, *ToPred, *To->getParent(),
                          To, MRI, TII, VTM::VOpOr).getReg();

  ToPred->ChangeToRegister(NewPred, false);
  ToPred->setTargetFlags(1);
  // Update the byte enable.
  int64_t ByteEn = std::max(To->getOperand(4).getImm(),
                            From->getOperand(4).getImm());
  To->getOperand(4).ChangeToImmediate(ByteEn);

  if (!FromPred[1].isIdenticalTo(ToPred[1]))
    VInstrInfo::ResetTrace(To);

  // Use From instead of To.
  MRI->replaceRegWith(From->getOperand(0).getReg(), To->getOperand(0).getReg());
  ++MemOpFused;
}

MemOpCompGraph::NodeTy *
MemOpsFusing::getNeighborToCombine(MemOpCompGraph::NodeTy *P,
                                   MemOpCompGraph &MemOps) {
  typedef MemOpCompGraph::NodeTy NodeTy;
  typedef NodeTy::iterator neighbor_it;

  unsigned MaxCommonNeighbors = 0;
  MemOpCompGraph::NodeTy *Q = 0;
  // 2. Pick a neighbor of p, q, such that the number of common neighbor is
  //    maximum
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
  if (unsigned NCommonNeighbors = N->computeNeighborWeight(P)) {
    if (NCommonNeighbors > MaxCommon
        // Tie-breaking: Select q such that the node degree of q is minimum.
        || (NCommonNeighbors == MaxCommon
            && N->degree() < Q->degree())) {
      MaxCommon = NCommonNeighbors;
      Q = N;
    }
  }
}

Pass *llvm::createMemOpsFusingPass() {
  return new MemOpsFusing();
}
