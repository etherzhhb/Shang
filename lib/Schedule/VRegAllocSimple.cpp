//===- VRegAllocSimple.cpp - Simple Register Allocation ---------*- C++ -*-===//
//
//                            The Verilog Backend
//
// Copyright: 2011 by Hongbin Zheng. all rights reserved.
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
// This file defines a simple register allocation pass for Verilog target
// machine.
//
//===----------------------------------------------------------------------===//

#include "CompGraphTraits.h"
#include "CompGraph.h"
#include "CompGraphDOT.h"

#include "vtm/VFInfo.h"
#include "vtm/VRegisterInfo.h"
#include "vtm/MicroState.h"
#include "vtm/Passes.h"

//Dirty Hack:
#include "llvm/../../lib/CodeGen/LiveIntervalUnion.h"
#include "llvm/../../lib/CodeGen/RegAllocBase.h"
#include "llvm/../../lib/CodeGen/VirtRegMap.h"

#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/Function.h"
#include "llvm/PassAnalysisSupport.h"
#include "llvm/CodeGen/CalcSpillWeights.h"
#include "llvm/CodeGen/EdgeBundles.h"
#include "llvm/CodeGen/LiveIntervalAnalysis.h"
#include "llvm/CodeGen/LiveStackAnalysis.h"
#include "llvm/CodeGen/MachineDominators.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineLoopInfo.h"
#include "llvm/CodeGen/MachineLoopRanges.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/RegAllocRegistry.h"
#include "llvm/CodeGen/RegisterCoalescer.h"
#include "llvm/Target/TargetOptions.h"
#include "llvm/ADT/SmallSet.h"
#include "llvm/ADT/Statistic.h"
#define DEBUG_TYPE "vtm-regalloc"
#include "llvm/Support/Debug.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/CommandLine.h"

using namespace llvm;

static RegisterRegAlloc VSimpleRegalloc("vsimple",
                                        "vtm-simple register allocator",
                                        createSimpleRegisterAllocator);

namespace {
struct VRASimple : public MachineFunctionPass,
                   public RegAllocBase {
  // DIRTY HACK: We need to init the PhysReg2LiveUnion again with correct
  // physics register number.
  LiveIntervalUnion::Allocator UnionAllocator;
  // Context.
  MachineFunction *MF;
  VFInfo *VFI;

  // Analysis
  LiveStacks *LS;

  // Register Compatibility Graph.
  typedef CompGraph<LiveInterval*> LICGraph;
  typedef CompGraphNode<LiveInterval*> LICGraphNode;

  typedef const std::vector<unsigned> VRegVec;

  // MUX size limit factor.
  static unsigned MuxSizeCost;

  unsigned UserTag;

  VRASimple();
  void init(VirtRegMap &vrm, LiveIntervals &lis);
  
  static char ID;

  void getAnalysisUsage(AnalysisUsage &AU) const;
  void releaseMemory();

  // Abstract functions from the base class.
  unsigned selectOrSplit(LiveInterval &, SmallVectorImpl<LiveInterval*> &) {
    return 0;
  }
  Spiller &spiller() { return *(Spiller*)0; }
  virtual float getPriority(LiveInterval *LI) { return LI->weight; }
  virtual void enqueue(LiveInterval *LI) {}
  virtual LiveInterval *dequeue() { return 0; }
  // End of Abstract functions

  LiveInterval *getInterval(unsigned RegNum) {
    if (MRI->reg_nodbg_empty(RegNum)) {
      //LIS->removeInterval(RegNum);
      return 0;
    }

    LiveInterval &LI = LIS->getInterval(RegNum);
    if (LI.empty()) {
      LIS->removeInterval(RegNum);
      return 0;
    }

    return &LI;
  }

  unsigned getBitWidthOf(unsigned RegNum) {
    unsigned BitWidth = 0;
    for (MachineRegisterInfo::def_iterator I = MRI->def_begin(RegNum);
      I != MachineRegisterInfo::def_end(); ++I) {
        ucOperand &DefOp = cast<ucOperand>(I.getOperand());
        BitWidth = std::max(BitWidth, DefOp.getBitWidth());
    }

    return BitWidth;
  }

  // Run the functor on each definition of the register, and return true
  // if any functor return true (this means we meet some unexpected situation),
  // return false otherwise.
  template<class DefFctor>
  bool iterateUseDefChain(unsigned Reg, DefFctor &F) const {
    for (MachineRegisterInfo::reg_iterator I = MRI->reg_begin(Reg),
         E = MRI->reg_end(); I != E; ++I)
      if (F(I)) return true;

    return false;
  }

  // commutable
  unsigned extractComBinOperand() { return 0; }

  void joinPHINodeIntervals();
  void mergeLI(LiveInterval *FromLI, LiveInterval *ToLI);

  // Pre-bound function unit binding functions.
  void bindMemoryBus();
  void bindBlockRam();
  void bindCalleeFN();

  // Compatibility Graph building.
  void buildCompGraph(LICGraph &G);

  template<class CompEdgeWeight>
  bool reduceCompGraph(LICGraph &G, CompEdgeWeight &C);

  void bindCompGraph(LICGraph &G);
  // We need handle to special case when binding adder.
  void bindAdders(LICGraph &G);

  bool runOnMachineFunction(MachineFunction &F);

  const char *getPassName() const {
    return "Verilog Backend Resource Binding Pass";
  }
};

struct WidthChecker {
  // The Maximum bit width of current register.
  unsigned CurMaxWidth;

  WidthChecker() : CurMaxWidth(0) {}

  void resetWidth() { CurMaxWidth = 0; }

  bool checkWidth(unsigned CurWidth) {
    //CurMaxWidth = std::max(CurMaxWidth, CurWidth);
    if (!CurMaxWidth)
      CurMaxWidth = CurWidth;
    else if (CurMaxWidth != CurWidth)
      // Do not merge regisrters with difference width.
      return false;

    return true;
  }

  unsigned getWidth() const { return CurMaxWidth; }
};

template<int NUMSRC>
struct SourceChecker {
  // All copy source both fu of the edge.
  SmallSet<unsigned, 4> Srcs[NUMSRC];
  unsigned SrcNum[NUMSRC];
  unsigned ExtraCost;
  // TODO: Timing cost.

  SourceChecker() {}

  void resetSrcs() {
    ExtraCost = 0;
    for (unsigned i = 0; i < NUMSRC; ++i) {
      Srcs[i].clear();
      SrcNum[i] = 0;
    }
  }

  template<int N>
  void addSrc(ucOperand &SrcOp) {
    if (SrcOp.isReg()) {
      Srcs[N].insert(SrcOp.getReg());
      ++SrcNum[N];
    } else if (SrcOp.isImm())
      ExtraCost += /*LUT Cost*/ VFUs::LUTCost;
    else
      ExtraCost += /*Reg Mux Cost Pre-bit*/ VFUs::MuxSizeCost;
  }

  template<int N>
  void removeSrcReg(unsigned Reg) {
    Srcs[N].erase(Reg);
  }

  template<int N>
  int getSrcMuxSize() const {
    return Srcs[N].size();
  }

  template<int N>
  int getSrcNum() const {
    return SrcNum[N];
  }

  template<int N>
  int getSavedSrcMuxSize() const {
    return getSrcNum<N>() - getSrcMuxSize<N>();
  }

  template<int N>
  int getSrcMuxCost() const {
    if (unsigned MuxSize = getSrcMuxSize<N>())
      return (MuxSize - 1) * /*Mux pre-port cost */VFUs::MuxSizeCost;

    return 0;
  }

  template<int N>
  int getSavedSrcMuxCost() const {
    return getSavedSrcMuxSize<N>() * /* Mux pre-port area cost */VFUs::MUXCost;
  }

  int getExtraCost() const { return ExtraCost; }
};

struct DstChecker {
  // All copy source both fu of the edge.
  SmallSet<unsigned, 8> Dsts;
  int NumDsts;
  void resetDsts() {
    Dsts.clear();
    NumDsts = 0;
  }

  unsigned addDst(ucOp Op, MachineOperand &MO) {
    // Ignore the datapath at the moment.
    if (!Op.isControl()) return 0;
    ++NumDsts;
    MachineOperand &DefMO = Op.getOperand(0);
    if (!DefMO.isReg()) {
      if (Op->isOpcode(VTM::VOpRetVal))
        addDst(0, 0);

      return 0;
    }

    unsigned DstReg = DefMO.getReg();
    addDst(DstReg, Op.getOpIdx(MO));
    return DstReg;
  }

  void addDst(unsigned Reg, unsigned OpIdx) {
    Dsts.insert(Reg | (OpIdx << 28));
  }

  int getSavedDstMuxSize() const {
    return NumDsts - int(Dsts.size());
  }

  int getSavedDstMuxCost() const {
    return getSavedDstMuxSize() * /* Mux pre-port area cost */VFUs::MUXCost;
  }
};

// Weight computation functor for register Compatibility Graph.
struct CompRegEdgeWeight : public WidthChecker, public SourceChecker<1>,
                           public DstChecker {
  VRASimple *VRA;
  // The pre-bit cost of this kind of function unit.
  unsigned Cost;
  // Context
  // Do not try to merge PHI copies, it is conditional and we cannot handle
  // them correctly at the moment.
  bool hasPHICopy;
  unsigned DstReg;

  CompRegEdgeWeight(VRASimple *V, unsigned cost) : VRA(V), Cost(cost) {}

  void resetDefEvalator(unsigned DstR) {
    DstReg = DstR;
    resetDsts();
    resetSrcs();
    hasPHICopy = false;
    resetWidth();
  }

  bool visitUse(ucOp Op, MachineOperand &MO) {
    unsigned DefReg = addDst(Op, MO);
    if (Op->isOpcode(VTM::VOpMvPhi) || Op->isOpcode(VTM::VOpMvPhi)
        || Op->isOpcode(VTM::VOpSel)) {
      // We cannot handle these ops correctly after their src and dst merged.
      if (DefReg == DstReg) return true;
    }

    return false;
  }

  bool visitDef(ucOp Op) {
    switch (Op->getOpcode()) {
    case VTM::VOpMvPhi:
    case VTM::VOpMvPipe:
    case VTM::VOpMove_rw:
    case VTM::VOpMove_rr:
    case VTM::VOpMove_ri:
    case VTM::COPY:
    case VTM::VOpReadFU:
      addSrc<0>(Op.getOperand(1));
      break;
    case VTM::VOpReadReturn:
      addSrc<0>(Op.getOperand(2));
      break;
    case VTM::VOpSel:
      addSrc<0>(Op.getOperand(2));
      addSrc<0>(Op.getOperand(3));
      break;
    default:
#ifndef NDEBUG
      Op.dump();
      llvm_unreachable("Unexpected opcode in CompRegEdgeWeight!");
#endif
    }
    return false;
  }

  // Run on the definition of a register to collect information about the live
  // interval.
  bool operator()(MachineRegisterInfo::reg_iterator I) {
    MachineOperand &MO = I.getOperand();
    if (MO.isDef()) {
      // 1. Get the bit width information.
      if (!checkWidth(cast<ucOperand>(I.getOperand()).getBitWidth()))
        return true;

      // 2. Analyze the definition op.
      return visitDef(ucOp::getParent(I));
    }

    if (!MO.isImplicit())
      return visitUse(ucOp::getParent(I), MO);

    return false;
  }

  // Run on the edge of the Compatibility Graph and return the weight of the
  // edge.
  int operator()(LiveInterval *Src, LiveInterval *Dst) {
    assert(Dst && Src && "Unexpected null li!");
    resetDefEvalator(Dst->reg);

    if (VRA->iterateUseDefChain(Src->reg, *this))
      return CompGraphWeights::HUGE_NEG_VAL;

    if (VRA->iterateUseDefChain(Dst->reg, *this))
      return CompGraphWeights::HUGE_NEG_VAL;

    if (hasPHICopy) return CompGraphWeights::HUGE_NEG_VAL;
    // Src register appear in the src of mux do not cost anything.
    removeSrcReg<0>(Src->reg);

    int Weight = 0;
    // We can save some register if we merge these two registers.
    Weight += /*Reg Cost*/ Cost;
    // How many mux port we can save?
    Weight += getSavedSrcMuxCost<0>();
    // We also can save the mux for the dsts.
    Weight += getSavedDstMuxCost();
    // How big the mux it is after the registers are merged? Do not make it too
    // big.
    Weight -= getSrcMuxCost<0>();
    // Other cost.
    Weight -= getExtraCost();
    return Weight * getWidth();
  }
};

// Weight computation functor for commutable binary operation Compatibility
// Graph.
template<unsigned OpCode, unsigned OpIdx>
struct CompBinOpEdgeWeight : public WidthChecker, SourceChecker<2>,
                             public DstChecker {
  VRASimple *VRA;
  // The pre-bit cost of this kind of function unit.
  unsigned Cost;

  // Is there a copy between the src and dst of the edge?
  //bool hasCopy;

  void resetDefEvalator() {
    //hasCopy = 0;
    resetWidth();
    resetSrcs();
    resetDsts();
  }

  template<unsigned Offset>
  void visitOperand(ucOp &Op) {
    addSrc<Offset>(Op.getOperand(OpIdx + Offset));
  }

  CompBinOpEdgeWeight(VRASimple *V, unsigned cost) : VRA(V), Cost(cost) {}

  // Run on the use-def chain of a FU to collect information about the live
  // interval.
  bool operator()(MachineRegisterInfo::reg_iterator I) {
    MachineOperand &MO = I.getOperand();
    if (I.getOperand().isDef()) {
      // 1. Get the bit width information.
      if (!checkWidth(cast<ucOperand>(MO).getBitWidth()))
        return true;
      // 2. Analyze the definition op.
      ucOp Op = ucOp::getParent(I);
      assert(Op->isOpcode(OpCode) && "Unexpected Opcode!");

      visitOperand<0>(Op);
      visitOperand<1>(Op);
    }

    if (!MO.isImplicit())
      addDst(ucOp::getParent(I), MO);

    return false;
  }

  int operator()(LiveInterval *Src, LiveInterval *Dst) {
    assert(Dst && Src && "Unexpected null li!");
    resetDefEvalator();

    if (VRA->iterateUseDefChain(Src->reg, *this))
      return CompGraphWeights::HUGE_NEG_VAL;

    if (VRA->iterateUseDefChain(Dst->reg, *this))
      return CompGraphWeights::HUGE_NEG_VAL;

    int Weight = 0;
    // We can save some register if we merge these two registers.
    Weight += /*FU Cost*/ Cost;
    // How many mux port we can save?
    Weight += getSavedSrcMuxCost<0>() + getSavedSrcMuxCost<1>();
    // We also can save the mux for the dsts.
    Weight += getSavedDstMuxCost();
    // How big the mux it is after the registers are merged? Do not make it too
    // big.
    Weight -= getSrcMuxCost<0>() + getSrcMuxCost<1>();
    // Other cost.
    Weight -= getExtraCost();
    return Weight * getWidth();
  }
};
}

namespace llvm {
  // Specialized For liveInterval.
template<> struct CompGraphTraits<LiveInterval*> {
  static bool isEarlier(LiveInterval *LHS, LiveInterval *RHS) {
    assert(LHS && RHS && "Unexpected virtual node!");

    return *LHS < *RHS;
  }

  static bool compatible(LiveInterval *LHS, LiveInterval *RHS) {
    if (LHS == 0 || RHS == 0) return true;

    // A LiveInterval may has multiple live ranges for example:
    // LI0 = [0, 4), [32, 35)
    // Suppose we have another LiveInterval LI1 and LI2:
    // LI1 = [5, 9)
    // LI2 = [26, 33)
    // If we make LiveIntervals compatible if they are not overlap, we may
    // have a compatible graph for LI0, LI1 and LI2 like this:
    // LI0 -> LI1 -> LI2
    // The compatibility in a compatible graph suppose to be transitive,
    // but this not true in the above graph because LI0 is overlap with LI2.
    // This means two live interval may not mark as "compatible" even if they
    // are not overlap.

    // LHS and RHS is compatible if RHS end before LHS begin and vice versa.
    return LHS->beginIndex() >= RHS->endIndex()
            || RHS->beginIndex() >= LHS->endIndex();
  }
};
}

//===----------------------------------------------------------------------===//
/// ComputeUltimateVN - Assuming we are going to join two live intervals,
/// compute what the resultant value numbers for each value in the input two
/// ranges will be.  This is complicated by copies between the two which can
/// and will commonly cause multiple value numbers to be merged into one.
///
/// VN is the value number that we're trying to resolve.  InstDefiningValue
/// keeps track of the new InstDefiningValue assignment for the result
/// LiveInterval.  ThisFromOther/OtherFromThis are sets that keep track of
/// whether a value in this or other is a copy from the opposite set.
/// ThisValNoAssignments/OtherValNoAssignments keep track of value #'s that have
/// already been assigned.
///
/// ThisFromOther[x] - If x is defined as a copy from the other interval, this
/// contains the value number the copy is from.
///
static unsigned ComputeUltimateVN(VNInfo *VNI,
                                  SmallVector<VNInfo*, 16> &NewVNInfo,
                                  DenseMap<VNInfo*, VNInfo*> &ThisFromOther,
                                  DenseMap<VNInfo*, VNInfo*> &OtherFromThis,
                                  SmallVector<int, 16> &ThisValNoAssignments,
                                  SmallVector<int, 16> &OtherValNoAssignments) {
    unsigned VN = VNI->id;

    // If the VN has already been computed, just return it.
    if (ThisValNoAssignments[VN] >= 0)
      return ThisValNoAssignments[VN];
    assert(ThisValNoAssignments[VN] != -2 && "Cyclic value numbers");

    // If this val is not a copy from the other val, then it must be a new value
    // number in the destination.
    DenseMap<VNInfo*, VNInfo*>::iterator I = ThisFromOther.find(VNI);
    if (I == ThisFromOther.end()) {
      NewVNInfo.push_back(VNI);
      return ThisValNoAssignments[VN] = NewVNInfo.size()-1;
    }
    VNInfo *OtherValNo = I->second;

    // Otherwise, this *is* a copy from the RHS.  If the other side has already
    // been computed, return it.
    if (OtherValNoAssignments[OtherValNo->id] >= 0)
      return ThisValNoAssignments[VN] = OtherValNoAssignments[OtherValNo->id];

    // Mark this value number as currently being computed, then ask what the
    // ultimate value # of the other value is.
    ThisValNoAssignments[VN] = -2;
    unsigned UltimateVN =
      ComputeUltimateVN(OtherValNo, NewVNInfo, OtherFromThis, ThisFromOther,
      OtherValNoAssignments, ThisValNoAssignments);
    return ThisValNoAssignments[VN] = UltimateVN;
}

/// JoinIntervals - Attempt to join these two intervals.  On failure, this
/// returns false.
static bool JoinIntervals(LiveInterval &LHS, LiveInterval &RHS,
                          const TargetRegisterInfo *TRI,
                          MachineRegisterInfo *MRI) {
  //DEBUG({ dbgs() << "\t\tRHS = "; RHS.print(dbgs(), TRI); dbgs() << "\n"; });
  assert(!TargetRegisterInfo::isPhysicalRegister(LHS.reg)
         && !TargetRegisterInfo::isPhysicalRegister(RHS.reg)
         && "Cannot join physics registers yet!");
  // If a live interval is a physical register, check for interference with any
  // aliases. The interference check implemented here is a bit more conservative
  // than the full interfeence check below. We allow overlapping live ranges
  // only when one is a copy of the other.
  //if (CP.isPhys()) {
  //  for (const unsigned *AS = tri_->getAliasSet(CP.getDstReg()); *AS; ++AS){
  //    if (!li_->hasInterval(*AS))
  //      continue;
  //    const LiveInterval &LHS = li_->getInterval(*AS);
  //    LiveInterval::const_iterator LI = LHS.begin();
  //    for (LiveInterval::const_iterator RI = RHS.begin(), RE = RHS.end();
  //      RI != RE; ++RI) {
  //        LI = std::lower_bound(LI, LHS.end(), RI->start);
  //        // Does LHS have an overlapping live range starting before RI?
  //        if ((LI != LHS.begin() && LI[-1].end > RI->start) &&
  //          (RI->start != RI->valno->def ||
  //          !CP.isCoalescable(li_->getInstructionFromIndex(RI->start)))) {
  //            DEBUG({
  //              dbgs() << "\t\tInterference from alias: ";
  //              LHS.print(dbgs(), tri_);
  //              dbgs() << "\n\t\tOverlap at " << RI->start << " and no copy.\n";
  //            });
  //            return false;
  //        }

  //        // Check that LHS ranges beginning in this range are copies.
  //        for (; LI != LHS.end() && LI->start < RI->end; ++LI) {
  //          if (LI->start != LI->valno->def ||
  //            !CP.isCoalescable(li_->getInstructionFromIndex(LI->start))) {
  //              DEBUG({
  //                dbgs() << "\t\tInterference from alias: ";
  //                LHS.print(dbgs(), tri_);
  //                dbgs() << "\n\t\tDef at " << LI->start << " is not a copy.\n";
  //              });
  //              return false;
  //          }
  //        }
  //    }
  //  }
  //}

  // Compute the final value assignment, assuming that the live ranges can be
  // coalesced.
  SmallVector<int, 16> LHSValNoAssignments;
  SmallVector<int, 16> RHSValNoAssignments;
  DenseMap<VNInfo*, VNInfo*> LHSValsDefinedFromRHS;
  DenseMap<VNInfo*, VNInfo*> RHSValsDefinedFromLHS;
  SmallVector<VNInfo*, 16> NewVNInfo;

  //DEBUG({ dbgs() << "\t\tLHS = "; LHS.print(dbgs(), TRI); dbgs() << "\n"; });

  // Loop over the value numbers of the LHS, seeing if any are defined from
  // the RHS.
  for (LiveInterval::vni_iterator i = LHS.vni_begin(), e = LHS.vni_end();
    i != e; ++i) {
      VNInfo *VNI = *i;
      if (VNI->isUnused() || !VNI->isDefByCopy())  // Src not defined by a copy?
        continue;

      // Never join with a register that has EarlyClobber redefs.
      if (VNI->hasRedefByEC())
        return false;

      // DstReg is known to be a register in the LHS interval.  If the src is
      // from the RHS interval, we can use its value #.
      //if (!CP.isCoalescable(VNI->getCopy()))
      //  continue;

      // Figure out the value # from the RHS.
      LiveRange *lr = RHS.getLiveRangeContaining(VNI->def.getPrevSlot());
      // The copy could be to an aliased physreg.
      if (!lr) continue;
      LHSValsDefinedFromRHS[VNI] = lr->valno;
  }

  // Loop over the value numbers of the RHS, seeing if any are defined from
  // the LHS.
  for (LiveInterval::vni_iterator i = RHS.vni_begin(), e = RHS.vni_end();
    i != e; ++i) {
      VNInfo *VNI = *i;
      if (VNI->isUnused() || !VNI->isDefByCopy())  // Src not defined by a copy?
        continue;

      // Never join with a register that has EarlyClobber redefs.
      if (VNI->hasRedefByEC())
        return false;

      // DstReg is known to be a register in the RHS interval.  If the src is
      // from the LHS interval, we can use its value #.
      //if (!CP.isCoalescable(VNI->getCopy()))
      //  continue;

      // Figure out the value # from the LHS.
      LiveRange *lr = LHS.getLiveRangeContaining(VNI->def.getPrevSlot());
      // The copy could be to an aliased physreg.
      if (!lr) continue;
      RHSValsDefinedFromLHS[VNI] = lr->valno;
  }

  LHSValNoAssignments.resize(LHS.getNumValNums(), -1);
  RHSValNoAssignments.resize(RHS.getNumValNums(), -1);
  NewVNInfo.reserve(LHS.getNumValNums() + RHS.getNumValNums());

  for (LiveInterval::vni_iterator i = LHS.vni_begin(), e = LHS.vni_end();
    i != e; ++i) {
      VNInfo *VNI = *i;
      unsigned VN = VNI->id;
      if (LHSValNoAssignments[VN] >= 0 || VNI->isUnused())
        continue;
      ComputeUltimateVN(VNI, NewVNInfo,
        LHSValsDefinedFromRHS, RHSValsDefinedFromLHS,
        LHSValNoAssignments, RHSValNoAssignments);
  }
  for (LiveInterval::vni_iterator i = RHS.vni_begin(), e = RHS.vni_end();
    i != e; ++i) {
      VNInfo *VNI = *i;
      unsigned VN = VNI->id;
      if (RHSValNoAssignments[VN] >= 0 || VNI->isUnused())
        continue;
      // If this value number isn't a copy from the LHS, it's a new number.
      if (RHSValsDefinedFromLHS.find(VNI) == RHSValsDefinedFromLHS.end()) {
        NewVNInfo.push_back(VNI);
        RHSValNoAssignments[VN] = NewVNInfo.size()-1;
        continue;
      }

      ComputeUltimateVN(VNI, NewVNInfo,
        RHSValsDefinedFromLHS, LHSValsDefinedFromRHS,
        RHSValNoAssignments, LHSValNoAssignments);
  }

  // Armed with the mappings of LHS/RHS values to ultimate values, walk the
  // interval lists to see if these intervals are coalescable.
  LiveInterval::const_iterator I = LHS.begin();
  LiveInterval::const_iterator IE = LHS.end();
  LiveInterval::const_iterator J = RHS.begin();
  LiveInterval::const_iterator JE = RHS.end();

  // Skip ahead until the first place of potential sharing.
  if (I != IE && J != JE) {
    if (I->start < J->start) {
      I = std::upper_bound(I, IE, J->start);
      if (I != LHS.begin()) --I;
    } else if (J->start < I->start) {
      J = std::upper_bound(J, JE, I->start);
      if (J != RHS.begin()) --J;
    }
  }

  while (I != IE && J != JE) {
    // Determine if these two live ranges overlap.
    bool Overlaps;
    if (I->start < J->start) {
      Overlaps = I->end > J->start;
    } else {
      Overlaps = J->end > I->start;
    }

    // If so, check value # info to determine if they are really different.
    if (Overlaps) {
      // If the live range overlap will map to the same value number in the
      // result liverange, we can still coalesce them.  If not, we can't.
      if (LHSValNoAssignments[I->valno->id] !=
        RHSValNoAssignments[J->valno->id])
        return false;
      // If it's re-defined by an early clobber somewhere in the live range,
      // then conservatively abort coalescing.
      if (NewVNInfo[LHSValNoAssignments[I->valno->id]]->hasRedefByEC())
        return false;
    }

    if (I->end < J->end)
      ++I;
    else
      ++J;
  }

  // Update kill info. Some live ranges are extended due to copy coalescing.
  for (DenseMap<VNInfo*, VNInfo*>::iterator I = LHSValsDefinedFromRHS.begin(),
    E = LHSValsDefinedFromRHS.end(); I != E; ++I) {
      VNInfo *VNI = I->first;
      unsigned LHSValID = LHSValNoAssignments[VNI->id];
      if (VNI->hasPHIKill())
        NewVNInfo[LHSValID]->setHasPHIKill(true);
  }

  // Update kill info. Some live ranges are extended due to copy coalescing.
  for (DenseMap<VNInfo*, VNInfo*>::iterator I = RHSValsDefinedFromLHS.begin(),
    E = RHSValsDefinedFromLHS.end(); I != E; ++I) {
      VNInfo *VNI = I->first;
      unsigned RHSValID = RHSValNoAssignments[VNI->id];
      if (VNI->hasPHIKill())
        NewVNInfo[RHSValID]->setHasPHIKill(true);
  }

  if (LHSValNoAssignments.empty())
    LHSValNoAssignments.push_back(-1);
  if (RHSValNoAssignments.empty())
    RHSValNoAssignments.push_back(-1);

  // If we get here, we know that we can coalesce the live ranges.  Ask the
  // intervals to coalesce themselves now.
  LHS.join(RHS, &LHSValNoAssignments[0], &RHSValNoAssignments[0], NewVNInfo,
           MRI);
  return true;
}

//===----------------------------------------------------------------------===//

FunctionPass *llvm::createSimpleRegisterAllocator() {
  return new VRASimple();
}

char VRASimple::ID = 0;

VRASimple::VRASimple() : MachineFunctionPass(ID) {
  initializeLiveIntervalsPass(*PassRegistry::getPassRegistry());
  initializeSlotIndexesPass(*PassRegistry::getPassRegistry());
  initializeStrongPHIEliminationPass(*PassRegistry::getPassRegistry());
  initializeRegisterCoalescerAnalysisGroup(*PassRegistry::getPassRegistry());
  initializeCalculateSpillWeightsPass(*PassRegistry::getPassRegistry());
  initializeLiveStacksPass(*PassRegistry::getPassRegistry());
  initializeMachineDominatorTreePass(*PassRegistry::getPassRegistry());
  initializeMachineLoopInfoPass(*PassRegistry::getPassRegistry());
  initializeVirtRegMapPass(*PassRegistry::getPassRegistry());
  initializeBitLevelInfoPass(*PassRegistry::getPassRegistry());
}

void VRASimple::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.setPreservesCFG();
  AU.addRequired<AliasAnalysis>();
  AU.addPreserved<AliasAnalysis>();
  AU.addRequired<LiveIntervals>();
  AU.addPreserved<SlotIndexes>();
  AU.addRequiredTransitive<RegisterCoalescer>();
  AU.addRequired<CalculateSpillWeights>();
  AU.addRequired<LiveStacks>();
  AU.addPreserved<LiveStacks>();
  AU.addRequiredID(MachineDominatorsID);
  AU.addPreservedID(MachineDominatorsID);
  AU.addRequired<MachineLoopInfo>();
  AU.addPreserved<MachineLoopInfo>();
  AU.addRequired<VirtRegMap>();
  AU.addPreserved<VirtRegMap>();
  MachineFunctionPass::getAnalysisUsage(AU);
}

void VRASimple::releaseMemory() {
  RegAllocBase::releaseMemory();
}

void VRASimple::init(VirtRegMap &vrm, LiveIntervals &lis) {
  TRI = &vrm.getTargetRegInfo();
  MRI = &vrm.getRegInfo();
  VRM = &vrm;
  LIS = &lis;
  // FIXME: Init the PhysReg2LiveUnion right before we start to bind the physics
  // registers.
  PhysReg2LiveUnion.init(UnionAllocator, MRI->getNumVirtRegs() + 1);
  // Cache an interferece query for each physical reg
  // Queries.reset(new LiveIntervalUnion::Query[PhysReg2LiveUnion.numRegs()]);
}

bool VRASimple::runOnMachineFunction(MachineFunction &F) {
  MF = &F;
  VFI = F.getInfo<VFInfo>();

  UserTag = 0;

  init(getAnalysis<VirtRegMap>(), getAnalysis<LiveIntervals>());

  DEBUG(dbgs() << "Before simple register allocation:\n";
        //printVMF(dbgs(), F);
  ); 

  joinPHINodeIntervals();

  // Bind the pre-bind function units.
  bindMemoryBus();
  bindBlockRam();
  bindCalleeFN();

  //Build the Compatibility Graphs
  LICGraph RCG(VTM::DRRegClassID),
           AdderCG(VTM::RADDRegClassID),
           MulCG(VTM::RMULRegClassID),
           AsrCG(VTM::RASRRegClassID),
           LsrCG(VTM::RLSRRegClassID),
           ShlCG(VTM::RSHLRegClassID);
  buildCompGraph(RCG);
  buildCompGraph(AdderCG);
  buildCompGraph(MulCG);
  buildCompGraph(AsrCG);
  buildCompGraph(LsrCG);
  buildCompGraph(ShlCG);

  CompRegEdgeWeight RegWeight(this, VFUs::RegCost);
  CompBinOpEdgeWeight<VTM::VOpAdd, 2> AddWeight(this, VFUs::AddCost);
  CompBinOpEdgeWeight<VTM::VOpMult, 1> MulWeiht(this, VFUs::MulCost);
  CompBinOpEdgeWeight<VTM::VOpSRA, 1> SRAWeight(this, VFUs::ShiftCost);
  CompBinOpEdgeWeight<VTM::VOpSRL, 1> SRLWeight(this, VFUs::ShiftCost);
  CompBinOpEdgeWeight<VTM::VOpSHL, 1> SHLWeight(this, VFUs::ShiftCost);

  bool SomethingBind = true;
  // Reduce the Compatibility Graphs
  while (SomethingBind) {
    DEBUG(dbgs() << "Going to reduce CompGraphs\n");
    SomethingBind = false;
    SomethingBind |= reduceCompGraph(RCG, RegWeight);
    SomethingBind |= reduceCompGraph(AdderCG, AddWeight);
    SomethingBind |= reduceCompGraph(MulCG, MulWeiht);
    SomethingBind |= reduceCompGraph(AsrCG, SRAWeight);
    SomethingBind |= reduceCompGraph(LsrCG, SRLWeight);
    SomethingBind |= reduceCompGraph(ShlCG, SHLWeight);
  }

  // Bind the Compatibility Graphs
  bindCompGraph(RCG);
  bindAdders(AdderCG);
  bindCompGraph(MulCG);
  bindCompGraph(AsrCG);
  bindCompGraph(LsrCG);
  bindCompGraph(ShlCG);

  addMBBLiveIns(MF);
  LIS->addKillFlags();

  // FIXME: Verification currently must run before VirtRegRewriter. We should
  // make the rewriter a separate pass and override verifyAnalysis instead. When
  // that happens, verification naturally falls under VerifyMachineCode.
#ifndef NDEBUG
  if (VerifyEnabled) {
    // Verify accuracy of LiveIntervals. The standard machine code verifier
    // ensures that each LiveIntervals covers all uses of the virtual reg.

    // FIXME: MachineVerifier is badly broken when using the standard
    // spiller. Always use -spiller=inline with -verify-regalloc. Even with the
    // inline spiller, some tests fail to verify because the coalescer does not
    // always generate verifiable code.
    MF->verify(this, "In RABasic::verify");

    // Verify that LiveIntervals are partitioned into unions and disjoint within
    // the unions.
    verify();
  }
#endif // !NDEBUG

  // Run rewriter
  VRM->rewrite(LIS->getSlotIndexes());

  releaseMemory();

  DEBUG(dbgs() << "After simple register allocation:\n";
        //printVMF(dbgs(), F);
  ); 

  return true;
}

void VRASimple::joinPHINodeIntervals() {
  VRegVec &VRegs = MRI->getRegClassVirtRegs(VTM::PHIRRegisterClass);


  for (VRegVec::const_iterator I = VRegs.begin(), E = VRegs.end(); I != E; ++I){
    unsigned PHINum = *I;

    MachineRegisterInfo::use_iterator UI = MRI->use_begin(PHINum);
    ucOp Op = ucOp::getParent(UI);
    assert(Op->getOpcode() == VTM::VOpDefPhi && "Unexpected Opcode!");
    unsigned DstReg = Op.getOperand(0).getReg();
    LiveInterval &DstLI = LIS->getInterval(DstReg);

    JoinIntervals(DstLI, LIS->getInterval(PHINum), TRI, MRI);

    if (MRI->getRegClass(DstReg) != VTM::DRRegisterClass) {
      // Join the phi source to dst, too.
      MachineRegisterInfo::def_iterator DI = MRI->def_begin(PHINum);
      ucOp OpMove = ucOp::getParent(DI);
      assert(++DI == MRI->def_end() && "PHI source not in SSA form!");
      MachineOperand &PHISrcMO = OpMove.getOperand(1);
      if (!PHISrcMO.isReg()) continue;

      unsigned PHISrc = PHISrcMO.getReg();

      JoinIntervals(DstLI, LIS->getInterval(PHISrc), TRI, MRI);
      MRI->replaceRegWith(PHISrc, DstReg);
      // DirtyHack: Remove the define flag of the PHI stuffs so we have only
      // 1 define for the register.
      OpMove.getOperand(0).setIsDef(false);
    }

    MRI->replaceRegWith(PHINum, DstReg);
    // DirtyHack: Remove the define flag of the PHI stuffs so we have only
    // 1 define for the register.
    Op.getOperand(0).setIsDef(false);
  }
}

void VRASimple::mergeLI(LiveInterval *FromLI, LiveInterval *ToLI) {
  assert(!ToLI->overlaps(*FromLI) && "Cannot mrege LI!");
  assert(getBitWidthOf(FromLI->reg) == getBitWidthOf(ToLI->reg)
         && "Cannot merge LIs difference with bit width!");
  JoinIntervals(*ToLI, *FromLI, TRI, MRI);
  MRI->replaceRegWith(FromLI->reg, ToLI->reg);
}

void VRASimple::bindMemoryBus() {
  VRegVec &VRegs = MRI->getRegClassVirtRegs(VTM::RINFRegisterClass);

  LiveInterval *MemBusLI = 0;

  for (VRegVec::const_iterator I = VRegs.begin(), E = VRegs.end(); I != E; ++I){
    unsigned RegNum = *I;

    if (LiveInterval *LI = getInterval(RegNum)) {
      if (MemBusLI == 0) {
        assign(*LI, VFI->allocateFN(VTM::RINFRegClassID));
        MemBusLI = LI;
        continue;
      }

      // Merge all others LI to MemBusLI.
      mergeLI(LI, MemBusLI);
    }
  }
}

void VRASimple::bindBlockRam() {
  VRegVec &VRegs = MRI->getRegClassVirtRegs(VTM::RBRMRegisterClass);
  std::map<unsigned, LiveInterval*> RepLIs;

  for (VRegVec::const_iterator I = VRegs.begin(), E = VRegs.end(); I != E; ++I){
    unsigned RegNum = *I;

    if (LiveInterval *LI = getInterval(RegNum)) {
      ucOp Op = ucOp::getParent(MRI->def_begin(RegNum));
      assert(Op->getOpcode() == VTM::VOpBRam && "Unexpected opcode!");
      unsigned BRamNum = Op->getFUId().getFUNum();

      VFInfo::BRamInfo &Info = VFI->getBRamInfo(BRamNum);
      unsigned &PhyReg = Info.PhyRegNum;

      // Had we allocate a register for this bram?
      if (PhyReg == 0) {
        unsigned BitWidth = Info.ElemSizeInBytes * 8;
        PhyReg = VFI->allocateFN(VTM::RBRMRegClassID, BitWidth);
        RepLIs[PhyReg] = LI;
        assign(*LI, PhyReg);
        continue;
      }

      // Merge to the representative live interval.
      mergeLI(LI, RepLIs[PhyReg]);
    }
  }
}

void VRASimple::bindCalleeFN() {
  VRegVec &VRegs = MRI->getRegClassVirtRegs(VTM::RCFNRegisterClass);
  std::map<unsigned, LiveInterval*> RepLIs;

  for (VRegVec::const_iterator I = VRegs.begin(), E = VRegs.end(); I != E; ++I){
    unsigned RegNum = *I;

    if (LiveInterval *LI = getInterval(RegNum)) {
      ucOp Op = ucOp::getParent(MRI->def_begin(RegNum));
      unsigned FNNum = Op.getOperand(1).getBitWidth();
      LiveInterval *&RepLI = RepLIs[FNNum];

      if (RepLI == 0) {
        // This is the first live interval bound to this callee function.
        // Use this live interval to represent the live interval of this callee
        // function.
        RepLI = LI;
        assign(*LI, VFI->allocateFN(VTM::RCFNRegClassID));
        continue;
      }

      // Merge to the representative live interval.
      mergeLI(LI, RepLI);
    }
  }
}

void VRASimple::buildCompGraph(LICGraph &G) {
  VRegVec &VRegs = MRI->getRegClassVirtRegs(TRI->getRegClass(G.ID));

  for (VRegVec::const_iterator I = VRegs.begin(), E = VRegs.end(); I != E; ++I)
    if (LiveInterval *LI = getInterval(*I))
      G.GetOrCreateNode(LI);
}

template<class CompEdgeWeight>
bool VRASimple::reduceCompGraph(LICGraph &G, CompEdgeWeight &C) {
  SmallVector<LiveInterval*, 8> LongestPath, MergedLIs;
  G.updateEdgeWeight(C);

  bool AnyReduced = false;

  for (int PathWeight = G.findLongestPath(LongestPath, true); PathWeight > 0;
       PathWeight = G.findLongestPath(LongestPath, true)) {
    DEBUG(dbgs() << "// longest path in graph: {"
      << TRI->getRegClass(G.ID)->getName() << "} weight:" << PathWeight << "\n";
    for (unsigned i = 0; i < LongestPath.size(); ++i) {
      LiveInterval *LI = LongestPath[i];
      dbgs() << *LI << " bitwidth:" << getBitWidthOf(LI->reg) << '\n';
    });

    LiveInterval *RepLI = LongestPath.pop_back_val();

    // Merge the other LIs in the path to the first LI.
    while (!LongestPath.empty())
      mergeLI(LongestPath.pop_back_val(), RepLI);
    // Add the merged LI back to the graph later.
    MergedLIs.push_back(RepLI);
    AnyReduced = true;
  }

  // Re-add the merged LI to the graph.
  while (!MergedLIs.empty())
    G.GetOrCreateNode(MergedLIs.pop_back_val());

  return AnyReduced;
}

void VRASimple::bindCompGraph(LICGraph &G) {
  unsigned RC = G.ID;
  for (LICGraph::iterator I = G.begin(), E = G.end(); I != E; ++I) {
    LiveInterval *LI = (*I)->get();
    assign(*LI, VFI->allocatePhyReg(RC, getBitWidthOf(LI->reg)));
  }
}

void VRASimple::bindAdders(LICGraph &G) {
  for (LICGraph::iterator I = G.begin(), E = G.end(); I != E; ++I) {
    LiveInterval *LI = (*I)->get();
    unsigned Width = getBitWidthOf(LI->reg);
    // Allocate the register for the adder, which also contains the carry bit
    unsigned AdderReg = VFI->allocatePhyReg(VTM::RADDRegClassID, Width + 1);
    unsigned SumReg = VFI->getSubRegOf(AdderReg, Width, 0);
    assign(*LI, SumReg);
    DEBUG(dbgs() << "Assign " << SumReg << " to " << *LI << '\n');
    // Bind the carry.
    unsigned CarryReg = VFI->getSubRegOf(AdderReg, Width + 1, Width);
    for (MachineRegisterInfo::def_iterator I = MRI->def_begin(LI->reg),
         E = MRI->def_end(); I != E; ++I) {
      ucOp Op = ucOp::getParent(I);
      assert(Op->getOpcode() == VTM::VOpAdd && "Unexpected opcode for adder!");
      unsigned CarryNum = Op.getOperand(1).getReg();
      LiveInterval *CarryLI = getInterval(CarryNum);
      assert(CarryLI && "Cannot found interval of carry!");
      assign(*CarryLI, CarryReg);
    }
  }
}
