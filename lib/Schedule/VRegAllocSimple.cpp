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

#include "llvm/ADT/Statistic.h"
#include "llvm/Support/MathExtras.h"
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
#define DEBUG_TYPE "vtm-regalloc"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/CommandLine.h"

#include <queue>

using namespace llvm;

cl::opt<bool> EnableSimpleRegisterSharing("vtm-enable-simple-register-sharing",
                                          cl::init(false), cl::Hidden);

static RegisterRegAlloc VSimpleRegalloc("vsimple",
                                        "vtm-simple register allocator",
                                        createSimpleRegisterAllocator);

namespace {
struct CompSpillWeight {
  bool operator()(LiveInterval *A, LiveInterval *B) const {
    return A->weight < B->weight;
  }
};

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

  typedef CompGraph<unsigned> RegCompGraph;
  typedef CompGraphNode<unsigned> RegCompGraphNode;

  typedef const std::vector<unsigned> VRegVec;

  std::priority_queue<LiveInterval*, std::vector<LiveInterval*>,
                      CompSpillWeight> Queue;
  unsigned UserTag;

  //LiveIntervalUnion::Query &query(LiveInterval &VirtReg, unsigned PhysReg) {
  //  Queries[PhysReg].init(UserTag, &VirtReg, &PhysReg2LiveUnion[PhysReg]);
  //  return Queries[PhysReg];
  //}

  VRASimple();
  void init(VirtRegMap &vrm, LiveIntervals &lis);
  
  static char ID;

  void getAnalysisUsage(AnalysisUsage &AU) const;
  void releaseMemory();


  Spiller &spiller() {
    llvm_unreachable("VRegAllocSimple - Never spill!");
    return *(Spiller*)0;
  }

  virtual float getPriority(LiveInterval *LI) { return LI->weight; }

  virtual void enqueue(LiveInterval *LI) {
    unsigned Reg = LI->reg;

    if (LI->isZeroLength())
      return;

    // Preserves SSA From for wires.
    // if (MRI->getRegClass(Reg) == VTM::WireRegisterClass)
    if (VRegisterInfo::IsWire(Reg, MRI))
      return;

    Queue.push(LI);
  }

  virtual LiveInterval *dequeue() {
    if (Queue.empty())
      return 0;
    LiveInterval *LI = Queue.top();
    Queue.pop();
    return LI;
  }

  unsigned checkPhysRegInterference(LiveInterval &VirtReg, unsigned PhysReg);

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

  ucOp getDefineOp(unsigned Reg) const {
    assert(!MRI->def_empty(Reg) && "Cannot get define op!");
    return ucOp::getParent(MRI->def_begin(Reg));
  }

  unsigned getRepRegister(unsigned Reg) const {
    if (TargetRegisterInfo::isVirtualRegister(Reg)) {
      if (unsigned PhyReg = VRM->getPhys(Reg))
        return PhyReg;
    }

    return Reg;
  }

  unsigned getDrivingFU(ucOp Op) const {
    if (Op->isOpcode(VTM::VOpReadFU))
      return getRepRegister(Op.getOperand(1).getReg());

    return 0;
  }

  void joinPHINodeIntervals();

  void bindMemoryBus();
  void bindBlockRam();
  void bindCalleeFN();

  bool bindDataRegister();
  bool bindFUs();
  bool bindFUsOf(const TargetRegisterClass *RC);
  bool bindAdders();

  unsigned getBitWidthOf(unsigned RegNum) {
    ucOperand &DefOp = cast<ucOperand>(MRI->def_begin(RegNum).getOperand());
    return DefOp.getBitWidth();
  }


  unsigned selectOrSplit(LiveInterval &VirtReg,
                         SmallVectorImpl<LiveInterval*> &splitLVRs);

  bool runOnMachineFunction(MachineFunction &F);

  const char *getPassName() const {
    return "Verilog Backend Resource Binding Pass";
  }
};

char VRASimple::ID = 0;

}

namespace llvm {
  // Specialized For liveInterval.
  template<> struct CompGraphQuery<unsigned> {
    VRASimple *VRA;

    CompGraphQuery(VRASimple *V) : VRA(V) {}

    unsigned getVirtualEdgeWeight () const { return 0; }
    unsigned getSameFUWeight() const { return 128; }

    bool isEarlier(unsigned LHS, unsigned RHS) const {
      if (LHS == 0) return true;

      if (RHS == 0) return false;

      LiveInterval *LHSLI = VRA->getInterval(LHS),
                   *RHSLI = VRA->getInterval(RHS);
      assert(LHSLI && RHSLI && "Unexpected null live iterval!");
      return *LHSLI < *RHSLI;
    }

    bool compatible(unsigned LHS, unsigned RHS) const {
      if (LHS == 0 || RHS == 0) return true;

      LiveInterval *LHSLI = VRA->getInterval(LHS),
                   *RHSLI = VRA->getInterval(RHS);
      assert(LHSLI && RHSLI && "Unexpected null live iterval!");
      return !LHSLI->overlapsFrom(*RHSLI, RHSLI->begin());
    }

    unsigned calcWeight(unsigned Src, unsigned Dst) const {
      assert(Dst && Src && "Unexpected noreg!");

      ucOp SrcOp = VRA->getDefineOp(Src), DstOp = VRA->getDefineOp(Dst);
      unsigned SrcFU = VRA->getDrivingFU(SrcOp),
               DstFU = VRA->getDrivingFU(DstOp);
      if (SrcFU && SrcFU == DstFU) {
        return getSameFUWeight();
      }

      return 0;
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
  DEBUG({ dbgs() << "\t\tRHS = "; RHS.print(dbgs(), TRI); dbgs() << "\n"; });
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

  DEBUG({ dbgs() << "\t\tLHS = "; LHS.print(dbgs(), TRI); dbgs() << "\n"; });

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

  PhysReg2LiveUnion.init(UnionAllocator, MRI->getNumVirtRegs() + 1);
  // Cache an interferece query for each physical reg
  Queries.reset(new LiveIntervalUnion::Query[PhysReg2LiveUnion.numRegs()]);
}

bool VRASimple::runOnMachineFunction(MachineFunction &F) {
  MF = &F;
  VFI = F.getInfo<VFInfo>();

  UserTag = 0;

  init(getAnalysis<VirtRegMap>(), getAnalysis<LiveIntervals>());

  DEBUG(dbgs() << "Before simple register allocation:\n";
        printVMF(dbgs(), F);
  ); 

  joinPHINodeIntervals();

  // Bind the pre-bind function units.
  bindMemoryBus();
  bindBlockRam();
  bindCalleeFN();

  bool SomethingBind = true;

  //while (SomethingBind) {
    SomethingBind = false;
    SomethingBind |= bindDataRegister();
    SomethingBind |= bindFUs();
  //}

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
        printVMF(dbgs(), F);
  ); 

  return true;
}

unsigned VRASimple::checkPhysRegInterference(LiveInterval &VirtReg,
                                             unsigned PhysReg) {
  //unsigned Overlaps[16];

  //for (unsigned i = 0, e = VFI->getOverlaps(PhysReg, Overlaps); i < e; ++i)
  //  if (query(VirtReg, Overlaps[i]).checkInterference())
  //    return Overlaps[i];

  //ucOp Op = ucOp::getParent(MRI->def_begin(VirtReg.reg));
  //if (Op->getOpcode() == VTM::VOpDefPhi) {
  //  unsigned PHINum = Op.getOperand(1).getReg();
  //  return checkPhysRegInterference(LIS->getInterval(PHINum), PhysReg);
  //}

  return 0;
}
unsigned VRASimple::selectOrSplit(LiveInterval &VirtReg,
                                  SmallVectorImpl<LiveInterval*> &splitLVRs) {
  unsigned VReg = VirtReg.reg;
  ucOperand &DefOp = cast<ucOperand>(MRI->def_begin(VReg).getOperand());
  unsigned Bitwidth = DefOp.getBitWidth();
  ////if (EnableSimpleRegisterSharing)
  //  for (reg_it I = VFI->phyreg_begin(Size), E = VFI->phyreg_end(Size);
  //       I < E; ++I) {
  //    unsigned PhysReg = *I;
  //    if (checkPhysRegInterference(VirtReg, PhysReg) == 0)
  //      return PhysReg;
  //  }

  //unsigned Reg =  VFI->allocatePhyReg(Size);

  //while (checkPhysRegInterference(VirtReg, Reg) != 0)
  //  Reg =  VFI->allocatePhyReg(Size);

  //return Reg;
  return VFI->allocatePhyReg(MRI->getRegClass(VReg)->getID(), Bitwidth);
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
      assert(!MemBusLI->overlapsFrom(*LI, LI->begin()) && "Cannot bind membus!");
      JoinIntervals(*MemBusLI, *LI, TRI, MRI);
      MRI->replaceRegWith(LI->reg, MemBusLI->reg);
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
      LiveInterval *RepLI = RepLIs[PhyReg];
      assert(!RepLI->overlapsFrom(*LI, LI->begin()) && "Cannot bind membus!");
      JoinIntervals(*RepLI, *LI, TRI, MRI);
      MRI->replaceRegWith(LI->reg, RepLI->reg);
    }
  }
}

void VRASimple::bindCalleeFN() {
  VRegVec &VRegs = MRI->getRegClassVirtRegs(VTM::RCFNRegisterClass);

  for (VRegVec::const_iterator I = VRegs.begin(), E = VRegs.end(); I != E; ++I){
    unsigned RegNum = *I;

    if (LiveInterval *LI = getInterval(RegNum)) {
      ucOp Op = ucOp::getParent(MRI->def_begin(RegNum));
      unsigned FR = VFI->allocateFN(VTM::RCFNRegClassID);

      assert(!query(*LI, FR).checkInterference()&&"Cannot bind sub module!");
      assign(*LI, FR);
    }
  }
}

bool VRASimple::bindDataRegister() {
  VRegVec &VRegs = MRI->getRegClassVirtRegs(VTM::DRRegisterClass);
  //CompGraphQuery<unsigned> Q(this);
  //RegCompGraph G(Q);

  for (VRegVec::const_iterator I = VRegs.begin(), E = VRegs.end(); I != E; ++I){
    unsigned RegNum = *I;

    if (LiveInterval *LI = getInterval(RegNum)) {
      unsigned PhyReg = VFI->allocatePhyReg(VTM::DRRegClassID,
                                            getBitWidthOf(RegNum));
      //G.GetOrCreateNode(LI->reg);
      assign(*LI, PhyReg);
    }
  }

  //G.buildGraph();

  //G.viewGraph();

  //SmallVector<RegCompGraphNode*, 8> LongestPath;
  //G.findLongestPath(LongestPath);

  //for (unsigned i = 0; i < LongestPath.size(); ++i)
  //  dbgs() << PrintReg(LongestPath[i]->get()) << '\n';

  return false;
}

bool VRASimple::bindFUs() {
  bindAdders();
  bindFUsOf(VTM::RMULRegisterClass);
  bindFUsOf(VTM::RASRRegisterClass);
  bindFUsOf(VTM::RSHLRegisterClass);
  bindFUsOf(VTM::RLSRRegisterClass);
  return false;
}

bool VRASimple::bindFUsOf(const TargetRegisterClass *RC) {
  VRegVec &VRegs = MRI->getRegClassVirtRegs(RC);
  unsigned RegID = RC->getID();

  for (VRegVec::const_iterator I = VRegs.begin(), E = VRegs.end(); I != E; ++I){
    unsigned RegNum = *I;

    if (LiveInterval *LI = getInterval(RegNum)) {
      unsigned PhyReg = VFI->allocatePhyReg(RegID, getBitWidthOf(RegNum));

      assign(*LI, PhyReg);
    }
  }

  return false;
}

bool VRASimple::bindAdders() {
  VRegVec &VRegs = MRI->getRegClassVirtRegs(VTM::RADDRegisterClass);

  for (VRegVec::const_iterator I = VRegs.begin(), E = VRegs.end(); I != E; ++I){
    unsigned RegNum = *I;

    if (LiveInterval *LI = getInterval(RegNum)) {
      unsigned Width = getBitWidthOf(RegNum);
      // Allocate the register for the adder, which also contains the carry bit
      unsigned AdderReg = VFI->allocatePhyReg(VTM::RADDRegClassID, Width + 1);
      unsigned SumReg = VFI->getSubRegOf(AdderReg, Width, 0);
      assign(*LI, SumReg);

      ucOp Op = ucOp::getParent(MRI->def_begin(RegNum));
      assert(Op->getOpcode() == VTM::VOpAdd && "Unexpected opcode for adder!");
      unsigned CarryNum = Op.getOperand(1).getReg();
      LiveInterval *CarryLI = getInterval(CarryNum);
      assert(CarryLI && "Cannot found interval of carry!");
      unsigned CarryReg = VFI->getSubRegOf(AdderReg, Width + 1, Width);
      assign(*CarryLI, CarryReg);
    }
  }

  return false;
}
