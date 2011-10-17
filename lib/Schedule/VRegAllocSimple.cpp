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

#include "vtm/VFInfo.h"
#include "vtm/VRegisterInfo.h"
#include "vtm/BitLevelInfo.h"
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

class VRASimple : public MachineFunctionPass,
                  public RegAllocBase {
  // DIRTY HACK: We need to init the PhysReg2LiveUnion again with correct
  // physics register number.
  LiveIntervalUnion::Allocator UnionAllocator;
  // Context.
  MachineFunction *MF;
  VFInfo *VFI;

  // Analysis
  LiveStacks *LS;
  const BitLevelInfo *BLI;

  std::priority_queue<LiveInterval*, std::vector<LiveInterval*>,
                      CompSpillWeight> Queue;
  unsigned UserTag;

  //LiveIntervalUnion::Query &query(LiveInterval &VirtReg, unsigned PhysReg) {
  //  Queries[PhysReg].init(UserTag, &VirtReg, &PhysReg2LiveUnion[PhysReg]);
  //  return Queries[PhysReg];
  //}

public:
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
  unsigned selectOrSplit(LiveInterval &VirtReg,
                         SmallVectorImpl<LiveInterval*> &splitLVRs);

  bool runOnMachineFunction(MachineFunction &F);

  const char *getPassName() const {
    return "Verilog Backend Resource Binding Pass";
  }
};

char VRASimple::ID = 0;

}

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
  AU.addRequired<BitLevelInfo>();
  AU.addPreserved<BitLevelInfo>();
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

  PhysReg2LiveUnion.init(UnionAllocator, MRI->getNumVirtRegs());
  // Cache an interferece query for each physical reg
  Queries.reset(new LiveIntervalUnion::Query[PhysReg2LiveUnion.numRegs()]);

}

bool VRASimple::runOnMachineFunction(MachineFunction &F) {
  MF = &F;
  VFI = F.getInfo<VFInfo>();
  BLI = &getAnalysis<BitLevelInfo>();

  UserTag = 0;

  init(getAnalysis<VirtRegMap>(), getAnalysis<LiveIntervals>());

  DEBUG(dbgs() << "Before simple register allocation:\n";
        printVMF(dbgs(), F);
  ); 

  allocatePhysRegs();

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
  unsigned Size = BLI->getBitWidth(VReg);

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
  return VFI->allocatePhyReg(MRI->getRegClass(VReg)->getID(), Size);
}
