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

  typedef const std::vector<unsigned> VRegVec;

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

  void bindMemoryBus();
  void bindCalleeFN();

  bool bindDataRegister();
  bool bindFUs();
  bool bindFUsOf(const TargetRegisterClass *RC);

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

  // Bind the pre-bind function units.
  bindMemoryBus();
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

void VRASimple::bindMemoryBus() {
  VRegVec &VRegs = MRI->getRegClassVirtRegs(VTM::RINFRegisterClass);

  //unsigned DataWidth = getFUDesc<VFUMemBus>()->getDataWidth();
  unsigned MemBusReg = VFI->allocateFN(VTM::RINFRegClassID);

  for (VRegVec::const_iterator I = VRegs.begin(), E = VRegs.end(); I != E; ++I){
    unsigned RegNum = *I;

    if (LiveInterval *LI = getInterval(RegNum)) {
      // The register may defined by VOpMove_ww which produced by PHINodes, but
      // it is also correct to check interference and bind it to memory bus
      // register, because this just connecting the two wires(and their live
      // intervals) together.
      assert(!query(*LI, MemBusReg).checkInterference() && "Cannot bind membus!");
      assign(*LI, MemBusReg);
    }
  }
}

void VRASimple::bindCalleeFN() {
  VRegVec &VRegs = MRI->getRegClassVirtRegs(VTM::RCFNRegisterClass);
  std::map<unsigned, unsigned> FNMap;

  for (VRegVec::const_iterator I = VRegs.begin(), E = VRegs.end(); I != E; ++I){
    unsigned RegNum = *I;

    if (LiveInterval *LI = getInterval(RegNum)) {
      ucOp Op = ucOp::getParent(MRI->def_begin(RegNum));
      unsigned FNNum;

      //if (CallInst->getOpcode() != VTM::VOpInternalCall) {
      //  assert(CallInst->getOpcode() == VTM::VOpDefPhi
      //         && "Unexpected opcode defining callee fu!");

      //  // Join the registers
      //  FNNum = CallInst.getOperand(1).getTargetFlags();
      //} else
      FNNum = Op.getOperand(1).getBitWidth();
      //}

      unsigned &FR = FNMap[FNNum];
      // Allocate the register for this submodule if it is not yet allocated.
      if (FR == 0) FR = VFI->allocateFN(VTM::RCFNRegClassID);

      assert(!query(*LI, FR).checkInterference()&&"Cannot bind sub module!");
      assign(*LI, FR);
    }
  }
}

bool VRASimple::bindDataRegister() {
  VRegVec &VRegs = MRI->getRegClassVirtRegs(VTM::DRRegisterClass);

  for (VRegVec::const_iterator I = VRegs.begin(), E = VRegs.end(); I != E; ++I){
    unsigned RegNum = *I;

    if (LiveInterval *LI = getInterval(RegNum)) {
      unsigned PhyReg = VFI->allocatePhyReg(VTM::DRRegClassID,
                                            getBitWidthOf(RegNum));

      assign(*LI, PhyReg);
    }
  }

  return false;
}

bool VRASimple::bindFUs() {
  bindFUsOf(VTM::RMULRegisterClass);
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
