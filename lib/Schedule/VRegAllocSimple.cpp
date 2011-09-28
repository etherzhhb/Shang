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

  typedef SmallVector<const LiveInterval*, 16> LiveSet;
  typedef std::map<unsigned,LiveSet> LiveMap;
  LiveMap LiveIntervalMap;

  typedef SmallVector<unsigned, 128> VirRegSet;
  typedef SmallVector<VirRegSet, 128> FUSet;
  typedef std::map<unsigned,FUSet> FUMap;
  FUMap physFU;


public:
  VRASimple();
  void init(VirtRegMap &vrm, LiveIntervals &lis);
  
  static char ID;

  void getAnalysisUsage(AnalysisUsage &AU) const;
  void releaseMemory();

  void collectLiveInterval(MachineRegisterInfo &MRI);

  Spiller &spiller() {
    llvm_unreachable("VRegAllocSimple - Never spill!");
    return *(Spiller*)0;
  }

  virtual float getPriority(LiveInterval *LI) { return LI->weight; }

  virtual void enqueue(LiveInterval *LI) {
    unsigned Reg = LI->reg;

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

  // DIRTY HACK: If all virtual registers is 64 bits,
  // we have N * 8 byte registers.
  PhysReg2LiveUnion.init(UnionAllocator, MRI->getNumVirtRegs() * 8);
  // Cache an interferece query for each physical reg
  Queries.reset(new LiveIntervalUnion::Query[PhysReg2LiveUnion.numRegs()]);

}

bool VRASimple::runOnMachineFunction(MachineFunction &F) {
  MF = &F;
  VFI = F.getInfo<VFInfo>();
  BLI = &getAnalysis<BitLevelInfo>();
  MachineRegisterInfo &MRI =F.getRegInfo();

  init(getAnalysis<VirtRegMap>(), getAnalysis<LiveIntervals>());

  DEBUG(dbgs() << "Before simple register allocation:\n";
        printVMF(dbgs(), F);
  );

  collectLiveInterval(MRI);

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
  LiveIntervalMap.clear();

  DEBUG(dbgs() << "After simple register allocation:\n";
        printVMF(dbgs(), F);
  ); 

  return true;
}

void VRASimple::collectLiveInterval(MachineRegisterInfo &MRI){
  unsigned a[4][128][10];
  unsigned FUtype;
  unsigned specificFU = 0;
  unsigned VirRegCounter = 0;

  //collect all liveintervals and put them in the map respectively according to the
  //different types.
  for (LiveIntervals::iterator I = LIS->begin(), E = LIS->end(); I != E; ++I) {
    unsigned RegNum = I->first;
    LiveInterval *VirtReg = I->second;
    const TargetRegisterClass *RC = MRI.getRegClass(RegNum);
    if(RC->getID() != VTM::WireRegClassID)
      LiveIntervalMap[RC->getID()].push_back(VirtReg);  
  }

  //print the content in map for testing
  for (LiveMap::iterator I = LiveIntervalMap.begin(), E = LiveIntervalMap.end();
    I != E; ++I) {
      unsigned Id = I->first;
      dbgs() << "reg type is " << Id <<"\n";
      for (LiveSet::iterator UI = I->second.begin(),
        UE = I->second.end(); UI != UE; ++UI) {
        const LiveInterval *SI  = *UI;
        unsigned RegNum = SI->reg;
        dbgs() << "include following reg:"<<RegNum<<"\n";

      }
  }

  // bind the virReg to the specific FU 
  for (LiveMap::iterator I = LiveIntervalMap.begin(), E = LiveIntervalMap.end();
    I != E; ++I) {
      FUtype = I->first;
      VirRegCounter = 0;
      for (LiveSet::iterator UI = I->second.begin(),
        UE = I->second.end(); UI != UE; ++UI) {
          LiveInterval LI = **UI;
          unsigned interfReg = checkPhysRegInterference(LI, specificFU);
          if(interfReg == 0){
            a[FUtype][specificFU][VirRegCounter] = LI.reg;
            VirRegCounter++;
          }
          else{
            specificFU++;
            VirRegCounter = 0;
            return;
          }
      }
  }

  //check the VirReg to FU array
  for(int i = 0 ; i< 5 ; i++){
    dbgs()<<"the FUtype is"<<""<<i<<"\n";
    for(int j =0 ; j<2; j++){     
      dbgs()<<"the binding FU is"<<""<<j<<"\n";
      for(int k =0;k<4; k++)
        dbgs()<<"contains virReg is"<<""<<a[i][j][k]<<"\n";
    }
  }
          
}
//liveset.resize(unsigned N) {
unsigned VRASimple::checkPhysRegInterference(LiveInterval &VirtReg,
                                             unsigned PhysReg) {
  unsigned Overlaps[16];

  for (unsigned i = 0, e = VFI->getOverlaps(PhysReg, Overlaps); i < e; ++i)
    if (query(VirtReg, Overlaps[i]).checkInterference())
      return Overlaps[i];

  return 0;
}
unsigned VRASimple::selectOrSplit(LiveInterval &VirtReg,
                                  SmallVectorImpl<LiveInterval*> &splitLVRs) {
  unsigned VReg = VirtReg.reg;
  unsigned Size = BLI->getBitWidth(VReg);
  if (Size < 8) Size = 8;
  // Since we are allocating register with witdh of 2^N, round up the size.
  Size = NextPowerOf2(Size - 1) / 8;

  typedef VFInfo::phyreg_iterator reg_it;
  if (EnableSimpleRegisterSharing)
    for (reg_it I = VFI->phyreg_begin(Size), E = VFI->phyreg_end(Size);
         I < E; ++I) {
      unsigned PhysReg = *I;
      if (checkPhysRegInterference(VirtReg, PhysReg) == 0)
        return PhysReg;
    }

  unsigned Reg =  VFI->allocatePhyReg(Size);

  while (checkPhysRegInterference(VirtReg, Reg) != 0)
    Reg =  VFI->allocatePhyReg(Size);

  return Reg;
}
