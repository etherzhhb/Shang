//===------ RegAllocOptimalSSA.cpp - Optimal Register Sharing for HLS -----===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implement the register sharing algorithm descripted in
//  Optimal Register Sharing for High-Level Synthesis of SSA Form Programs
//  Philip Brisk, Foad Dabiri, Student Member, IEEE,
//  Roozbeh Jafari, Student Member, IEEE,
//  and Majid Sarrafzadeh, Fellow, IEEE
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "ssa-regalloc"
#include "vtm/Passes.h"

#include "LiveIntervalUnion.h"
#include "RegAllocBase.h"
#include "RenderMachineFunction.h"
#include "Spiller.h"
#include "VirtRegRewriter.h"
#include "llvm/Function.h"
#include "llvm/PassAnalysisSupport.h"
#include "llvm/CodeGen/CalcSpillWeights.h"
#include "llvm/CodeGen/MachineDominators.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineLoopInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/RegAllocRegistry.h"
#include "llvm/CodeGen/RegisterCoalescer.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetOptions.h"
#include "llvm/ADT/DepthFirstIterator.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"

#include "VirtRegMap.h"
#include "llvm/CodeGen/LiveIntervalAnalysis.h"
#include "llvm/Target/TargetRegisterInfo.h"


#include <vector>
#include <queue>

using namespace llvm;

static RegisterRegAlloc OptimalSSARegAlloc("optimal-ssa", 
                                           "Optimal SSA form register allocator",
                                           createOptimalSSARegisterAllocator);

namespace {

/// RAOptimalSSA provides a minimal implementation of the basic register allocation
/// algorithm. It prioritizes live virtual registers by spill weight and spills
/// whenever a register is unavailable. This is not practical in production but
/// provides a useful baseline both for measuring other allocators and comparing
/// the speed of the basic algorithm against other styles of allocators.
class RAOptimalSSA : public MachineFunctionPass, public RegAllocBase {
  // context
  MachineFunction *MF;
  const TargetMachine *TM;
  MachineRegisterInfo *MRI;

  // Remember the status of allocated registers.
  std::vector<bool> FreeRegs;
  void freePhyReg(unsigned PReg) {
    FreeRegs[PReg - 1] = true;
  }

  bool isPhyRegFree(unsigned PReg) {
    return FreeRegs[PReg - 1];
  }

  unsigned getAvailablePhyReg() {
    std::vector<bool>::iterator I = std::find(FreeRegs.begin(), FreeRegs.end(), true);
    if (I != FreeRegs.end())
      return I - FreeRegs.begin() + 1;
    
    // Create a new physics register if necessary.
    FreeRegs.push_back(true);
    return FreeRegs.size();
  }

public:
  RAOptimalSSA();

  /// Return the pass name.
  virtual const char* getPassName() const {
    return "Optimal SSA form register allocato";
  }

  /// RAOptimalSSA analysis usage.
  virtual void getAnalysisUsage(AnalysisUsage &AU) const;

  virtual void releaseMemory();

  /// Perform register allocation.
  virtual bool runOnMachineFunction(MachineFunction &mf);

  void allocatePhysRegs(MachineBasicBlock *MBB);

  virtual Spiller &spiller() {
    assert(0 && "VTM never spill!");
    return *(Spiller*)0;
  }

  virtual unsigned selectOrSplit(LiveInterval &lvr,
                                 SmallVectorImpl<LiveInterval*> &splitLVRs);

  static char ID;
};

char RAOptimalSSA::ID = 0;

} // end anonymous namespace

// We should not need to publish the initializer as long as no other passes
// require RAOptimalSSA.
#if 0 // disable INITIALIZE_PASS
INITIALIZE_PASS_BEGIN(RAOptimalSSA, "optimal-ssa-regalloc",
                      "Optimal SSA Register Allocator", false, false)
INITIALIZE_PASS_DEPENDENCY(LiveIntervals)
INITIALIZE_PASS_DEPENDENCY(StrongPHIElimination)
INITIALIZE_AG_DEPENDENCY(RegisterCoalescer)
INITIALIZE_PASS_DEPENDENCY(MachineLoopInfo)
INITIALIZE_PASS_DEPENDENCY(VirtRegMap)
#ifndef NDEBUG
INITIALIZE_PASS_DEPENDENCY(RenderMachineFunction)
#endif
INITIALIZE_PASS_END(RAOptimalSSA, "optimal-ssa-regalloc",
                    "Optimal SSA Register Allocator", false, false)
#endif // disable INITIALIZE_PASS

RAOptimalSSA::RAOptimalSSA(): MachineFunctionPass(ID) {
  initializeLiveIntervalsPass(*PassRegistry::getPassRegistry());
  initializeSlotIndexesPass(*PassRegistry::getPassRegistry());
  initializeStrongPHIEliminationPass(*PassRegistry::getPassRegistry());
  initializeRegisterCoalescerAnalysisGroup(*PassRegistry::getPassRegistry());
  initializeMachineDominatorTreePass(*PassRegistry::getPassRegistry());
  initializeMachineLoopInfoPass(*PassRegistry::getPassRegistry());
  initializeVirtRegMapPass(*PassRegistry::getPassRegistry());
}

void RAOptimalSSA::getAnalysisUsage(AnalysisUsage &au) const {
  au.setPreservesCFG();
  au.addRequired<LiveIntervals>();
  au.addPreserved<SlotIndexes>();
  if (StrongPHIElim)
    au.addRequiredID(StrongPHIEliminationID);
  au.addRequiredTransitive<RegisterCoalescer>();
  au.addRequiredID(MachineDominatorsID);
  au.addPreservedID(MachineDominatorsID);
  au.addRequired<MachineLoopInfo>();
  au.addPreserved<MachineLoopInfo>();
  au.addRequired<VirtRegMap>();
  au.addPreserved<VirtRegMap>();
  MachineFunctionPass::getAnalysisUsage(au);
}

void RAOptimalSSA::releaseMemory() {
  FreeRegs.clear();
  RegAllocBase::releaseMemory();
}

//===----------------------------------------------------------------------===//
//                         RAOptimalSSA Implementation
//===----------------------------------------------------------------------===//

bool RAOptimalSSA::runOnMachineFunction(MachineFunction &mf) {
  DEBUG(dbgs() << "********** BASIC REGISTER ALLOCATION **********\n"
               << "********** Function: "
               << ((Value*)mf.getFunction())->getName() << '\n');

  MF = &mf;
  TM = &mf.getTarget();
  MRI = &mf.getRegInfo(); 
  MachineDominatorTree &DT = getAnalysis<MachineDominatorTree>();

  const TargetRegisterInfo *TRI = TM->getRegisterInfo();
  RegAllocBase::init(*TRI, getAnalysis<VirtRegMap>(),
                     getAnalysis<LiveIntervals>());

  for (df_iterator<MachineDominatorTree*> I = df_begin(&DT), E = df_end(&DT);
       I != E;++I)
    allocatePhysRegs(I->getBlock());
  
  // Add MBB live ins

  // Run rewriter
  std::auto_ptr<VirtRegRewriter> rewriter(createVirtRegRewriter());
  rewriter->runOnMachineFunction(mf, *vrm_, lis_);

  // The pass output is in VirtRegMap. Release all the transient data.
  releaseMemory();
  
  return true;
}

void RAOptimalSSA::allocatePhysRegs(MachineBasicBlock *MBB) {
  for (MachineBasicBlock::iterator I = MBB->begin(), E = MBB->end();
      I != E; ++I) {
    MachineInstr &MI = *I;
    for (MachineInstr::mop_iterator OI = MI.operands_begin(),
        OE = MI.operands_end(); OI != OE; ++OI) {
      // Allocate register for the value define by the machine instruction.
      MachineOperand &MO = *OI;
      if (!MO.isReg()) continue;
      
      unsigned Reg = MO.getReg();
      assert(Reg && "Whats this?");

      // Check if we can free some registers.
      if (MO.isUse() && MO.isKill()) {
        unsigned PhyReg = vrm_->getPhys(Reg);
        assert(PhyReg && "Bad PhyReg");
        freePhyReg(PhyReg);
      } else if (MO.isDef()) { // Allocate register for defines.
        assert(!vrm_->hasPhys(Reg) && "duplicate vreg in interval unions");
        unsigned PhyReg = getAvailablePhyReg();
        vrm_->assignVirt2Phys(Reg, PhyReg);
        //
        // physReg2liu_[PhyReg].unify(lis_->getInterval(Reg));
      }
    }
  }
}

unsigned RAOptimalSSA::selectOrSplit(LiveInterval &lvr,
                                     SmallVectorImpl<LiveInterval*> &splitLVRs) {
  return 0;
}

FunctionPass* llvm::createOptimalSSARegisterAllocator() {
  return new RAOptimalSSA();
}
