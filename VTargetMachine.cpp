//===- VerilogTargetMachine.h - TargetMachine for Verilog Backend -*- C++ -*-=//
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
// This file implement the Verilog TargetMachine, we can leverage existing llvm
// low level optimization with Verilog TargetMachine by:
//   Translate LLVM IR to Verilog TargetMachine Code, perform low level
//     optimization.
//   Translate Verilog TargetMachine code to schedule units and perform schedule.
//   Perform register allocation with existing register allocation passes.
//
//===----------------------------------------------------------------------===//
//#include "VFrameLowering.h"
#include "VTargetMachine.h"

#include "vtm/Passes.h"
#include "vtm/SynSettings.h"

#include "llvm/PassManager.h"
#include "llvm/Analysis/Verifier.h"
#include "llvm/Analysis/Passes.h"
#include "llvm/Assembly/PrintModulePass.h"
#include "llvm/Target/TargetOptions.h"
#include "llvm/Target/TargetLoweringObjectFile.h"
#include "llvm/CodeGen/MachineFunctionAnalysis.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/Transforms/Scalar.h"
#include "llvm/Support/TargetRegistry.h"
#define DEBUG_TYPE "vtm-emit-passes"
#include "llvm/Support/Debug.h"

using namespace llvm;
//===----------------------------------------------------------------------===//

extern "C" void LLVMInitializeVerilogBackendTarget() {
  // Register the target.
  RegisterTargetMachine<VTargetMachine> X(TheVBackendTarget);
  //RegisterAsmInfo<VMCAsmInfo> Y(TheVBackendTarget);
}

VTargetMachine::VTargetMachine(const Target &T, StringRef TT,StringRef CPU,
                               StringRef FS, TargetOptions Options, Reloc::Model RM,
                               CodeModel::Model CM, CodeGenOpt::Level OL)
  : LLVMTargetMachine(T, TT, "generic", "", Options, RM, CM, OL),
  // FIXME: Allow speicific data layout.
  DataLayout(FS),
  TLInfo(*this),
  TSInfo(*this),
  InstrInfo(),
  FrameInfo() {}

bool VTargetMachine::addInstSelector(PassManagerBase &PM) {
  PM.add(createVISelDag(*this));
  return false;
}

namespace {
struct VTMPassConfig : public TargetPassConfig {
  VTMPassConfig(VTargetMachine *TM, PassManagerBase &PM)
    : TargetPassConfig(TM, PM) {}

  virtual bool addPreRegAlloc() {
    PM.add(createBasicAliasAnalysisPass());
    // Run the SCEVAA pass to compute more accurate alias information.
    PM.add(createScalarEvolutionAliasAnalysisPass());
    PM.add(createVPreRegAllocSchedPass());
    PM.add(createForwardWireUsersPass());
    //addPass(FinalizeMachineBundlesID);
    return true;
  }

  virtual bool addInstSelector() {
    PM.add(createVISelDag(getTM<VTargetMachine>()));
    return false;
  }

  virtual bool addFinalizeRegAlloc() {
    PM.add(createRTLCodegenPreparePass());
    // Generate the code.
    PM.add(createVerilogASTBuilderPass());

    // Analyse the dependency between registers.
    PM.add(createRtlSSAAnalysisPass());

    return true;
  }

  virtual void addMachineSSAOptimization() {
    // Annotate the bit level information.
    PM.add(createBitLevelInfoPass());

    // Pre-ra tail duplication.
    if (addPass(EarlyTailDuplicateID) != &NoPassID)
      printAndVerify("After Pre-RegAlloc TailDuplicate");

    // Optimize PHIs before DCE: removing dead PHI cycles may make more
    // instructions dead.
    addPass(OptimizePHIsID);

    // If the target requests it, assign local variables to stack slots relative
    // to one another and simplify frame index references where possible.
    addPass(LocalStackSlotAllocationID);

    // With optimization, dead code should already be eliminated. However
    // there is one known exception: lowered code for arguments that are only
    // used by tail calls, where the tail calls reuse the incoming stack
    // arguments directly (see t11 in test/CodeGen/X86/sibcall.ll).
    addPass(DeadMachineInstructionElimID);
    printAndVerify("After codegen DCE pass");

    addPass(MachineLICMID);
    addPass(MachineCSEID);
    addPass(MachineSinkingID);
    printAndVerify("After Machine LICM, CSE and Sinking passes");

    addPass(PeepholeOptimizerID);
    printAndVerify("After codegen peephole optimization pass");

    // Fix the machine code to avoid unnecessary mux.
    PM.add(createFixMachineCodePass(true));

    // Construct multiplexer tree for prebound function units.
    PM.add(createPrebindUnbalanceMuxPass());
    //PM.add(createPrebindMuxBasePass());

    // Optimize the CFG.
    PM.add(createFixTerminatorsPass());
    PM.add(createHyperBlockFormationPass());
    printAndVerify("After merge fall through pass.");
    // Make sure we have a branch instruction for every success block.
    PM.add(createFixTerminatorsPass());

    // Fix the machine code for schedule and function unit allocation.
    PM.add(createFixMachineCodePass(false));

    // Perform logic synthesis.
    PM.add(createLogicSynthesisPass());
    printAndVerify("After logic synthesis.");

    // Clean up the MachineFunction.
    addPass(MachineCSEID);
    // Clean up the MachineFunction.
    addPass(DeadMachineInstructionElimID);
  }

  virtual void addOptimizedRegAlloc(FunctionPass *RegAllocPass) {
    // LiveVariables currently requires pure SSA form.
    //
    // FIXME: Once TwoAddressInstruction pass no longer uses kill flags,
    // LiveVariables can be removed completely, and LiveIntervals can be directly
    // computed. (We still either need to regenerate kill flags after regalloc, or
    // preferably fix the scavenger to not depend on them).
    addPass(LiveVariablesID);

    // Add passes that move from transformed SSA into conventional SSA. This is a
    // "copy coalescing" problem.
    //
    // if (!EnableStrongPHIElim) {
      // Edge splitting is smarter with machine loop info.
      addPass(MachineLoopInfoID);
      addPass(PHIEliminationID);
    // }
    // addPass(TwoAddressInstructionPassID);

    // FIXME: Either remove this pass completely, or fix it so that it works on
    // SSA form. We could modify LiveIntervals to be independent of this pass, But
    // it would be even better to simply eliminate *all* IMPLICIT_DEFs before
    // leaving SSA.
    addPass(ProcessImplicitDefsID);

    //if (EnableStrongPHIElim)
    //  addPass(StrongPHIEliminationID);

    addPass(RegisterCoalescerID);

    // Add the selected register allocation pass.
    PM.add(RegAllocPass);
    printAndVerify("After Register Allocation");

    // FinalizeRegAlloc is convenient until MachineInstrBundles is more mature,
    // but eventually, all users of it should probably be moved to addPostRA and
    // it can go away.  Currently, it's the intended place for targets to run
    // FinalizeMachineBundles, because passes other than MachineScheduling an
    // RegAlloc itself may not be aware of bundles.
    if (addFinalizeRegAlloc())
      printAndVerify("After RegAlloc finalization");

    // Perform stack slot coloring and post-ra machine LICM.
    //
    // FIXME: Re-enable coloring with register when it's capable of adding
    // kill markers.
    // addPass(StackSlotColoringID);

    // Run post-ra machine LICM to hoist reloads / remats.
    //
    // FIXME: can this move into MachineLateOptimization?
    // addPass(PostRAMachineLICMID);

    // printAndVerify("After StackSlotColoring and postra Machine LICM");
  }

  virtual void addMachinePasses() {
    // Print the instruction selected machine code...
    printAndVerify("After Instruction Selection");

    // Expand pseudo-instructions emitted by ISel.
    addPass(ExpandISelPseudosID);

    // Add passes that optimize machine instructions in SSA form.
    //if (getOptLevel() != CodeGenOpt::None) {
      addMachineSSAOptimization();
    //}
    //else {
    //  // If the target requests it, assign local variables to stack slots relative
    //  // to one another and simplify frame index references where possible.
    //  addPass(LocalStackSlotAllocationID);
    //}

    // Run pre-ra passes.
    if (addPreRegAlloc())
      printAndVerify("After PreRegAlloc passes");

    // Run register allocation and passes that are tightly coupled with it,
    // including phi elimination and scheduling.
    //if (getOptimizeRegAlloc())
      addOptimizedRegAlloc(createSimpleRegisterAllocator());
    //else
    //  addFastRegAlloc(createRegAllocPass(false));
  }

  virtual void addIRPasses() {
    // add the pass which will convert the AllocaInst to GlobalVariable.
    PM.add(createStackToGlobalPass());

    // The construct block ram for local memory access.
    // PM.add(createContoBromPass(*getIntrinsicInfo()));

    // Dirty Hack: Map all frame stuffs to bram 1.
    // PM.add(createLowerFrameInstrsPass(*getIntrinsicInfo()));

    TargetPassConfig::addIRPasses();

    // Turn exception handling constructs into something the code generators can
    // handle.
    PM.add(createLowerInvokePass(getTargetLowering()));

    PM.add(createCFGSimplificationPass());
  }
};
} // namespace

TargetPassConfig *VTargetMachine::createPassConfig(PassManagerBase &PM) {
  return new VTMPassConfig(this, PM);
}

bool VTargetMachine::addPassesToEmitFile(PassManagerBase &PM,
                                         formatted_raw_ostream &Out,
                                         CodeGenFileType FileType,
                                         bool DisableVerify) {
  //addPassesToGenerateCode
  //  PassConfig->addIRPasses();
  //  addPassesToHandleExceptions
  //  addISelPrepare
  //  addMachinePasses
  LLVMTargetMachine::addPassesToEmitFile(PM, Out, FileType, DisableVerify);

  PM.add(createGCInfoDeleter());
  return false;
}
