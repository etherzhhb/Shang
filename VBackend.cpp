//===- VerilogTargetMachine.h - TargetMachine for Verilog Backend -*- C++ -*-=//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
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
#include "VMCAsmInfo.h"

#include "vtm/Passes.h"
#include "vtm/SynSettings.h"

#include "llvm/PassManager.h"
#include "llvm/Analysis/Verifier.h"
#include "llvm/Analysis/Passes.h"
#include "llvm/Assembly/PrintModulePass.h"
#include "llvm/Target/TargetRegistry.h"
#include "llvm/Target/TargetOptions.h"
#include "llvm/Target/TargetAsmInfo.h"
#include "llvm/CodeGen/LiveVariables.h"
#include "llvm/CodeGen/MachineFunctionAnalysis.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/Transforms/Scalar.h"
#include "llvm/Support/CommandLine.h"
#define DEBUG_TYPE "vtm-emit-passes"
#include "llvm/Support/Debug.h"


//===----------------------------------------------------------------------===//

extern "C" void LLVMInitializeVerilogBackendTarget() {
  // Register the target.
  RegisterTargetMachine<VTargetMachine> X(TheVBackendTarget);
  RegisterAsmInfo<VMCAsmInfo> Y(TheVBackendTarget);
}

VTargetMachine::VTargetMachine(const Target &T, const std::string &TT,
                               const std::string &FS)
  : LLVMTargetMachine(T, TT),
  // FIXME: Allow speicific data layout.
  DataLayout(FS),
  Subtarget(TT, ""),
  TLInfo(*this),
  TSInfo(*this),
  InstrInfo(DataLayout, TLInfo),
  FrameInfo(Subtarget) {}

bool VTargetMachine::addInstSelector(PassManagerBase &PM,
                                     CodeGenOpt::Level OptLevel) {
  PM.add(createVISelDag(*this, OptLevel));
  return false;
}

// DIRTYHACK: Copy form LLVMTargetMachine.cpp
static void printAndVerify(PassManagerBase &PM,
                           const char *Banner) {
  if (PrintMachineCode)
    PM.add(createMachineFunctionPrinterPass(dbgs(), Banner));

   PM.add(createMachineVerifierPass());
}

bool VTargetMachine::addPassesToEmitFile(PassManagerBase &PM,
                                         formatted_raw_ostream &Out,
                                         CodeGenFileType FileType,
                                         CodeGenOpt::Level OptLevel,
                                         bool DisableVerify) {

  // add the pass which will convert the AllocaInst to GlobalVariable.
  PM.add(createStackToGlobalPass());

  // The ContoBromPass
  // PM.add(createContoBromPass(*getIntrinsicInfo()));

  // Dirty Hack: Map all frame stuffs to bram 1.
  // PM.add(createLowerFrameInstrsPass(*getIntrinsicInfo()));

  // Standard LLVM-Level Passes.

  // Before running any passes, run the verifier to determine if the input
  // coming from the front-end and/or optimizer is valid.
  if (!DisableVerify)
    PM.add(createVerifierPass());

  // Optionally, tun split-GEPs and no-load GVN.
  if (true/*EnableSplitGEPGVN*/) {
    //PM.add(createGEPSplitterPass());
    PM.add(createGVNPass(/*NoLoads=*/true));
  }

  // Run loop strength reduction before anything else.
  if (OptLevel != CodeGenOpt::None && !false/*DisableLSR*/) {

    //PM.add(createLoopStrengthReducePass(getTargetLowering()));
    DEBUG(PM.add(createPrintFunctionPass("\n\n*** Code after LSR ***\n",
                                         &dbgs())));
  }

  PM.add(createCFGSimplificationPass());

  PM.add(createGCLoweringPass());

  // Make sure that no unreachable blocks are instruction selected.
  PM.add(createUnreachableBlockEliminationPass());

  // Turn exception handling constructs into something the code generators can
  // handle.
  PM.add(createLowerInvokePass(getTargetLowering()));

  // The lower invoke pass may create unreachable code. Remove it.
  PM.add(createUnreachableBlockEliminationPass());

  if (OptLevel != CodeGenOpt::None && !false/*DisableCGP*/)
    PM.add(createCodeGenPreparePass(getTargetLowering()));

  PM.add(createStackProtectorPass(getTargetLowering()));

  addPreISel(PM, OptLevel);

  DEBUG(
    PM.add(createPrintFunctionPass("\n\n"
                                   "*** Final LLVM Code input to ISel ***\n",
                                   &dbgs()));
  );

  // All passes which modify the LLVM IR are now complete; run the verifier
  // to ensure that the IR is valid.
  if (!DisableVerify)
    PM.add(createVerifierPass());

  // Standard Lower-Level Passes.

  // Install a MachineModuleInfo class, which is an immutable pass that holds
  // all the per-module stuff we're generating, including MCContext.
  TargetAsmInfo *TAI = new TargetAsmInfo(*this);
  MachineModuleInfo *MMI = new MachineModuleInfo(*getMCAsmInfo(), TAI);
  PM.add(MMI);

  // Set up a MachineFunction for the rest of CodeGen to work on.
  PM.add(new MachineFunctionAnalysis(*this, OptLevel));

  // Enable FastISel with -fast, but allow that to be overridden.
  if (false/*EnableFastISelOption == cl::BOU_TRUE*/ ||
      (OptLevel == CodeGenOpt::None && false/*EnableFastISelOption != cl::BOU_FALSE*/))
    EnableFastISel = true;

  // Provide BasicAA for target independent codegen backend.
  PM.add(createBasicAliasAnalysisPass());

  // Ask the target for an isel.
  if (addInstSelector(PM, OptLevel))
    return true;

  // Print the instruction selected machine code...
  printAndVerify(PM, "After Instruction Selection");

  // PerformBit level information analyze.
  PM.add(createBitLevelInfoPass());
  // Optimize PHIs before DCE: removing dead PHI cycles may make more
  // instructions dead.
  PM.add(createOptimizePHIsPass());

  // If the target requests it, assign local variables to stack slots relative
  // to one another and simplify frame index references where possible.
  PM.add(createLocalStackSlotAllocationPass());

  // With optimization, dead code should already be eliminated. However
  // there is one known exception: lowered code for arguments that are only
  // used by tail calls, where the tail calls reuse the incoming stack
  // arguments directly (see t11 in test/CodeGen/X86/sibcall.ll).
  PM.add(createDeadMachineInstructionElimPass());
  printAndVerify(PM, "After codegen DCE pass");

  PM.add(createPeepholeOptimizerPass());

  PM.add(createMachineLICMPass());
  PM.add(createMachineCSEPass());
  PM.add(createMachineSinkingPass());
  printAndVerify(PM, "After Machine LICM, CSE and Sinking passes");

  // Pre-ra tail duplication.
  //PM.add(createTailDuplicatePass(true));
  //printAndVerify(PM, "After Pre-RegAlloc TailDuplicate");

  // Run pre-ra passes.
  if (addPreRegAlloc(PM, OptLevel))
    printAndVerify(PM, "After PreRegAlloc passes");


  PM.add(createFixTerminatorsPass());
  PM.add(createMergeFallThroughBlocksPass());
  printAndVerify(PM, "After merge fall through pass");

  // Make sure we have a branch instruction for every success block.
  PM.add(createFixTerminatorsPass());

  PM.add(createMachineCSEPass());
  // Fix machine code so we can handle them easier.
  PM.add(createFixMachineCodePass());
  // Construct multiplexer tree for prebound function units.
  PM.add(createPrebindMuxPass());

  // With optimization, dead code should already be eliminated. However
  // there is one known exception: lowered code for arguments that are only
  // used by tail calls, where the tail calls reuse the incoming stack
  // arguments directly (see t11 in test/CodeGen/X86/sibcall.ll).
  PM.add(createDeadMachineInstructionElimPass());

  // Run the SCEVAA pass to compute more accurate alias information.
  PM.add(createScalarEvolutionAliasAnalysisPass());

  // Schedule.
  PM.add(createVPreRegAllocSchedPass());

  // Forward the register used by wire operations so we can compute live
  // interval correctly.
  PM.add(createForwardWireUsersPass());

  PM.add(createSimpleRegisterAllocator());

  PM.add(createRTLCodegenPreparePass());

  return false;
}
