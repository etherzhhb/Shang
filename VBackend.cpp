/*
* Copyright: 2008 by Nadav Rotem. all rights reserved.
* IMPORTANT: This software is supplied to you by Nadav Rotem in consideration
* of your agreement to the following terms, and your use, installation, 
* modification or redistribution of this software constitutes acceptance
* of these terms.  If you do not agree with these terms, please do not use, 
* install, modify or redistribute this software. You may not redistribute, 
* install copy or modify this software without written permission from 
* Nadav Rotem. 
*/
#include "vtm/VTargetMachine.h"
#include "VMCAsmInfo.h"

#include "llvm/PassManager.h"
#include "llvm/Analysis/Verifier.h"
#include "llvm/Assembly/PrintModulePass.h"
#include "llvm/Target/TargetRegistry.h"
#include "llvm/Target/TargetOptions.h"
#include "llvm/CodeGen/LiveVariables.h"
#include "llvm/CodeGen/MachineFunctionAnalysis.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/Transforms/Scalar.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"

#include "vtm/Passes.h"

#define DEBUG_TYPE "vtm-emit-passes"
#include "llvm/Support/Debug.h"

// Include the lua headers (the extern "C" is a requirement because we're
// using C++ and lua has been compiled as C code)
extern "C" {
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
}

// This is the only header we need to include for LuaBind to work
#include "luabind/luabind.hpp"

using namespace llvm;

//===----------------------------------------------------------------------===//
static cl::opt<std::string>
ConfigScriptName("vbe-res-config-script",
                 cl::desc("vbe - The resource config script."));

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
  DataLayout("e-p:64:64-i64:32-f64:32-n1:8:16:32:64"),
  Subtarget(TT, FS),
  TLInfo(*this),
  TSInfo(*this),
  InstrInfo(DataLayout, TLInfo) {

  for (size_t i = 0, e = array_lengthof(ResSet); i != e; ++i)
    ResSet[i] = 0;
  
  initializeTarget();
}

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
  // Standard LLVM-Level Passes.

  // Before running any passes, run the verifier to determine if the input
  // coming from the front-end and/or optimizer is valid.
  if (!DisableVerify)
    PM.add(createVerifierPass());

  // Optionally, tun split-GEPs and no-load GVN.
  if (true/*EnableSplitGEPGVN*/) {
    PM.add(createGEPSplitterPass());
    PM.add(createGVNPass(/*NoLoads=*/true));
  }

  // Run loop strength reduction before anything else.
  if (OptLevel != CodeGenOpt::None && !false/*DisableLSR*/) {
    PM.add(createLoopStrengthReducePass(getTargetLowering()));
    DEBUG(PM.add(createPrintFunctionPass("\n\n*** Code after LSR ***\n",
                                         &dbgs())));
  }

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
  MachineModuleInfo *MMI = new MachineModuleInfo(*getMCAsmInfo());
  PM.add(MMI);

  // Set up a MachineFunction for the rest of CodeGen to work on.
  PM.add(new MachineFunctionAnalysis(*this, OptLevel));

  // Enable FastISel with -fast, but allow that to be overridden.
  if (false/*EnableFastISelOption == cl::BOU_TRUE*/ ||
      (OptLevel == CodeGenOpt::None && false/*EnableFastISelOption != cl::BOU_FALSE*/))
    EnableFastISel = true;

  // Ask the target for an isel.
  if (addInstSelector(PM, OptLevel))
    return true;

  // Print the instruction selected machine code...
  printAndVerify(PM, "After Instruction Selection");

  // PerformBit level information analyze.
  PM.add(createBitLevelInfoPass());

  // Optimize PHIs before DCE: removing dead PHI cycles may make more
  // instructions dead.
  if (OptLevel != CodeGenOpt::None)
    PM.add(createOptimizePHIsPass());

  // If the target requests it, assign local variables to stack slots relative
  // to one another and simplify frame index references where possible.
  PM.add(createLocalStackSlotAllocationPass());

  if (OptLevel != CodeGenOpt::None) {
    // With optimization, dead code should already be eliminated. However
    // there is one known exception: lowered code for arguments that are only
    // used by tail calls, where the tail calls reuse the incoming stack
    // arguments directly (see t11 in test/CodeGen/X86/sibcall.ll).
    PM.add(createDeadMachineInstructionElimPass());
    printAndVerify(PM, "After codegen DCE pass");

    PM.add(createPeepholeOptimizerPass());
    if (!false/*DisableMachineLICM*/)
      PM.add(createMachineLICMPass());
    PM.add(createMachineCSEPass());
    if (!false/*DisableMachineSink*/)
      PM.add(createMachineSinkingPass());
    printAndVerify(PM, "After Machine LICM, CSE and Sinking passes");
  }

  // Pre-ra tail duplication.
  if (OptLevel != CodeGenOpt::None && !false/*DisableEarlyTailDup*/) {
    PM.add(createTailDuplicatePass(true));
    printAndVerify(PM, "After Pre-RegAlloc TailDuplicate");
  }

  // Run pre-ra passes.
  if (addPreRegAlloc(PM, OptLevel))
    printAndVerify(PM, "After PreRegAlloc passes");

  PM.add(createMachineFunctionPrinterPass(dbgs(), "==========MF============"));

  PM.add(new LiveVariables());

  PM.add(createHWAtonInfoPass(*this));

  PM.add(createRTLWriterPass(*this, Out));

  return false;
}

VTargetMachine::~VTargetMachine() {
  for (iterator I = begin(), E = end(); I != E; ++I)
    delete *I;
}

//===----------------------------------------------------------------------===//
/// Resource config implement

template<class BinOpResType>
void VTargetMachine::setupBinOpRes(unsigned latency, unsigned startInt,
                                   unsigned totalRes) {
  ResSet[BinOpResType::getType() - VFUs::FirstFUType]
    = new BinOpResType(latency, startInt, totalRes,
                       // Dirty Hack.
                       64);
}


void VTargetMachine::setupMemBus(unsigned latency, unsigned startInt,
                                 unsigned totalRes) {
  ResSet[VFUs::MemoryBus - VFUs::FirstFUType]
    = new VFUMemBus(latency, startInt, totalRes,
                    DataLayout.getPointerSizeInBits(),
                    // Dirty Hack.
                    64);
}

void VTargetMachine::initializeTarget() {
  std::string ErrMsg;

  lua_State *ScriptState = lua_open();

  luabind::open(ScriptState);

  luabind::module(ScriptState)[
    luabind::class_<VTargetMachine>("VTargetMachine")
      .def("setupMemBus", &VTargetMachine::setupMemBus)
      .def("setupSHL",    &VTargetMachine::setupBinOpRes<VFUSHL>)
      .def("setupASR",    &VTargetMachine::setupBinOpRes<VFUASR>)
      .def("setupLSR",    &VTargetMachine::setupBinOpRes<VFULSR>)
      .def("setupAddSub", &VTargetMachine::setupBinOpRes<VFUAddSub>)
      .def("setupMult",   &VTargetMachine::setupBinOpRes<VFUMult>)
  ];

  luabind::globals(ScriptState)["Config"] = this;

  luaL_dofile(ScriptState, ConfigScriptName.c_str());

  lua_close(ScriptState);
}
