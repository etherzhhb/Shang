//===- Writer.cpp - VTM machine instructions to RTL verilog  ----*- C++ -*-===//
//
//                            The Verilog Backend
//
// Copyright: 2010 by Hongbin Zheng. all rights reserved.
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
// This file implement the VerilogASTWriter pass, which write VTM machine instructions
// in form of RTL verilog code.
//
//===----------------------------------------------------------------------===//

#include "vtm/Passes.h"
#include "vtm/VerilogAST.h"
#include "vtm/MicroState.h"
#include "vtm/VFInfo.h"
#include "vtm/Utilities.h"

#include "llvm/Module.h"
#include "llvm/Target/Mangler.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/SourceMgr.h"
#include "llvm/Support/Debug.h"

using namespace llvm;
STATISTIC(TotalRegisterBits,
  "Number of total register bits in synthesised modules");
namespace {
  class VerilogASTWriter : public MachineFunctionPass {
    vlang_raw_ostream Out;

    TargetData *TD;

  public:
    /// @name FunctionPass interface
    //{
    static char ID;
    VerilogASTWriter(raw_ostream &O);
    VerilogASTWriter() : MachineFunctionPass(ID) {
      assert( 0 && "Cannot construct the class without the raw_stream!");
    }

    ~VerilogASTWriter(){}

    bool doInitialization(Module &M);

    bool runOnMachineFunction(MachineFunction &MF);

  };

}

//===----------------------------------------------------------------------===//
char VerilogASTWriter::ID = 0;

Pass *llvm::createVerilogASTWriterPass(raw_ostream &O) {
  return new VerilogASTWriter(O);
}

INITIALIZE_PASS_BEGIN(VerilogASTWriter, "vtm-rtl-info",
  "Build RTL Verilog module for synthesised function.",
  false, true)
  INITIALIZE_PASS_END(VerilogASTWriter, "vtm-rtl-info",
  "Build RTL Verilog module for synthesised function.",
  false, true)

  VerilogASTWriter::VerilogASTWriter(raw_ostream &O) : MachineFunctionPass(ID), Out(O) {
    initializeVerilogASTWriterPass(*PassRegistry::getPassRegistry());
}

bool VerilogASTWriter::doInitialization(Module &Mod) {

  SMDiagnostic Err;
  const char *GlobalScriptPath[] = { "Misc", "RTLGlobalScript" };
  std::string GlobalScript = getStrValueFromEngine(GlobalScriptPath);
  if (!runScriptOnGlobalVariables(Mod, TD, GlobalScript, Err))
    report_fatal_error("VerilogASTWriter: Cannot run globalvariable script:\n"
    + Err.getMessage());

  const char *GlobalCodePath[] = { "RTLGlobalCode" };
  std::string GlobalCode = getStrValueFromEngine(GlobalCodePath);
  Out << GlobalCode << '\n';

  return false;
}

bool VerilogASTWriter::runOnMachineFunction(MachineFunction &F) {
  TD = getAnalysisIfAvailable<TargetData>();
  VFInfo *FInfo =F.getInfo<VFInfo>();

  DEBUG(
    Out << "`ifdef wtf_is_this\n" << "Function for RTL Codegen:\n";
  printVMF(Out, F);
    Out << "`endif\n";
  );

  VASTModule *VM = FInfo->getRtlMod();
  // Building the Slot active signals.
  // FIXME: It is in fact simply printing the logic out.
  VM->buildSlotLogic();

  // TODO: Optimize the RTL net list.
  // FIXME: Do these in seperate passes.
  VM->eliminateConstRegisters();

  // Write buffers to output
  VM->printModuleDecl(Out);
  Out.module_begin();
  Out << "\n\n";
  // Reg and wire
  Out << "// Reg and wire decl\n";
  VM->printSignalDecl(Out);
  Out << "\n\n";
  // Datapath
  Out << "// Datapath\n";
  Out << VM->getDataPathStr();
  VM->printDatapath(Out);

  Out << "\n\n";
  Out << "// Always Block\n";
  Out.always_ff_begin();

  VM->printRegisterReset(Out);
  Out.else_begin();

  VM->printRegisterAssign(Out);
  Out << VM->getControlBlockStr();

  Out.always_ff_end();

  Out.module_end();
  Out.flush();

  return false;
}


