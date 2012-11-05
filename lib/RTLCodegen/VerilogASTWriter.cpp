//===- Writer.cpp - VTM machine instructions to RTL verilog  ----*- C++ -*-===//
//
//                      The Shang HLS frameowrk                               //
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implement the VerilogASTWriter pass, which write VTM machine instructions
// in form of RTL verilog code.
//
//===----------------------------------------------------------------------===//

#include "vtm/Passes.h"
#include "vtm/VerilogAST.h"
#include "vtm/VFInfo.h"
#include "vtm/Utilities.h"
#include "vtm/VerilogModuleAnalysis.h"

#include "llvm/Module.h"
#include "llvm/Target/Mangler.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/Target/TargetData.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/SourceMgr.h"
#include "llvm/Support/CommandLine.h"
#define DEBUG_TYPE "vtm-rtl-codegen"
#include "llvm/Support/Debug.h"

using namespace llvm;

static cl::opt<bool>
EnalbeDumpIR("vtm-dump-ir", cl::desc("Dump the IR to the RTL code."),
             cl::init(false));

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

    void getAnalysisUsage(AnalysisUsage &AU) const {
      MachineFunctionPass::getAnalysisUsage(AU);
      AU.addRequired<VerilogModuleAnalysis>();
      AU.setPreservesAll();
    }
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
  INITIALIZE_PASS_DEPENDENCY(VerilogModuleAnalysis);
INITIALIZE_PASS_END(VerilogASTWriter, "vtm-rtl-info",
                    "Build RTL Verilog module for synthesised function.",
                    false, true)

VerilogASTWriter::VerilogASTWriter(raw_ostream &O)
                : MachineFunctionPass(ID), Out(O) {
  initializeVerilogASTWriterPass(*PassRegistry::getPassRegistry());
}

bool VerilogASTWriter::doInitialization(Module &Mod) {
  TD = getAnalysisIfAvailable<TargetData>();
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
  if (EnalbeDumpIR) {
    Out << "`ifdef wtf_is_this\n" << "Function for RTL Codegen:\n";
    F.print(Out);
    Out << "`endif\n";
  }

  VASTModule *VM = getAnalysis<VerilogModuleAnalysis>().getModule();

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

  // Sequential logic of the registers.
  VM->printRegisterBlocks(Out);

  Out << "\n\n";
  Out << "// Always Block\n";
  Out.always_ff_begin();

  Out.else_begin();

  Out << VM->getControlBlockStr();

  Out.always_ff_end();

  Out.module_end();
  Out.flush();

  return false;
}


