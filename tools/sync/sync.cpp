//===-- sync.cpp - Implement the C Synthesis Code Generator ---------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This is the C Synthesis code generator driver. It provides a convenient
// command-line interface for generating RTL Verilog and various interfacing
// code, given LLVM bitcode.
//
//===----------------------------------------------------------------------===//

#include "llvm/LLVMContext.h"
#include "llvm/Module.h"
#include "llvm/PassManager.h"
#include "llvm/Pass.h"
#include "llvm/ADT/Triple.h"
#include "llvm/Support/IRReader.h"
//#include "llvm/CodeGen/LinkAllAsmWriterComponents.h"
#include "llvm/CodeGen/LinkAllCodegenComponents.h"
#include "llvm/Config/config.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/ManagedStatic.h"
#include "llvm/Support/PluginLoader.h"
#include "llvm/Support/PrettyStackTrace.h"
#include "llvm/Support/ToolOutputFile.h"
#include "llvm/Support/Host.h"
#include "llvm/Support/Signals.h"
#include "llvm/Target/SubtargetFeature.h"
#include "llvm/Target/TargetData.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetRegistry.h"
#include "llvm/Target/TargetSelect.h"
#include <memory>
using namespace llvm;

// General options for sync.  Other pass-specific options are specified
// within the corresponding sync passes, and target-specific options
// and back-end code generation options are specified with the target machine.
//
static cl::opt<std::string>
InputFilename(cl::Positional, cl::desc("<input bitcode>"), cl::init("-"));

namespace llvm {
  extern Target TheVBackendTarget;
}

// main - Entry point for the sync compiler.
//
int main(int argc, char **argv) {
  sys::PrintStackTraceOnErrorSignal();
  
  PrettyStackTraceProgram X(argc, argv);

  // Enable debug stream buffering.
  EnableDebugBuffering = true;

  LLVMContext &Context = getGlobalContext();
  llvm_shutdown_obj Y;  // Call llvm_shutdown() on exit.

  // Initialize target first, so that --version shows registered targets.
  LLVMInitializeVerilogBackendTargetInfo();
  LLVMInitializeVerilogBackendTarget();
  
  cl::ParseCommandLineOptions(argc, argv, "llvm system compiler\n");
  
  // TODO: Run the constraints scritp.

  // Load the module to be compiled...
  SMDiagnostic Err;
  std::auto_ptr<Module> M;

  M.reset(ParseIRFile(InputFilename, Err, Context));
  if (M.get() == 0) {
    Err.Print(argv[0], errs());
    return 1;
  }
  Module &mod = *M.get();

  // TODO: Build the right triple.
  Triple TheTriple(mod.getTargetTriple());

  std::auto_ptr<TargetMachine>
    target(TheVBackendTarget.createTargetMachine(TheTriple.getTriple(), ""));

  // Build up all of the passes that we want to do to the module.
  PassManager PM;

  // TODO: Create the target data from constraints.
  PM.add(new TargetData(&mod));

  // We do not use the stream that passing into addPassesToEmitFile.
  formatted_raw_ostream formatted_nulls(nulls());
  // Ask the target to add backend passes as necessary.
  target->addPassesToEmitFile(PM, formatted_nulls,
                              TargetMachine::CGFT_Null,
                              CodeGenOpt::Aggressive,
                              false/*NoVerify*/);

  PM.run(mod);

  return 0;
}
