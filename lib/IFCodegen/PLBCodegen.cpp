//===-- PLBCodegen - Interface Generation for Xilinx PLB ------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file hardware interface generation pass for xilinx PLB.
//
//===----------------------------------------------------------------------===//


#include "vtm/Passes.h"
#include "vtm/VerilogAST.h"
#include "vtm/VFuncInfo.h"
#include "vtm/FileInfo.h"
#include "vtm/LangSteam.h"
#include "vtm/Utilities.h"

#include "llvm/Function.h"
#include "llvm/DerivedTypes.h"

#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"

#include "llvm/ADT/StringExtras.h"

#include "llvm/Support/Format.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/ToolOutputFile.h"
#include "llvm/Support/ErrorHandling.h"

#include "llvm/Support/Debug.h"

#include <map>

using namespace llvm;

//===----------------------------------------------------------------------===//
namespace {
struct PLBIfCodegen : public MachineFunctionPass {
  static char ID;

  // Streams
  tool_output_file *FOut;
  vlang_raw_ostream Stream;

  VASTModule *RTLMod;
  const VFuncInfo *FuncInfo;

  PLBIfCodegen() : MachineFunctionPass(ID), FOut(0), Stream(),
    RTLMod(0), FuncInfo(0) {}

  void getAnalysisUsage(AnalysisUsage &AU) const {
    MachineFunctionPass::getAnalysisUsage(AU);
    AU.setPreservesAll();
  }

  bool doInitialization(Module &M) {
    // Setup the streams.
    FOut = vtmfiles().getIFDvrOut("plb.v");
    Stream.setStream(FOut->os());

    return false;
  }

  bool doFinalization(Module &M) {
    Stream.flush();
    FOut->keep();
    return false;
  }

  bool runOnMachineFunction(MachineFunction &MF);
};
}

char PLBIfCodegen::ID = 0;

Pass *llvm::createPLBIfCodegenPass() {
  return new PLBIfCodegen();
}

bool PLBIfCodegen::runOnMachineFunction(MachineFunction &MF) {
  Stream << "// IPIF User Logic for PLB.\n";
  return false;
}


