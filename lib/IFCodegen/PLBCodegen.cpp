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
#include "vtm/VFInfo.h"
#include "vtm/LangSteam.h"
#include "vtm/Utilities.h"

#include "llvm/Function.h"
#include "llvm/DerivedTypes.h"

#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"

#include "llvm/ADT/StringExtras.h"

#include "llvm/Support/Format.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/ErrorHandling.h"

#include "llvm/Support/Debug.h"

#include <map>

using namespace llvm;

//===----------------------------------------------------------------------===//
namespace {
struct PLBIfCodegen : public MachineFunctionPass {
  static char ID;

  // Streams
  vlang_raw_ostream Out;

  VASTModule *RTLMod;
  const VFInfo *FInfo;

  PLBIfCodegen(raw_ostream &O) : MachineFunctionPass(ID), Out(O),
    RTLMod(0), FInfo(0) {}

  void getAnalysisUsage(AnalysisUsage &AU) const {
    MachineFunctionPass::getAnalysisUsage(AU);
    AU.setPreservesAll();
  }

  bool runOnMachineFunction(MachineFunction &MF);
};
}

char PLBIfCodegen::ID = 0;

Pass *llvm::createPLBIfCodegenPass(raw_ostream &O) {
  return new PLBIfCodegen(O);
}

bool PLBIfCodegen::runOnMachineFunction(MachineFunction &MF) {
  Out << "// IPIF User Logic for PLB.\n";
  return false;
}


