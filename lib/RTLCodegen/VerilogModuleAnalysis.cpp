//===--------------------VerilogModuleAnalysis.cpp------------------------===//
//
//                      The Shang HLS frameowrk                               //
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Implement the VerilogModuleAnalysis pass, which is the container of the
// VASTModule.
//
//===----------------------------------------------------------------------===//
#include "vtm/Passes.h"
#include "vtm/VerilogModuleAnalysis.h"
#include "vtm/VerilogAST.h"
#include "vtm/VerilogAST.h"

using namespace llvm;

INITIALIZE_PASS(VerilogModuleAnalysis, "verilog-module-analysis",
                "verilog module analysis", false, true)

char VerilogModuleAnalysis::ID = 0;

VerilogModuleAnalysis::VerilogModuleAnalysis():MachineFunctionPass(ID),Module(0)
{
  initializeVerilogModuleAnalysisPass(*PassRegistry::getPassRegistry());
}

bool VerilogModuleAnalysis::runOnMachineFunction(MachineFunction &MF){
  return false;
}

VASTModule *VerilogModuleAnalysis::createModule(const std::string &Name,
                                                VASTExprBuilder *Builder) {
  assert(Module == 0 && "Module has been already created!");
  Module = new VASTModule(Name, Builder);
  return Module;
}

void VerilogModuleAnalysis::releaseMemory() {
  if (Module == 0) return;

  delete Module;
  Module = 0;
}
