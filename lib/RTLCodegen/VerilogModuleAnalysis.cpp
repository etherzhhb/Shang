//===--------------------VerilogModuleAnalysis.cpp------------------------===//
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
