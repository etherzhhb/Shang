//===------------------------VerilogModuleAnalysis.cpp--------------------------------===//
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
// Define the VerilogModuleAnalysis pass, which is the container of the
// VASTModule.
//
//===----------------------------------------------------------------------===//

#include "llvm/CodeGen/MachineFunctionPass.h"


namespace llvm {
class VASTModule;
class VASTExprBuilder;

class VerilogModuleAnalysis : public MachineFunctionPass {
  VASTModule *Module;
public:
  static char ID;

  VerilogModuleAnalysis();

  bool runOnMachineFunction(MachineFunction &MF);
  void releaseMemory();

  VASTModule *createModule(const std::string &Name, VASTExprBuilder *Builder);
  VASTModule *getModule() const {
    assert(Module && "The module is not yet created!");
    return Module;
  }
};
}
