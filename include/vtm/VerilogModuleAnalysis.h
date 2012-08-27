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
//
//
//
//===----------------------------------------------------------------------===//
#include "vtm/Passes.h"

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "VerilogAST.h"

namespace llvm {
class VerilogModuleAnalysis : public MachineFunctionPass {
  VASTModule *VMPtr;
public:
  static char ID;

  VerilogModuleAnalysis();

  bool runOnMachineFunction(MachineFunction &MF);
};
}
