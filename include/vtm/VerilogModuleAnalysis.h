//===------------------------VerilogModuleAnalysis.cpp--------------------------------===//
//
//                      The Shang HLS frameowrk                               //
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
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
