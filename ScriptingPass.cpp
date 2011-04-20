//===- ScriptingPass.cpp - Pass Applies Script to Pogram Data ---*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the ScriptingPass, a MachineFunctionPass that allow user
// manipulate some data such as current generated RTL module with external script
//
//===----------------------------------------------------------------------===//

#include "vtm/VerilogAST.h"
#include "vtm/VFInfo.h"
#include "vtm/Passes.h"

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/SourceMgr.h"
#define DEBUG_TYPE "vtm-scripting-pass"
#include "llvm/Support/Debug.h"


using namespace llvm;

namespace {
struct ScriptingPass : public MachineFunctionPass {
  static char ID;
  std::string PassName, PassScript;

  ScriptingPass(const char *Name, const char *Script)
    : MachineFunctionPass(ID), PassName(Name), PassScript(Script) {}

  const char *getPassName() const { return PassName.c_str(); }

  bool runOnMachineFunction(MachineFunction &MF);
};
}

char ScriptingPass::ID = 0;

//Dirty Hack: Invoke the functions from the scripting library, which compile
// with rtti.
void bindToScriptEngine(const char *name, VASTModule *M);
bool runScriptFile(const std::string &ScriptPath, SMDiagnostic &Err);

bool ScriptingPass::runOnMachineFunction(MachineFunction &MF) {
  bindToScriptEngine("CurModule", MF.getInfo<VFInfo>()->getRtlMod());
  SMDiagnostic Err;
  if (!runScriptFile(PassScript, Err))
    report_fatal_error("In Scripting pass[" + PassName + "]:\n"
                       + Err.getMessage());

  return false;
}

Pass *llvm::createScriptingPass(const char *Name, const char *Script) {
  return new ScriptingPass(Name, Script);
}
