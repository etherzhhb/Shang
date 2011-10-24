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

#include "vtm/VFInfo.h"
#include "vtm/Passes.h"

#include "llvm/Module.h"
#include "llvm/DerivedTypes.h"
#include "llvm/Constants.h"
#include "llvm/Target/TargetData.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/SourceMgr.h"
#define DEBUG_TYPE "vtm-scripting-pass"
#include "llvm/Support/Debug.h"

using namespace llvm;

namespace {
struct ScriptingPass : public MachineFunctionPass {
  static char ID;
  std::string PassName, GlobalScript, FunctionScript;
  TargetData *TD;

  ScriptingPass(const char *Name, const char *FScript, const char *GScript)
    : MachineFunctionPass(ID), PassName(Name),
      GlobalScript(GScript), FunctionScript(FScript), TD(0) {}

  const char *getPassName() const { return PassName.c_str(); }

  bool doInitialization(Module &M);
  bool doFinalization(Module &M);

  bool runOnMachineFunction(MachineFunction &MF);
};
}

char ScriptingPass::ID = 0;

//Dirty Hack: Invoke the functions from the scripting library, which compile
// with rtti.
void bindToScriptEngine(const char *name, VASTModule *M);
bool runScriptFile(const std::string &ScriptPath, SMDiagnostic &Err);
bool runScriptStr(const std::string &ScriptStr, SMDiagnostic &Err);

static void ExtractConstant(raw_ostream &OS, Constant *C) {
  if (ConstantInt *CI = dyn_cast<ConstantInt>(C)) {
    const Type* Ty = CI->getType();
    if (Ty == Type::getInt1Ty(C->getContext()))
      OS << (CI->getZExtValue() ? '1' : '0');
    else
      OS << CI->getZExtValue();

    return;
  }

  errs() << "Unsupported constant type!";
  OS << '0';
}

static void CreateInitializerInfo(raw_ostream &OS, GlobalVariable *GV) {
  if (!GV->hasInitializer()) {
    OS << "nil";
    return;
  }

  Constant *C = GV->getInitializer();
  if (C->isNullValue()) {
    OS << "nil";
    return;
  }

  if (ConstantArray *CA = dyn_cast<ConstantArray>(C)) {
    OS << "{ ";
    ExtractConstant(OS, cast<Constant>(CA->getOperand(0)));
    for (unsigned i = 1, e = CA->getNumOperands(); i != e; ++i) {
      OS << ", ";
      ExtractConstant(OS, cast<Constant>(CA->getOperand(i)));
    }
    OS << "}";
    return;
  }

  OS << "nil";
}

bool ScriptingPass::doInitialization(Module &M) {
  TD = getAnalysisIfAvailable<TargetData>();
  assert(TD && "TD not avaialbe?");

  SMDiagnostic Err;
  // Put the global variable information to the script engine.
  if (!runScriptStr("GlobalVariables = {}\n", Err))
    llvm_unreachable("Cannot create globalvariable table in scripting pass!");

  std::string Script;
  raw_string_ostream SS(Script);
  // Push the global variable information into the script engine.
  for (Module::global_iterator GI = M.global_begin(), E = M.global_end();
       GI != E; ++GI ){

    GlobalVariable *GV = GI;
    // GlobalVariable information:
    // GVInfo {
    //   bool isLocal
    //   unsigned NumElems
    //   unsigned ElemSize
    //   table Initializer.
    //}

    SS << "GlobalVariables." << GV->getName() << " = { ";
    SS << "isLocal = " << GV->hasLocalLinkage() << ", ";

    SS << "NumElems = ";
    const Type *Ty = cast<PointerType>(GV->getType())->getElementType();
    // The element type of a scalar is the type of the scalar.
    const Type *ElemTy = Ty;

    if (const ArrayType *AT = dyn_cast<ArrayType>(Ty)) {
      ElemTy = AT->getElementType();
      SS << AT->getNumElements();
    } else
      SS << '1';
    SS << ", ";

    SS << "ElemSize = " << TD->getTypeStoreSizeInBits(ElemTy) << ", ";

    SS << "Initializer = ";
    CreateInitializerInfo(SS, GV);
    SS << '}';

    SS.flush();
    if (!runScriptStr(Script, Err))
      llvm_unreachable("Cannot create globalvariable infomation!");

    Script.clear();
  }

  // Run the script against the GlobalVariables table.
  if (!runScriptStr(GlobalScript, Err))
    report_fatal_error("In Scripting pass[" + PassName + "]:\n"
                       + Err.getMessage());


  return false;
}

bool ScriptingPass::doFinalization(Module &M) {
  return false;
}

bool ScriptingPass::runOnMachineFunction(MachineFunction &MF) {
  SMDiagnostic Err;
  // Push the function information into the script engine.
  // FuncInfo {
  //   String Name,
  //   unsinged ReturnSize,
  //   ArgTable : { ArgName = Size, ArgName = Size ... }
  // }

  const Function *F = MF.getFunction();

  std::string Script;
  raw_string_ostream SS(Script);

  SS << "FuncInfo = { ";
  SS << "Name = '" << F->getName() << "', ";

  SS << "ReturnSize = ";
  if (F->getReturnType()->isVoidTy())
    SS << '0';
  else
    SS << TD->getTypeStoreSizeInBits(F->getReturnType());
  SS << ", ";

  SS << "Args = { ";

  if (F->arg_size()) {
    Function::const_arg_iterator I = F->arg_begin();
    SS << "{ Name = '" << I->getName() << "', Size = "
       << TD->getTypeStoreSizeInBits(I->getType()) << "}";
    ++I;

    for (Function::const_arg_iterator E = F->arg_end(); I != E; ++I)
      SS << " , { Name = '" << I->getName() << "', Size = "
         << TD->getTypeStoreSizeInBits(I->getType()) << "}";
  }

  SS << "} }";

  SS.flush();
  if (!runScriptStr(Script, Err))
    llvm_unreachable("Cannot create function infomation!");
  Script.clear();

  bindToScriptEngine("CurModule", MF.getInfo<VFInfo>()->getRtlMod());

  if (!runScriptStr(FunctionScript, Err))
    report_fatal_error("In Scripting pass[" + PassName + "]:\n"
                       + Err.getMessage());

  return false;
}

Pass *llvm::createScriptingPass(const char *Name, const char *FScript,
                                const char *GScript) {
  return new ScriptingPass(Name, FScript, GScript);
}
