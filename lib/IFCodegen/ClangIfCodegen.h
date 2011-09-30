//===-- ClangIfCodegen.h - The base class for c interface code generation --==//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines the ClangIfCodegen pass, which is the common base of the
// interface code generation pass.
//
//===----------------------------------------------------------------------===//

#ifndef C_LANG_IF_CODEGEN
#define C_LANG_IF_CODEGEN

#include "vtm/LangSteam.h"

#include "llvm/Attributes.h"
#include "llvm/Pass.h"
#include "llvm/CodeGen/MachineFunctionPass.h"

namespace llvm {
class VASTModule;
class VFInfo;

class Value;
class Constant;
class ConstantArray;

class ClangIfCodegen : public MachineFunctionPass {
protected:
  lang_raw_ostream<CppTraits> Out;
  VASTModule *RTLMod;
  const VFInfo *FInfo;

  ClangIfCodegen(raw_ostream &O, char &ID)
    : MachineFunctionPass(ID), Out(O), RTLMod(0), FInfo(0) {}

  std::string GetValueName(const Value *Operand);

  raw_ostream &printSimpleType(raw_ostream &Out, const Type *Ty, bool isSigned,
                               const std::string &NameSoFar = "");
  raw_ostream &printType(raw_ostream &Out, const Type *Ty,
                         bool isSigned = false,
                         const std::string &NameSoFar = "",
                         bool IgnoreName = false,
                         const AttrListPtr &PAL = AttrListPtr());

  void printFunctionSignature(raw_ostream &Out, const Function *F);
  void printConstant(raw_ostream &Out, Constant *CPV, bool Static);
  void printConstantArray(raw_ostream &Out, ConstantArray *CPA,
                          bool Static);
};
}

#endif
