//===- FunctionFilter.cpp - This Pass filter out the SW part of the module -===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This pass perform the software/hardware sysinfo by simply move the SW part
// to another llvm module.
//
//===----------------------------------------------------------------------===//

#include "vtm/Passes.h"
#include "vtm/SystemInfo.h"

#include "llvm/Pass.h"
#include "llvm/Module.h"
#include "llvm/Transforms/Utils/Cloning.h"
#include "llvm/Bitcode/ReaderWriter.h"
#include "llvm/Assembly/AssemblyAnnotationWriter.h"

#include "llvm/ADT/OwningPtr.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

namespace {
struct FunctionFilter : public ModulePass {
  static char ID;
  const SystemInfo &Partition;
  // The output stream for software part.
  raw_ostream &SwOut;

  FunctionFilter(raw_ostream &O)
    : ModulePass(ID), Partition(sysinfo()), SwOut(O) {}


  bool runOnModule(Module &M);
};

} // end anonymous.

bool FunctionFilter::runOnModule(Module &M) {
  // No need to perform any sysinfo.
  if (Partition.empty()) return false;
  
  OwningPtr<Module> SoftMod(CloneModule(&M));
  SoftMod->setModuleIdentifier(M.getModuleIdentifier() + ".sw");

  // Remove software functions in current module.
  std::vector<Function*> Functions;
  for (Module::iterator I = M.begin(), E = M.end(); I != E; ++I) {
    Function &F = *I;
    if (!Partition.isHardware(F)) {
      F.dropAllReferences();
      F.getBasicBlockList().clear();
    }
  }

  // Remove hardware functions in software module and leave the declaretion only.
  for (Module::iterator I = SoftMod->begin(), E = SoftMod->end(); I != E; ++I) {
    Function &F = *I;
    if (Partition.isHardware(F)) {
      F.dropAllReferences();
      F.getBasicBlockList().clear();
    }
  }

  // TODO: We may rename the entry function, too.
  OwningPtr<AssemblyAnnotationWriter> Annotator;

  SoftMod->print(SwOut, Annotator.get());

  return true;
}

char FunctionFilter::ID = 0;

Pass *llvm::createFunctionFilterPass(raw_ostream &O) {
  return new FunctionFilter(O);
}
