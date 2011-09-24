//=- FunctionFilter.cpp --- This Pass filter out the SW part of the module -==//
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

#define DEBUG_TYPE "function-filter"

#include "vtm/Passes.h"
#include "vtm/SynSettings.h"

#include "llvm/Pass.h"
#include "llvm/Module.h"
#include "llvm/Transforms/Utils/Cloning.h"
#include "llvm/Bitcode/ReaderWriter.h"
#include "llvm/Assembly/AssemblyAnnotationWriter.h"

#include "llvm/Support/CallSite.h"
#include "llvm/ADT/OwningPtr.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/Casting.h"
#include "llvm/IntrinsicInst.h"
#include "llvm/ADT/SmallPtrSet.h"
#include "llvm/Analysis/CallGraph.h"
#include "llvm/ADT/DepthFirstIterator.h"
#include <utility>
#include "../Schedule/VSUnit.h"
#include "llvm/Support/Debug.h"
#include "vtm/Utilities.h"

using namespace llvm;

namespace {
struct FunctionFilter : public ModulePass {
  static char ID;
  // The output stream for software part.
  raw_ostream &SwOut;

  FunctionFilter(): ModulePass(ID), SwOut(nulls()) {
    initializeFunctionFilterPass(*PassRegistry::getPassRegistry());
  }
  FunctionFilter(raw_ostream &O): ModulePass(ID), SwOut(O) {
    initializeFunctionFilterPass(*PassRegistry::getPassRegistry());
  }

  void getAnalysisUsage(AnalysisUsage &AU) const {
    AU.addRequired<CallGraph>();
    ModulePass::getAnalysisUsage(AU);
  }

  bool runOnModule(Module &M);
};
} // end anonymous.

bool FunctionFilter::runOnModule(Module &M) {
  //Mangle the name of globalvariables so we can refer them in C source files.
  for (Module::global_iterator I = M.global_begin(), E = M.global_end();
       I != E; ++I){
    GlobalVariable *GV = I;
    GV->setName(VBEMangle(GV->getNameStr()));
  }

  OwningPtr<Module> SoftMod(CloneModule(&M));
  SoftMod->setModuleIdentifier(M.getModuleIdentifier() + ".sw");

  SmallPtrSet<const Function*, 32> HWFunctions;
  CallGraph &CG = getAnalysis<CallGraph>();
  for (CallGraph::iterator ICG = CG.begin(), ECG = CG.end(); ICG != ECG; 
       ++ICG){
    // const Function *F = ICG->first;  
    CallGraphNode *CGN = ICG->second;
    Function *F = CGN->getFunction();
    if (!F || F->isDeclaration())
      continue;
    DEBUG(dbgs() << "*************Function name: " << F->getName() << "\n");
    if (SynSettings *TopSetting = getSynSetting(F->getName())){
      HWFunctions.insert(F);
      for (df_iterator<CallGraphNode*> ICGN = df_begin(CGN),
           ECGN = df_end(CGN); ICGN != ECGN; ++ICGN){
        const CallGraphNode *SubCGN = *ICGN;
        Function *SubF = SubCGN->getFunction();
        if (!SubF || SubF->isDeclaration())
          continue;
        // Create the synthesis setting for subfunctions.
        if (SubF != F)
          getSynSetting(SubF->getName(), TopSetting)->setTopLevelModule(false);

        HWFunctions.insert(SubF);
      }
    }
  }

  for (Module::iterator IHW = M.begin(), ISW = SoftMod->begin(), 
       EHW = M.end(), E = SoftMod->end(); IHW != EHW; ++IHW, ++ISW) {
    Function *FHW = IHW;
    Function *FSW = ISW;

    // The function is s software function, delete it from the hardware module.
    if (!HWFunctions.count(FHW))
      FHW->deleteBody();
    else if (getSynSetting(FSW->getName())->isTopLevelModule())
      // Remove hardware functions in software module and leave the declaretion 
      // only.
      FSW->deleteBody();
  }

  std::vector<GlobalVariable*> GVs;
  for (Module::global_iterator I = M.global_begin(), E = M.global_end();
       I != E; ++I) {
    I->setLinkage(GlobalValue::ExternalLinkage);
    I->setInitializer(0);
  }

  // Dirty Hack: Make all global variable in software side visiable in hardware
  // side.
  // FIXME: Move them to Hardware module.
  for (Module::global_iterator I = SoftMod->global_begin(),
       E = SoftMod->global_end(); I != E; ++I)
    I->setLinkage(GlobalVariable::LinkOnceAnyLinkage);

  // TODO: We may rename the entry function, too.
  OwningPtr<AssemblyAnnotationWriter> Annotator;
  SoftMod->print(SwOut, Annotator.get());

  return true;
}

char FunctionFilter::ID = 0;

INITIALIZE_PASS_BEGIN(FunctionFilter, "FunctionFilter", 
                      "Function Filter", false, false)
  INITIALIZE_AG_DEPENDENCY(CallGraph)
INITIALIZE_PASS_END(FunctionFilter, "FunctionFilter", 
                    "Function Filter", false, false)

Pass *llvm::createFunctionFilterPass(raw_ostream &O) {
  return new FunctionFilter(O);
}
