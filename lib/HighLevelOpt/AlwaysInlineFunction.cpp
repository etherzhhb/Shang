//===------------------ AlwaysInlineFunction.cpp --------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
// This passs adds "AlwaysInline" attribute to the functions those don't have 
// "NoInline" attribute. And then inline the functions.
// 
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "AlwaysInlineFunction"

#include "vtm/Passes.h"
#include "llvm/CallingConv.h"
#include "llvm/Instructions.h"
#include "llvm/IntrinsicInst.h"
#include "llvm/Module.h"
#include "llvm/Type.h"
#include "llvm/Analysis/CallGraph.h"
#include "llvm/Analysis/InlineCost.h"
#include "llvm/Support/CallSite.h"
#include "llvm/Transforms/IPO.h"
#include "llvm/Transforms/IPO/InlinerPass.h"
#include "llvm/ADT/SmallPtrSet.h"

using namespace llvm;

namespace {

  // AlwaysInliner only inlines functions that are mark as "always inline".
  class AlwaysInlineFunction : public Inliner {
    // Functions that are never inlined
    SmallPtrSet<const Function*, 16> NeverInline; 
    InlineCostAnalyzer CA;
  public:
    // Use extremely low threshold. 
    AlwaysInlineFunction() : Inliner(ID, -2000000000) {
      initializeAlwaysInlineFunctionPass(*PassRegistry::getPassRegistry());
    }
    static char ID; // Pass identification, replacement for typeid
    InlineCost getInlineCost(CallSite CS) {
      return CA.getInlineCost(CS, NeverInline);
    }
    float getInlineFudgeFactor(CallSite CS) {
      return CA.getInlineFudgeFactor(CS);
    }
    void resetCachedCostInfo(Function *Caller) {
      CA.resetCachedCostInfo(Caller);
    }
    void growCachedCostInfo(Function* Caller, Function* Callee) {
      CA.growCachedCostInfo(Caller, Callee);
    }
    virtual bool doFinalization(CallGraph &CG) { 
      return removeDeadFunctions(CG, &NeverInline); 
    }
    virtual bool doInitialization(CallGraph &CG);
    void releaseMemory() {
      CA.clear();
    }
  };
}

char AlwaysInlineFunction::ID = 0;
INITIALIZE_PASS_BEGIN(AlwaysInlineFunction, "always-inline",
                "Inliner for always_inline functions", false, false)
INITIALIZE_AG_DEPENDENCY(CallGraph)
INITIALIZE_PASS_END(AlwaysInlineFunction, "always-inline",
                "Inliner for always_inline functions", false, false)

Pass *llvm::createAlwaysInlineFunctionPass() 
{ 
  return new AlwaysInlineFunction(); 
}

// doInitialization - Initializes the vector of functions that have not 
// been annotated with the "always inline" attribute.
bool AlwaysInlineFunction::doInitialization(CallGraph &CG) {
  Module &M = CG.getModule();
  bool Flag = false;
  
  for (Module::iterator I = M.begin(), E = M.end();
       I != E; ++I)
  {
    if (!I->hasFnAttr(Attribute::NoInline))
    {
      I->addAttribute(~0, Attribute::AlwaysInline);
      Flag = true;
    }
    if (!I->isDeclaration() && !I->hasFnAttr(Attribute::AlwaysInline))
      NeverInline.insert(I);
  }
  return Flag;
}
