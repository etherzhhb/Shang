//===-PrintFunctionName.cpp - This Pass prints out all the Function names.-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This pass simply prints out all the Function names.
//
//===----------------------------------------------------------------------===//

#include "vtm/Passes.h"

#include "vtm/HWPartitionInfo.h"
#include "llvm/Pass.h"
#include "llvm/Module.h"
#include "llvm/Function.h"

#define DEBUG_TYPE "function-name"

#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Transforms/IPO/InlinerPass.h"
#include "llvm/Transforms/IPO.h"
#include "vtm/SynSettings.h"
#include "llvm/ADT/DepthFirstIterator.h"
#include "llvm/Analysis/CallGraph.h"

using namespace llvm;

INITIALIZE_PASS_BEGIN(HWPartitionInfo, "HWPartitionInfo",
  "HW Partition Information", true, true)
  INITIALIZE_AG_DEPENDENCY(CallGraph)
INITIALIZE_PASS_END(HWPartitionInfo, "HWPartitionInfo",
"HW Partition Information", true, true)
char HWPartitionInfo::ID = 0;

Pass *llvm::createHWPartitionInfoPass(){
  return new HWPartitionInfo();
}



HWPartitionInfo::HWPartitionInfo() : ModulePass(ID){
  initializeHWPartitionInfoPass(*PassRegistry::getPassRegistry());
}

void HWPartitionInfo::getAnalysisUsage(AnalysisUsage &AU) const {
  // ModulePass::getAnalysisUsage(AU);
  AU.addRequired<CallGraph>();
  AU.setPreservesAll();
}

bool HWPartitionInfo::runOnModule(Module &M){

  releaseMemory();
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
        // Create the synthesis setting for subfunctions.
        if (!SubF || SubF->isDeclaration())
          continue;
        if (SubF != F)
          getSynSetting(SubF->getName(), TopSetting)->setTopLevelModule(false);
        HWFunctions.insert(SubF);
      }
    }
  }

  return false;
}

bool HWPartitionInfo::isHW(Function *F) {
  if (HWFunctions.count(F))
    return true;
  else 
    return false;
}





