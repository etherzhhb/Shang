//=====-HWPartitionInfo.h - This Pass prints out all the Function names.-=====//
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

#ifndef PRINT_FUNCTION_NAME
#define PRINT_FUNCTION_NAME

#include "llvm/Pass.h"
#include "llvm/ADT/SmallPtrSet.h"

namespace llvm{
  struct HWPartitionInfo : public ModulePass{

    static char ID;

    SmallPtrSet<const Function*, 32> HWFunctions;

    HWPartitionInfo();

    virtual void getAnalysisUsage(AnalysisUsage &AU) const;

    bool runOnModule(Module &M);

    bool isHW(Function *F);

    virtual void destroy() {
      HWFunctions.clear();
    }

  };//end struct

}//end anonymous namespace

#endif