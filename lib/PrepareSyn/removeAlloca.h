/*
* Copyright: 2008 by Nadav Rotem. all rights reserved.
* IMPORTANT: This software is supplied to you by Nadav Rotem in consideration
* of your agreement to the following terms, and your use, installation, 
* modification or redistribution of this software constitutes acceptance
* of these terms.  If you do not agree with these terms, please do not use, 
* install, modify or redistribute this software. You may not redistribute, 
* install copy or modify this software without written permission from 
* Nadav Rotem. 
*/
#ifndef REMOVE_ALLOCA_PASS_H
#define REMOVE_ALLOCA_PASS_H

#include "llvm/Pass.h"
#include "llvm/Function.h"
#include "llvm/Instructions.h"
#include "llvm/Constants.h"
#include "llvm/ADT/StringExtras.h"

#include "llvm/Support/FormattedStream.h"
#include "llvm/Module.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/Support/CFG.h"
#include "llvm/DerivedTypes.h"

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <set>
#include <map>

using namespace llvm;

using std::set;
using std::map;
using std::vector;
using std::pair;
using std::string;

namespace xVerilog {

    /* 
     */
    class RemoveAllocaPass : public ModulePass {

        public:
            /// needed by LLVM
            static char ID;
            /** 
             * @brief C'tor in LLVM style
             */
            RemoveAllocaPass() : ModulePass((intptr_t)&ID) {}

            /** 
             * @param AU llvm analysis usage internal object
             */
            virtual void getAnalysisUsage(AnalysisUsage &AU) const {
                //AU.setPreservesCFG();
            }

            virtual bool runOnModule(Module &M);
    }; // class



} //end of namespace
#endif // h guard
