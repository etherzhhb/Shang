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
#ifndef LLVM_REDUCE_WORDWIDTH_H
#define LLVM_REDUCE_WORDWIDTH_H

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
    class ReduceWordWidthPass : public FunctionPass {

        public:
            /// needed by LLVM
            static char ID;
            /** 
             * @brief C'tor in LLVM style
             */
            ReduceWordWidthPass() : FunctionPass((intptr_t)&ID),  Context(0){}

            /** 
             * @brief Requires LoopInfo analysis and tells llvm that
             * we change nothing
             * 
             * @param AU llvm analysis usage internal object
             */
            virtual void getAnalysisUsage(AnalysisUsage &AU) const {
                AU.setPreservesCFG();
            }

            bool removeUnusedInstructions(Instruction* inst);
            bool reduceOperatorWordWidth(Instruction* inst);

            virtual bool runOnFunction(Function &F);

        private:
          LLVMContext *Context;
    }; // class



} //end of namespace
#endif // h guard
