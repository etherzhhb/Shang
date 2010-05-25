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
#ifndef LLVM_PARALLEL_PASS_H
#define LLVM_PARALLEL_PASS_H

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
    class ParallelPass : public FunctionPass {

        public:
            /// needed by LLVM
            static char ID;
            /** 
             * @brief C'tor in LLVM style
             */
            ParallelPass() : FunctionPass((intptr_t)&ID) {}

            /** 
             * @brief Requires LoopInfo analysis and tells llvm that
             * we change nothing
             * 
             * @param AU llvm analysis usage internal object
             */
            virtual void getAnalysisUsage(AnalysisUsage &AU) const {
                AU.setPreservesCFG();
            }

            virtual bool runOnFunction(Function &F);

        public:
            /*
             * Drives the optimizations described below.
             */
            static void floodParallel(BasicBlock &BB, llvm::Instruction::BinaryOps binType);
            /*
             * This pass rearranges expression trees into a more balanced expression tree. It 
             * works on trees of the same type  (Add, Mul, etc). 
             */
            static bool floodParallelCommutative(Instruction *inst, llvm::Instruction::BinaryOps binType);

            /** 
             * @brief Is this instruction likely to be a 'fat' instruction. 
             * For example, an instruction which has many bits.
             * 
             * @param inst The instruction to test.
             * 
             * @return True if is probably is fat. 
             */
            static bool isLikelyToBeFatNode(Instruction* inst);
        private:
            /** 
             * @param inst  Instruction whos children we want to check
             * @param binType type to check. Example. Add, Mul, etc. 
             * @return True of all users of this opcode are of other types (not same binary opcode type)
             */
            static bool areAllUsersOfOtherType(Instruction* inst, llvm::Instruction::BinaryOps binType);
            /*
             * Find all children of an arithmetic expression tree. Put them in two lists.
             * One is the nodes of the expression tree, which is of type 'type' (for example ADD).
             * The Leafs of the tree (the data) goes into a second list. This adds only
             * nodes with use count of one. It adds the leafs and nodes in the order of discovery.
             */
            static void findAllChildrenOfSameType(Instruction::BinaryOps type, 
                    Value* inst, vector<Instruction*> *nodes, vector<Value*> *leafs);
    }; // class



} //end of namespace
#endif // h guard
