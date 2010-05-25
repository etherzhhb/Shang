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
#ifndef LLVM_MSCHED_H
#define LLVM_MSCHED_H

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
#include "llvm/Analysis/LoopPass.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/Transforms/Utils/Local.h"
#include "llvm/Transforms/Scalar.h"

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
     * The awesome ModuloScheduler pass 
     */
    class ModuloSchedulerDriverPass : public LoopPass {

        public:
            /// needed by LLVM
            static char ID;
            /** 
             * @brief C'tor in LLVM style
             */
            ModuloSchedulerDriverPass() : LoopPass((intptr_t)&ID) {}

            /** 
             * @brief Requires LoopInfo analysis and tells llvm that
             * we change nothing
             * 
             * @param AU llvm analysis usage internal object
             */
            virtual void getAnalysisUsage(AnalysisUsage &AU) const {
                AU.addRequiredID(LoopSimplifyID);
                AU.addPreservedID(LoopSimplifyID);
                AU.addRequired<LoopInfo>();
            }

            void runModuloScheduler(Function *F);

            virtual bool runOnLoop(Loop *IncomingLoop, LPPassManager &LPM_Ref);

        private:
            /** 
             * @brief Checks if we are able to apply MS to this loop. 
             *  1. Is this loop only one BB
             *  2. Are we storing and loading only to different arrays?
             *  3. etc (see code)
             * 
             * @param loop The loop to inspect
             * 
             * @return True if we can apply the transformation
             */
            bool  loop_is_ms_able(Loop* loop);
            /** 
             * @brief Copy the tree of instructions above the current instructions into the
             * Preheader. This will create the preheader for the MS. Each of the subscripts
             * will be adjusted by the 'offset' parameter. The instruction is also copied.
             * Store instructions are not copied. On phi nodes, they are replaced with the value 
             * thats coming from the preheader. 
             * 
             * @param inst the instruction whos 
             * @param induction the induction variable of the loop (the 'i')
             * @param header The 'preheader' of the loop
             * @param offset the offset to add to the subscriptions
             * @return the new instruction inside the preheader
             */
            Value* copyLoopBodyToHeader(Instruction* inst, Instruction* induction, BasicBlock* header, int offset);
            /** 
             * @brief Increments the index as used by MS. It creates a new constant 
             * and an 'add' instruction and replaces the 'i' reference with the new command.  
             * Adds a new instruction before 'inst'.
             * for example: A[i] -> A[i+offset]. 
             *
             * @param inst The instruction to modify.
             * @param ind the induction variable
             * @param offset the offset - how much to increment. 
             */
            void incrementInductionVarIfUsed(Instruction* inst, Instruction* ind, int offset);
            /** 
             * @brief clone all values in this BB which have a use>1. This is done so that 
             * each instruction will depend on values which are exactly one level above it. 
             * In the case where two instructions of different depth (in the DAG) depend on a third
             * instruction, we can't change the indexing according to the modulo scheduling.
             * We only duplicate instructions within this BB. 
             * 
             * @param bb the BasicBlock to modify
             * @param ind The induction variable 
             */
            void duplicateValuesWithMultipleUses(BasicBlock* bb, Instruction* ind);
            /** 
             * @brief Return a vector of all of the arrays
             * 
             * For example, This code will return the vector[%array1]:
             *   %ptr10 = getelementptr i32* %array1, i32 %i   ; <i32*> [#uses=1]
             *   %tmp41 = load i32* %ptr10, align 4            ; <i32>  [#uses=1]
             *
             * @param bb The basic block to search. 
             * @param store Look for StoreInst commands or LoadInst commands. 
             *  True if Store, False if Load.
             * 
             * @return the std vector with the values.  
             */
            std::vector<Value*> getAllArrayAccess(BasicBlock* bb, bool store);
            /** 
             * @brief After MS we often have multiple LoadInst from the same address
             * in the array. This pass removes them.
             * 
             * @param bb The BB to scan (This is the body of the loop).
             */
            void eliminateDuplicatedLoads(BasicBlock* bb);
            /** 
             * @brief Return True if the operand of both instructions identical.
             *  (Both take from the same values)
             *   Implemented:PHINode, GetElementPTR and Load
             * 
             * @param i1
             * @param i2
             * 
             * @return True if identical 
             */
            bool areInstructionsIdentical(Instruction* i1, Instruction* i2);
            /** 
             * @brief If this instruction is a BinaryOperand of type ADD
             * and can be optimized to one of the forms:
             *   = x + 0
             *   = const + const
             *  it replaces all values with the optimized add
             * 
             * @param add The instruction to work on. If this is not an ADD inst, do nothing.
             */
            void foldAddInstructions(Instruction* add);
    }; // class



} //end of namespace
#endif // h guard
