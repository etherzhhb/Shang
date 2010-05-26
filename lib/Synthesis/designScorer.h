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
#ifndef LLVM_DESIGN_SCORER_H
#define LLVM_DESIGN_SCORER_H

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
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/Support/InstIterator.h"

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <set>

#include "listScheduler.h"

using namespace llvm;

namespace xVerilog {

    /*
     * @brief A class for evaluating the score of a scheduling job based on the frequency
     * and number of clocks.
     */
    class designScorer {
        public:
            /** 
             * @brief C'tor
             * 
             * @param design A listScheduler object who's design
             * we want to examine 
             */
            designScorer(LoopInfo* LInfo):m_loopInfo(LInfo){
                 // get the configuration of the units from the command line
                map<string, unsigned int> resourceMap = machineResourceConfig::getResourceTable();
                m_pointerSize = resourceMap["mem_wordsize"];
            };

            /** 
             * @brief Adds a list scheduler to the list of schedulers to evaluate
             * 
             * @param ls the scheduler to add
             */
            void addListScheduler(listScheduler* ls) {
                m_basicBlocks.push_back(ls);
            }   

            /** 
             * 
             * @return time in usec
             */
            unsigned int getDesignClocks();
            /** 
             * @brief Returns the max frequency that the design can stand
             * 
             * @return max frequenct
             */
            double getDesignFrequency();

            /** 
             * @returns number between zero and one, the percentage of all BasicBlocks which are
             * inside a loop.
             */
            double getLoopBlocksCount();

            /** 
             * @brief approximates the number of flip flops this circuit takes
             * 
             * @param ls the basic block to eval
             * 
             * @return 
             */
            unsigned int getDesignSizeInGates(Function* F);
        private:

            /** 
             * @brief Get the number of clocks it takes a BasicBlock to be executed (steps)
             * 
             * @param ls the listScheduler which scheduled the BB
             * 
             * @return clocks to complete the bb
             */
            unsigned int getBasicBlockClocks(listScheduler* ls) { return ls->length(); }

            /** 
             * @brief Evaluate the max delay of a single BasicBlock in usec
             * 
             * @param ls the block to evaluate
             * 
             * @return the time in usec to execute the longest instruction
             */
            double getBasicBlockMaxDelay(listScheduler* ls);

            /** 
             * @brief Returns the physical hardware execution delay time in ns
             * 
             * @param inst instruction to eval
             * 
             * @return time in ns
             */
            double getDelayForInstruction(Instruction *inst);

            /** 
             * @brief approximates the number of flip flops this instruction takes
             * 
             * @param inst , the instruction who's size we want to get
             * 
             * @return in bits (flip flops)
             */
            int getInstructionSize(Instruction* inst);
            /// a vector of schedulers to evaluate
            listSchedulerVector m_basicBlocks;
            LoopInfo* m_loopInfo;
            unsigned int m_pointerSize;
    };


} //end of namespace
#endif // h guard
