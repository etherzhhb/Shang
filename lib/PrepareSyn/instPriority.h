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
#ifndef LLVM_INST_PRIO_H
#define LLVM_INST_PRIO_H

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
#include <map>
#include <set>

#include "vbe/utils.h"

using namespace llvm;


namespace xVerilog {

    using std::vector;
    using std::map;

    /*
     * This class gives a scheduling priority for each one of the instructions
     * in a basic block based on a reasonable order. It gives
     * instructions which are in the critical pass the first priority. 
     */
    class instructionPriority {
        public:
            typedef map<const Instruction*,unsigned int> InstPriorityMap; 

            /*
             *C'tor
             */
            instructionPriority(const BasicBlock* BB);

            /** 
             * @brief Debug prints the content of the priority table
             * 
             * @return string with the table in ascii
             */
            string toString();

            /** 
             * @brief Return the hight of the dependenct tree. 
             * This is the highest value in the dependency map
             * @return max depth 
             */
            unsigned int getMaxDepth() const {return m_maxDepth;}

            /** 
             * @brief Return the priority of the instruction in the graph
             * @param inst The param to check
             * @return the height in the priority graph
             */
            unsigned int getPriority(const Instruction* inst) {return m_depth[inst];}
            /** 
             * @brief Returns the number of uses this inst has
             * in this basic block
             */
            static unsigned int getLocalUses(const Instruction* inst, const BasicBlock* BB);

            /** 
             * @brief Replace the first use in the use chain with a given version 
             * 
             * @param inst 
             * @param orig
             * @param cloned 
             */
            static void replaceFirstUseOfWith(Instruction* inst, Instruction* cloned);
        private:
            /** 
             * @brief Returns the latency in cycles of instruction. 
             *  For example, a MUL may take 5 cycles to complete, add
             *  may take 2.
             * 
             * @param inst the instruction we want to test 
             * 
             * @return Latency in cycles
             */
            unsigned int getLatencyForInstruction(const Instruction* inst) const;

            /*
             * Return a list of all of the dependencies of instruction
             */
            set<const Instruction*> getDependencies(const Instruction* inst);

            /** 
             * @brief Calculate the dependencies between the
             * different opcodes. Give each one of the nodes a BFS
             * score.
             */
            void calculateDeps();

            /// Holds the depth of each of the elements in the graph
            InstPriorityMap m_depth;
            // Holds the maximum depth of the InstPriorityMap
            unsigned int m_maxDepth;
            /// hold the BasicBlock
            const BasicBlock* BB;
    }; //class


} //end of namespace
#endif // h guard
