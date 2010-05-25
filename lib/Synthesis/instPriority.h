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
#include "llvm/Support/Streams.h"
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

#include "../utils.h"
#include "abstractHWOpcode.h"

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
            typedef map<Instruction*,unsigned int> InstPriorityMap; 

            /*
             *C'tor
             */
            instructionPriority(BasicBlock* BB);

            /** 
             * @brief return a topologically ordered instruction list
             * @return InstructionVector of instructions. First instruction should be
             * scheduled first.
             */
            InstructionVector getOrderedInstructions(); 
        private:
            /** 
             * @brief Returns the number of uses this inst has
             * in this basic block
             */
            unsigned int getLocalUses(Instruction* inst);

            /*
             * Return a list of all of the dependencies of instruction
             */
            set<Instruction*> getDependencies(Instruction* inst);

            /** 
             * @brief Calculate the dependencies between the
             * different opcodes. Give each one of the nodes a BFS
             * score.
             */
            void calculateDeps();

            /// Holds the depth of each of the elements in the graph
            InstPriorityMap m_depth; 

            /// hold the BasicBlock
            BasicBlock* BB;
    }; //class


} //end of namespace
#endif // h guard
