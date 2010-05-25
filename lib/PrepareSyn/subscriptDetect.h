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
#ifndef LLVM_ARRAY_DETECTION_PASS_H
#define LLVM_ARRAY_DETECTION_PASS_H

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
#include "llvm/Analysis/LoopInfo.h"
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
     * A pass for detecting different array accesses in the code 
     */
    class subscriptDetect: public FunctionPass {

        public:
            /// needed by LLVM
            static char ID;
            /** 
             * @brief C'tor in LLVM style
             */
            subscriptDetect() : FunctionPass((intptr_t)&ID) {}

            /** 
             * @brief Requires LoopInfo analysis and tells llvm that
             * we change nothing
             * 
             * @param AU llvm analysis usage internal object
             */
            virtual void getAnalysisUsage(AnalysisUsage &AU) const {
                AU.addRequired<LoopInfo>();
                AU.addPreserved<LoopInfo>();
                AU.setPreservesAll();
            }

            virtual bool runOnFunction(Function &F);

            /** 
             * @brief finds all array references in the function 
             * 
             * @param BB 
             * 
             * @return vector of instructions which load the pointer to the array
             */

            static vector<Instruction*> findAllArrayReferences(Function *F);

        private:
            /** 
             * @brief This will increment the index of the array access. 
             *  For example, It will turn A[i+3] to A[i+5] (for offset = 2).
             * 
             * @param inst the LoadInst/StoreInst of the subscript
             * @param offset how much to increment/decrement
             */
            void incrementAccessIndex(Value* inst, int offset); 
            /** 
             * @brief Tests if this value is an induction variable
             *
             * @param inst The param we want to check.
             * 
             * @return True if this is an induction variable
             */
            bool isInductionVariable(Value *inst);
            /** 
             * @brief Tests if this value is an index which depends on an induction variable
             *  using a binary operand. For example, [i + 2] will return true because i is indvar and
             *  we modify it with the binary operation add.
             *
             * @param inst The param we want to check.
             * 
             * @return True if this is indeed a modified induction variable.
             */
            bool isModifiedInductionVariable(Value *inst);
            /** 
             * @brief Tests if this value is a 'getelementptr' instruction which returns
             * an address which is obtained via induction pointer.
             * 
             * @param inst The value we want to evaluate 
             * 
             * @return pointer to subscript variable if this instruction 
             *  returns the address of a getelementptr. Else, NULL.
             */
            Value* isAddressCalculation(Value *inst);
            /** 
             * @brief Tests if this LoadInst loads a value from an array. 
             * 
             * @param inst The param we want to check.
             * 
             * @return pointer to subscript variable  if this is a subscript, Else NULL.
             */
            Value* isLoadSubscript(Value *inst);
            /** 
             * @brief Tests if this StoreInst stores a value to an array. 
             * 
             * @param inst The param we want to check.
             * 
             * @return pointer to subscript variable if this is a subscript, Else, NULL.
             */
            Value* isStoreSubscript(Value *inst);
            /** 
             * @brief Adds an 'Add' Instruction to the getElementPtr node
             *  so that it accesses the same expression as before, just 
             *  with an offset. This modifies the node and replaces the
             *  index parameter with an Add node that uses the previous
             *  expression plus a constant which reflects the offset. 
             * 
             * @param inst The GetElementPtrInst we want to modify
             * @param offset The offset we want to add. May be negative.
             */
            void addOffsetToGetElementPtr(Value *inst, int offset);


            /// A storage of the local instruction variables
            // It hold phi nodes of induction variables (i in for i loops)
            set<Value*> m_inductionVariables;

            /// At the end of this pass, pointers to arrays are stored
            // in this set
            set<Value*> m_LoadSubscripts;
            set<Value*> m_StoreSubscripts;

    }; // class



} //end of namespace
#endif // h guard
