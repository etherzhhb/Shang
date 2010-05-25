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
#ifndef LLVM_SUBSCRIPTS_HELPER_H
#define LLVM_SUBSCRIPTS_HELPER_H

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
    class subscripts {
        public:
            /** 
             * @brief C'tor, detects all load/stores to arrays
             * which are inside a loop
             */
            subscripts(Loop* myLoop) {
                    detectInductionVariables(myLoop);
                    detectSubscripts(myLoop);
                }

            subscripts(BasicBlock *bb, Instruction* inductionVar) {
                m_inductionVariables.insert(inductionVar);
                detectSubscripts(bb);
            }
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
             * @brief Turn all A[i] subscripts to the form of A[i+0] form. 
             * This will allow us to easily change it to A[i+c] for the modulo
             * scheduler
             */
            void normalizeSubscripts();

            /** 
             * @brief If this is a subscript (array access) then increment 
             * the offset of the index by 'offset'. Ex: A[i] becomes A[i+3] 
             * 
             * @param inst the possible induction variable
             * @param offset the offset to increment by. 
             */
            void incrementIfSubscript(Value* inst, int offset);
            /** 
             * @brief Increment the index of all subscripts (A[i+2] -> A[i+2+offset])
             * 
             * @param offset 
             */
            void incrementAllSubscripts(int offset);
            /** 
             * @brief return the induction variable
             */
            Instruction* getInductionVar() {
                if (Instruction *ind = dyn_cast<Instruction>(*m_inductionVariables.begin())) return ind;
                assert(0 && "No instruction variable");
                return NULL;
            }

            /** 
             * @brief A helper method for incrementing Values. This creates
             * a new value. Only works for integers. It does not insert the command
             * to anywhere.
             * 
             * @param val The value to change Ex: A
             * @param offset how much to increment. Ex: 5
             * @param inst if this is non null, insert the instruction here
             * @param return new result. Ex: A+5
             */
            static Instruction* incrementValue(Value* val, int offset, Instruction* insert=0);

            /** 
             * @brief Returns true if the induction variable directly depends on this instruction.
             * For example in the case of (i := i+1)
             * 
             * @param inst the instruction to test
             * 
             * @return true if IV directly depends on inst
             */
            bool isUsedByInductionVariable(Instruction* inst);

        private:
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
            void addOffsetToGetElementPtr(Value *inst, int offset=0);
            /** 
             * @brief Scans the loop and finds all of the Load/Store
             * to array inside a loop
             */
            void detectSubscripts(Loop* loop);
            /** 
             * @brief Scans the BB and finds all of the Load/Store
             * to array inside a loop
             */
            void detectSubscripts(BasicBlock* bb);
            /** 
             * @brief This will increment the index of the array access. 
             *  For example, It will turn A[i+3] to A[i+5] (for offset = 2).
             * 
             * @param inst the LoadInst/StoreInst of the subscript
             * @param offset how much to increment/decrement
             */
            void incrementAccessIndex(Value* inst, int offset); 
            /*
             * @brief Detect all induction variables in this loop and insert them to the set
             */
            void detectInductionVariables(Loop* loop);
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
