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
#ifndef LLVM_GLOBAL_VAR_REG_H
#define LLVM_GLOBAL_VAR_REG_H

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
#include <vector>
#include <set>
#include <map>

using namespace llvm;

using std::map;
using std::set;
using std::vector;
using std::pair;
using std::string;

namespace xVerilog {
    /*
     * This class holds a mapping between the names of the global variables and the
     * pointer to the actual data. This allows control of all of the global variables
     * in a single location. 
     */
    class globalVarRegistry {

        public:
            void init(Module *M) {m_module  = M; }

            void destroy();

            /*
             * add this variable to a list of instructions to be destructed on exit
             */
            void trashWhenDone(Instruction* val) { m_garbage.push_back(val); }

            /*
             * gets or creates a global variable which will be destructed on exit
             * @params number of bits, it this a pointer, default value, name
             */
            GlobalVariable* getGlobalVariableByName(string varName, 
                    int bits = 32, bool pointer = false ,int val = 0);

            GlobalVariable* getGlobalVariableByName(string varName, const Type* type);

            /*
             * Creates a type of the needed bit number. For example Int64Ty
             */
            static const Type* bitNumToType(int bitnum);

            // some default values to use
            static Value *Zero1;
            static Value *One1;
            static Value *Zero32;
            static Value *One32;

        private:
            static Module* m_module;
            static map<string, GlobalVariable*> m_map;
            static vector<Instruction*> m_garbage;
    };


} //end of namespace
#endif // h guard
