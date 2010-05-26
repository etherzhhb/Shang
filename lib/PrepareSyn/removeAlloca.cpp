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
#include "removeAlloca.h" 

#include "llvm/Transforms/Utils/Cloning.h"
#include "llvm/Pass.h"
#include "llvm/Argument.h"
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

#include <algorithm>
#include <sstream>
#include "vbe/utils.h"

namespace xVerilog {

    Function* MyCloneFunction(const Function *F, DenseMap<const Value*, Value*> &ValueMap, const Type* addedArg, std::string name) {
        std::vector<const Type*> ArgTypes;

        // The user might be deleting arguments to the function by specifying them in
        // the ValueMap.  If so, we need to not add the arguments to the arg ty vector
        //
        for (Function::const_arg_iterator I = F->arg_begin(), E = F->arg_end(); I != E; ++I)
            if (ValueMap.count(I) == 0)  // Haven't mapped the argument to anything yet?
                ArgTypes.push_back(I->getType());

        ArgTypes.push_back(addedArg);

        // Create a new function type...
        FunctionType *FTy = FunctionType::get(F->getFunctionType()->getReturnType(), ArgTypes, F->getFunctionType()->isVarArg());

        // Create the new function...
        Function *NewF = Function::Create(FTy, F->getLinkage(), F->getName());

        // Loop over the arguments, copying the names of the mapped arguments over...
        Function::arg_iterator DestI = NewF->arg_begin();
        for (Function::const_arg_iterator I = F->arg_begin(), E = F->arg_end(); I != E; ++I)
            if (ValueMap.count(I) == 0) {   // Is this argument preserved?
                DestI->setName(I->getName()); // Copy the name over...
                ValueMap[I] = DestI++;        // Add mapping to ValueMap
            }

        (--NewF->arg_end())->setName(name);


        SmallVector<ReturnInst*, 4> Returns;  // Ignore returns cloned...
        CloneFunctionInto(NewF, F, ValueMap, Returns, "", 0);
        return NewF;
    }

    bool removeOneAlloca(Module& M) {

        for (Module::iterator Fit=M.begin(); Fit!=M.end(); ++Fit) {
            for (Function::iterator bbit = Fit->begin(); bbit != Fit->end(); ++bbit) {
                for (BasicBlock::iterator it = bbit->begin(); it != bbit->end(); ++it) {
                    if (AllocaInst *alloca = dyn_cast<AllocaInst>(it)) {

                        DenseMap<const Value*, Value*> my_map;
                        const Type* tp = (alloca)->getType(); 
                        std::string name = (alloca)->getName();
                        Function* NewFunc = MyCloneFunction(Fit, my_map, tp, name);

                        Instruction* newalloca = dynamic_cast<Instruction*>(my_map[alloca]); // The alloca in the new function
                        Argument* arg = --NewFunc->arg_end();

                        newalloca->replaceAllUsesWith(arg);
                        newalloca->removeFromParent();
                        Fit->getParent()->getFunctionList().push_back(NewFunc);
                        Fit->removeFromParent();
                        return true;

                    }
                }
            }
        }

        // did not remove anything. We are done.
        return false;
    }



    bool RemoveAllocaPass::runOnModule(Module &M) {
        bool changed = false;
        while (removeOneAlloca(M)) {
            changed = true;
        } 

        return changed;
    }

    char RemoveAllocaPass::ID = 0;
    RegisterPass<RemoveAllocaPass> RAX("remove_alloca", "remove allocas");

} //namespace


