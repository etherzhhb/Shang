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
#include "GVRegistry.h"

using namespace xVerilog;

void globalVarRegistry::destroy() {
  // destroy all variables that should be destroied
  // We do that so that we don't have dead values with users
  // flying around

  // destroy all keys in hash table (all global variables)
  for( map<string, GlobalVariable*>::iterator I = m_map.begin();
      I != m_map.end(); ++I ) {
    // Replace all used of this variable with a null pointer
    I->second->replaceAllUsesWith(ConstantPointerNull::get(I->second->getType()));
    // Remove this global variable from its parent module
    I->second->removeFromParent();
    // Delete it
    delete I->second;
  }

  // For each load instruction that we have modified
  while(!CreateInsts.empty()) {
    Instruction *Inst = CreateInsts.front();
    // Replace the users of this dummy instruction with zero;
    if (Inst->hasNUsesOrMore(1))
      Inst->replaceAllUsesWith(ConstantInt::get(Inst->getType(),0));
    delete Inst;
  }
  GVRAllocator.Reset();
}

    const Type* globalVarRegistry::bitNumToType(int bitnum){
        if (bitnum==64) return IntegerType::get(*Context, 64);
        if (bitnum==32) return IntegerType::get(*Context, 32);
        if (bitnum==16) return IntegerType::get(*Context, 16);
        if (bitnum==8) return IntegerType::get(*Context, 8);
        if (bitnum==1) return IntegerType::get(*Context, 1);
        errs()<<"Unsupported bit addressing mode; "<<bitnum<<"\n";
        abort();
    }

    GlobalVariable* globalVarRegistry::getGVByName(string varName, 
            int bits, bool pointer ,int val) {
        if (m_map[varName] != NULL) return m_map[varName];
        GlobalVariable *glob;  
        if (pointer) {
            glob = new GlobalVariable(llvm::PointerType::get((bitNumToType(bits)),0),false,
                    GlobalValue::ExternalLinkage,0,varName,m_module);
            //glob = new GlobalVariable(llvm::PointerType::get((bitNumToType(bits))),false,
            //        GlobalValue::ExternalLinkage,0,varName,m_module);
        } else { /* int */
            const Type *t = bitNumToType(bits); 
            glob = new GlobalVariable(t, false,
                    GlobalValue::ExternalLinkage,0,varName,m_module);
        }

        m_map[varName] = glob;
        return glob;
    } 

    GlobalVariable* globalVarRegistry::getGVByName(string varName, const Type* type) {
        if (m_map[varName] != NULL) { 
                return m_map[varName];
        }
        GlobalVariable *glob = new GlobalVariable(type, false, GlobalValue::ExternalLinkage,0,varName,m_module);
        m_map[varName] = glob;
        return glob;
    } 
