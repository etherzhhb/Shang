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
#include "llvm/Analysis/FindUsedTypes.h" 
#include "llvm/DerivedTypes.h"
#include "llvm/Support/InstIterator.h"

#include <algorithm>
#include <sstream>

#include "vbe/utils.h"
#include "subscripts.h" 

namespace xVerilog {
    
    void subscripts::incrementAccessIndex(Value* inst, int offset) {
        Value* exp; 
        Value* ld = isLoadSubscript(inst);
        Value* st = isStoreSubscript(inst);
        exp = (ld? ld : (st? st: NULL ));
        if (NULL == exp) cerr<<"offset:"<<offset<<"  inst:"<<*inst;
        assert(exp && "Inst is nither Load or Store array subscript");
        GetElementPtrInst *ptr = dyn_cast<GetElementPtrInst>(exp);
        assert(ptr && "ptr is not of GetElementPtrInst type");
        // Pointer to the value that the GetElementPtrInst uses as index of the array
        Value* index = ptr->getOperand(1);
       
        // In this case, change the 'ADD' instruction to reflect the offset
        if (BinaryOperator* bin = dyn_cast<BinaryOperator>(index)) {
            unsigned int bitWidth = cast<IntegerType>(index->getType())->getBitWidth();

            // if this index is subtracted then we need to negate it first. A[i-(constant+offset)];
            if (bin->getOpcode() == Instruction::Sub) {
                offset=-offset;
            }

            if (bin->getOpcode() == Instruction::Add || bin->getOpcode() == Instruction::Sub) {
                for (int param_num=0; param_num<2;param_num++) {
                    if (ConstantInt *con = dyn_cast<ConstantInt>(bin->getOperand(param_num))) {
                        unsigned int current_off = con->getValue().getZExtValue();
                        // create a new constant
                        ConstantInt *ncon = ConstantInt::get(APInt(bitWidth, offset + current_off));
                        bin->setOperand(param_num,ncon);
                    }
                }
            } else {
                // we can't modify an existing node. Create a new add.
                addOffsetToGetElementPtr(ptr,offset);
            }

        } else if (isInductionVariable(index)) {
            // Else, we add an 'ADD' instruction for the PHI variable
            addOffsetToGetElementPtr(ptr,offset);
        } else assert(0 && "unknown type of index for array");


    }


    bool subscripts::isUsedByInductionVariable(Instruction* inst) {
        // get the IV
        Instruction *iv = getInductionVar();
        // search 'use' chain and see if iv is one of them
        if (std::find(inst->use_begin(), inst->use_end(), iv) != inst->use_end()) {
            return true;
        }
        return false;
    }


    void subscripts::addOffsetToGetElementPtr(Value *inst, int offset) {
        GetElementPtrInst* ptr = dyn_cast<GetElementPtrInst>(inst);
        assert(ptr && "inst is not of GetElementPtrInst type");
        Value* index = ptr->getOperand(1);
        // what's the bitwidth of our index?
        unsigned int bitWidth = cast<IntegerType>(index->getType())->getBitWidth();
        // New add instruction, place it before the GetElementPtrInst 
        BinaryOperator* newIndex = BinaryOperator::Create(Instruction::Add, 
                ConstantInt::get(APInt(bitWidth, offset)) , index, "i_offset", ptr);
        // Tell GetElementPtrInst to use it instead of the normal PHI
        ptr->setOperand(1,newIndex); 
    }

    bool subscripts::isInductionVariable(Value *inst) {
        return (m_inductionVariables.find(inst) != m_inductionVariables.end() );
    }

    bool subscripts::isModifiedInductionVariable(Value *inst) {
        if (BinaryOperator *calc = dyn_cast<BinaryOperator>(inst)) {
            Value *v0 = (calc->getOperand(0));
            Value *v1 = (calc->getOperand(1));
            if ( isInductionVariable(v0) || isInductionVariable(v1)) {
                return true;
            }
        }
        return false;
    }

    Value* subscripts::isAddressCalculation(Value *inst) {
        if (GetElementPtrInst *ptr = dyn_cast<GetElementPtrInst>(inst)) {
            Value *v = (ptr->getOperand(1));
            if (isModifiedInductionVariable(v) || isInductionVariable(v)) {
                return ptr;
            }
        }
        return NULL;
    }

    Value* subscripts::isLoadSubscript(Value *inst) {
        if (LoadInst *ptr = dyn_cast<LoadInst>(inst)) {
            Value *v = ptr->getOperand(0);
            return isAddressCalculation(v);
        }
        return NULL;
    }

    Value* subscripts::isStoreSubscript(Value *inst) {
        if (StoreInst *ptr = dyn_cast<StoreInst>(inst)) {
            Value *v = ptr->getOperand(1);
            return isAddressCalculation(v);         
        }
        return NULL;
    }

    void subscripts::detectInductionVariables(Loop* loop) {
        // for each BB, find it's loop
        for (Loop::block_iterator I = loop->block_begin(), E = loop->block_end(); I!=E; ++I) {
                //TODO: WTF ?
                PHINode* inductionVar = loop->getCanonicalInductionVariable();
                if (inductionVar) { 
                    // Collect all induction variables in here
                    m_inductionVariables.insert(inductionVar);
                }
        }
    }

    void subscripts::detectSubscripts(BasicBlock* bb) {
            for (BasicBlock::iterator i = (bb)->begin(); i != (bb)->end(); ++i) {
                Instruction *inst = i;
                if (isStoreSubscript(inst)) {
                    //cerr<<"Found Store "<<*inst;
                    m_StoreSubscripts.insert(inst);
                } else if (isLoadSubscript(inst)) {
                    //cerr<<"Found Load "<<*inst;
                    m_LoadSubscripts.insert(inst);
                }
            }
    }

    void subscripts::detectSubscripts(Loop* loop) {
        // detect A[i] accesses over all basic blocks in this loop
        for (Loop::block_iterator b = loop->block_begin(); b != loop->block_end(); ++b) {
            detectSubscripts(*b);
        }
    }
    
    void subscripts::normalizeSubscripts() {
        // by incrementing all we force them to be in the form A[i+0]
        incrementAllSubscripts(0);        
    }

    void subscripts::incrementIfSubscript(Value* inst, int offset) {

        if (m_LoadSubscripts.end() != m_LoadSubscripts.find(inst) ||
                m_StoreSubscripts.end() != m_StoreSubscripts.find(inst)) {
            incrementAccessIndex(inst, offset);
        }

    }

    void subscripts::incrementAllSubscripts(int offset) {
        for(set<Value*>::iterator it = m_LoadSubscripts.begin(); it!= m_LoadSubscripts.end(); ++it) {
            incrementAccessIndex(*it,offset);
        }
        for(set<Value*>::iterator it = m_StoreSubscripts.begin(); it!= m_StoreSubscripts.end(); ++it) {
            incrementAccessIndex(*it,offset);
        }
    }

    Instruction* subscripts::incrementValue(Value* val, int offset, Instruction* insert) {
        if (offset == 0 && dyn_cast<Instruction>(val)) {
            // If we do not need to change the offset and we know
            // that val is an instruction
            return dyn_cast<Instruction>(val);
        }
        unsigned int bitWidth = cast<IntegerType>(val->getType())->getBitWidth();
        ConstantInt *ncon = ConstantInt::get(APInt(bitWidth, offset));
        if (insert) {
            return BinaryOperator::Create(Instruction::Add, ncon, val, "incrementVal",insert);
        } else {
            return BinaryOperator::Create(Instruction::Add, ncon, val, "incrementVal");
        }
    }

} //namespace


