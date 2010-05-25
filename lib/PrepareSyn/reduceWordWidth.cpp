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
#include "llvm/DerivedTypes.h"
#include "llvm/Support/InstIterator.h"

#include <algorithm>
#include <sstream>

#include "vbe/utils.h"
#include "reduceWordWidth.h"

namespace xVerilog {

    bool ReduceWordWidthPass::removeUnusedInstructions(Instruction* inst) {
        if (dyn_cast<BinaryOperator>(inst) || dyn_cast<SExtInst>(inst) || dyn_cast<TruncInst>(inst)) {
            if (inst->hasNUses(0)) {
                inst->eraseFromParent();
                return true;
            }
        }

        // remove double casting
        if (TruncInst* tr = dyn_cast<TruncInst>(inst)) {
            if (SExtInst* se = dyn_cast<SExtInst>(tr->getOperand(0))) {
                    Value* v = se->getOperand(0);
                    unsigned int bitWidth0 = cast<IntegerType>(v->getType())->getBitWidth();
                    //unsigned int bitWidth1 = cast<IntegerType>(se->getType())->getBitWidth();
                    unsigned int bitWidth2 = cast<IntegerType>(tr->getType())->getBitWidth();
                     
                    if (se->hasOneUse() && bitWidth2 <= bitWidth0) {
                        tr->setOperand(0,v);
                        return true;
                    }
                
            } 

            unsigned int bitWidth1 = cast<IntegerType>(tr->getOperand(0)->getType())->getBitWidth();
            unsigned int bitWidth2 = cast<IntegerType>(tr->getType())->getBitWidth();
            if (bitWidth1 == bitWidth2 && tr->hasOneUse()) {
                tr->replaceAllUsesWith(tr->getOperand(0));
            }

        }

        return false;
    }

    bool ReduceWordWidthPass::reduceOperatorWordWidth(Instruction* inst) {

        if (BinaryOperator *calc = dyn_cast<BinaryOperator>(inst)) {
            Value *v0 = (calc->getOperand(0));
            Value *v1 = (calc->getOperand(1));
            // if we do a shift right then we loose bits ...
            if (calc->getOpcode() == Instruction::LShr) {
                if (dyn_cast<ConstantInt>(v1)) {
                    ConstantInt *con = dyn_cast<ConstantInt>(v1);
                    unsigned int bitWidth = cast<IntegerType>(calc->getType())->getBitWidth();
                    unsigned int shlVal = con->getValue().getZExtValue();
                    unsigned int newBitLen = bitWidth - shlVal;
                    if (bitWidth != newBitLen) {
                        CastInst* tv0 = new TruncInst(v0, IntegerType::get(newBitLen),"trunc_shl",calc);
                        CastInst* tv1 = new TruncInst(v1, IntegerType::get(newBitLen),"trunc_shl",calc);
                        BinaryOperator* newAnd = BinaryOperator::Create(Instruction::LShr, tv0, tv1,"reduced_and",calc);
                        CastInst* castedAnd = new SExtInst(newAnd, IntegerType::get(bitWidth),"exand",calc);
                        calc->replaceAllUsesWith(castedAnd);
                        return true;
                    }
                }

            }


            if (calc->getOpcode() == Instruction::Add) {
                // if both parameters are 'SExt' then we can resuce them to the smaller size
                if ((dyn_cast<SExtInst>(v0)) && (dyn_cast<SExtInst>(v1))) {
                    SExtInst* i0 = dyn_cast<SExtInst>(v0);
                    SExtInst* i1 = dyn_cast<SExtInst>(v1);
                    unsigned int currentBitWidth = cast<IntegerType>(calc->getType())->getBitWidth();
                    unsigned int bitWidth0 = cast<IntegerType>(i0->getOperand(0)->getType())->getBitWidth();
                    unsigned int bitWidth1 = cast<IntegerType>(i1->getOperand(0)->getType())->getBitWidth();
                    unsigned int combinedBitWidth = std::max(bitWidth0, bitWidth1) + 1;
                    if (combinedBitWidth < currentBitWidth) {

                        CastInst* cast0 = new SExtInst(i0->getOperand(0), IntegerType::get(combinedBitWidth),"exand",calc);
                        CastInst* cast1 = new SExtInst(i1->getOperand(0), IntegerType::get(combinedBitWidth),"exand",calc);
                        BinaryOperator* newAdd = BinaryOperator::Create(Instruction::Add, cast0, cast1,"reduced_and",calc);
                        CastInst* castedAnd = new SExtInst(newAdd, IntegerType::get(currentBitWidth),"exand",calc);
                        calc->replaceAllUsesWith(castedAnd);
                        return true;
                    }
                }
            }


            // If this instruction is an AND instruction which may reduce the size of this ...
            if (calc->getOpcode() == Instruction::And) {
                if (dyn_cast<ConstantInt>(v1)) {
                    Value *t = v1;
                    v1 = v0;
                    v0 = t;
                }
                if (dyn_cast<ConstantInt>(v0)) {
                    ConstantInt *con = dyn_cast<ConstantInt>(v0);
                    unsigned int bitWidth = cast<IntegerType>(calc->getType())->getBitWidth();
                    unsigned int bitWidth1 = cast<IntegerType>(v1->getType())->getBitWidth();
                    unsigned int active = con->getValue().getActiveBits(); 
                    if (bitWidth != active && bitWidth1 != active) {

                        CastInst* tv0 = new TruncInst(v0, IntegerType::get(active),"trunc",calc);
                        CastInst* tv1 = new TruncInst(v1, IntegerType::get(active),"trunc",calc);
                        BinaryOperator* newAnd = BinaryOperator::Create(Instruction::And, tv0, tv1,"reduced_and",calc);
                        CastInst* castedAnd = new SExtInst(newAnd, IntegerType::get(bitWidth),"exand",calc);
                        calc->replaceAllUsesWith(castedAnd);
                        return true;
                    }
                }

            }
        }
        return false;
    }

    bool ReduceWordWidthPass::runOnFunction(Function &F) {

        bool changed = false;
        bool changed_global = false;

        do {
            changed = false;
            // F is a pointer to a Function instance
            for (inst_iterator i = inst_begin(F), e = inst_end(F); i != e; ++i) {
                Instruction *inst = &*i;
                changed |= reduceOperatorWordWidth(inst);
                changed |= removeUnusedInstructions(inst);
            }
            changed_global |= changed;
        } while (changed);

        //if (changed_global) logPassMessage(__FUNCTION__,__LINE__,"Reduced bus width");
        return changed_global;
    }



    char ReduceWordWidthPass::ID = 0;
    RegisterPass<ReduceWordWidthPass> RWXX("reduce_bitwidth", "reduce the width of words whenever possible");

} //namespace


