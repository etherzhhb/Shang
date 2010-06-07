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
#include "designScorer.h"
#include "vbe/ResourceConfig.h"

namespace xVerilog {


    double designScorer::getDesignFrequency() {
        unsigned int mul_pipes = ResourceConfig::getResConfig("delay_mul");
        unsigned int shl_pipes = ResourceConfig::getResConfig("delay_shl");
        unsigned int div_pipes = ResourceConfig::getResConfig("delay_div");


        unsigned int min_stages = 
            std::min(mul_pipes, std::min(shl_pipes,div_pipes));

        double max_time = 3.5;

            if (0 == min_stages)  max_time = 100.0;
            else if (1 == min_stages)  max_time = 11.112;
            else if (2 == min_stages)  max_time = 6.737;
            else if (3 == min_stages)  max_time = 5.190;
            else if (4 == min_stages)  max_time = 4.852;
            else if (5 == min_stages)  max_time = 3.856;
            else if (6 == min_stages)  max_time = 3.645;
            else if (7 == min_stages)  max_time = 3.630;
            else if (min_stages < 16) max_time = 3.60;

        /*double max_time = 0;
          for (listSchedulerVector::iterator it = m_basicBlocks.begin(); it != m_basicBlocks.end(); ++it) {
          max_time = std::max(max_time, getBasicBlockMaxDelay(*it));
          }*/

        return max_time;
    }

    unsigned int designScorer::getDesignClocks() {
        unsigned int max_loop_clocks = 0;
        unsigned int max_clocks = 0;
        for (listSchedulerVector::iterator it = m_basicBlocks.begin(); 
                it != m_basicBlocks.end(); ++it) {
            // Is this BasicBlock a part of a loop ?
            if (m_loopInfo->getLoopFor((*it)->getBB())) {
                max_loop_clocks = std::max(max_loop_clocks, getBasicBlockClocks(*it));
            }
                max_clocks = std::max(max_clocks, getBasicBlockClocks(*it));
        }
        if (max_loop_clocks != 0) return max_loop_clocks;
        return max_clocks;
    }

    double designScorer::getLoopBlocksCount() {
        unsigned int bbs, lbbs;
        bbs = 0;
        lbbs = 0;

        for (listSchedulerVector::iterator it = m_basicBlocks.begin(); it != m_basicBlocks.end(); ++it) {
            bbs+=(*it)->length();
            // Is this BasicBlock a part of a loop ?
            if (m_loopInfo->getLoopFor((*it)->getBB())) {
                lbbs+=(*it)->length();
            }
        }

        if (lbbs==0) lbbs = bbs; //if no loops then all of the design is interesting

        return (double)lbbs/(double)bbs;
    }

    double designScorer::getBasicBlockMaxDelay(listScheduler* ls) {
        double max_delay = 0;
        //for each cycle in this basic block
        for (unsigned int cycle=0; cycle<ls->length();cycle++) {
            vector<Instruction*> inst = ls->getInstructionForCycle(cycle);
            //for each instruction in cycle, find it's delay ...
            for (vector<Instruction*>::iterator ii = inst.begin(); ii != inst.end(); ++ii) {
                max_delay = std::max(max_delay, getDelayForInstruction(*ii));
            }
        }// for each cycle      

        return max_delay;
    }


    //
    // Times for Virtex4  XC4VLX25
    //
    //
    double designScorer::getDelayForInstruction(Instruction *inst) {

        const double BASE_ASSIGN_DELAY = 1.849;

        double delay = 0;

        if (isa<LoadInst>(inst)) delay =  BASE_ASSIGN_DELAY;
        if (isa<StoreInst>(inst)) delay = BASE_ASSIGN_DELAY;
        if (isa<SelectInst>(inst)) delay = BASE_ASSIGN_DELAY;
        if (isa<PHINode>(inst)) delay = BASE_ASSIGN_DELAY;

        // Converting bits from one format to another
        if (isa<BitCastInst>(inst)) delay = 0.1;
        if (isa<IntToPtrInst>(inst)) delay = 0.1;
        if (isa<PtrToIntInst>(inst)) delay = 0.1;
        if (isa<ZExtInst>(inst)) delay = 0.1;
        if (isa<TruncInst>(inst)) delay =  0.1;

        // Binary instructions
        if (isa<BinaryOperator>(inst))  {
            BinaryOperator* bin = (BinaryOperator*) inst;
            // Simple binary functions 
            if (bin->getOpcode() == Instruction::Add || bin->getOpcode() == Instruction::Sub) {
                delay =  1.849;
            }
            if (bin->getOpcode() == Instruction::UDiv ) {
                delay =  10; // can't synthesis this
            }
            if (bin->getOpcode() == Instruction::Mul ) {
                delay =  7.72;
            }
            if (bin->getOpcode() == Instruction::And || 
                    bin->getOpcode() == Instruction::Or || 
                    bin->getOpcode() == Instruction::Xor) { 
                delay =  BASE_ASSIGN_DELAY * 1.3; // I guess that it's almost free
            } 

            // Shift, may be constant (routing ), may be assign part (shl module)
            if ((bin->getOpcode()) == Instruction::Shl || bin->getOpcode() == Instruction::LShr) {
                // shift by constant
                if (isa<Constant>(bin->getOperand(1))) delay = 0;
                // Assign part
                delay =  BASE_ASSIGN_DELAY;
            } 
        }

        const Type* Ty = inst->getType();
        // if we don't know this type, don't normalize it
        if (!Ty->isPrimitiveType() || !Ty->isIntegerTy() || !Ty->isSized()) return delay;
        if (Ty->getTypeID() ==  Type::IntegerTyID) {
            unsigned NBits = cast<IntegerType>(Ty)->getBitWidth();
            delay = (delay/32)*NBits;
        }

        return delay;
    }

    unsigned int designScorer::getDesignSizeInGates(Function* F) {

        unsigned int totalGateSize = 0;

        unsigned int mul_count = ResourceConfig::getResConfig("mul");
        unsigned int div_count = ResourceConfig::getResConfig("div");
        unsigned int shl_count = ResourceConfig::getResConfig("shl");

        totalGateSize += mul_count*1088 + div_count*1500 + shl_count*1000;

        for (inst_iterator i = inst_begin(*F), e = inst_end(*F); i != e; ++i) {
            totalGateSize += getInstructionSize(&*i); 
        }
        return totalGateSize;
    }

    int designScorer::getInstructionSize(Instruction* inst) {
        const Type* Ty = inst->getType();
        if (!Ty->isPrimitiveType() || !Ty->isIntegerTy() || !Ty->isSized()) return 1;
        unsigned int gates = 0;

        switch (Ty->getTypeID()) {
            case Type::VoidTyID: { 
                                     gates = 1; // one bit
                                 }
            case Type::PointerTyID: {   // define the verilog pointer type
                                        gates = m_pointerSize; // 32bit for our pointers
                                    }
            case Type::IntegerTyID: { // define the verilog integer type
                                        unsigned NBits = cast<IntegerType>(Ty)->getBitWidth();
                                        gates = NBits;
                                    }
            default: gates = 1;
        }

        if (BinaryOperator *calc = dyn_cast<BinaryOperator>(inst)) {
            if (calc->getOpcode() == Instruction::LShr) {
                gates *= 7;
            }
            if (calc->getOpcode() == Instruction::AShr) {
                gates *= 7;
            }
            if (calc->getOpcode() == Instruction::Sub) {
                gates *= 5;
            }
            if (calc->getOpcode() == Instruction::Add) {
                gates *= 5;
            }
            if (calc->getOpcode() == Instruction::Shl) {
                gates *= 9;
            }
        }
        return gates;

    }


} // namespace
