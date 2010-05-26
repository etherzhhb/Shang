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
#include "instPriority.h"

namespace xVerilog {

    string instructionPriority::toString() {
        stringstream ss;
        /// XXX: O(n^2)?
        ss<<"InstructionPriority for \""<<BB->getNameStr()<<"\"["<<getMaxDepth()<<"]\n";
        for (unsigned int i=0; i<getMaxDepth(); i++) {
            ss<<i<<":"<<std::endl;
            for (BasicBlock::const_iterator it = BB->begin(); it != BB->end(); ++it) {
                if (i == m_depth[it])
                  // XXX: use getNameStr
                  ss<<"\t"<<(*it).getNameStr() <<std::endl;
            }

        }    

        return ss.str();
    }

    instructionPriority::instructionPriority(const BasicBlock* BB) {
        this->BB = BB;
        m_maxDepth = 0;
        calculateDeps();
    }

    void instructionPriority::replaceFirstUseOfWith(Instruction* inst, Instruction* cloned) {
        for (Instruction::use_iterator it = inst->use_begin(); it!= inst->use_end(); ++it) {
            if (Instruction* dep = dyn_cast<Instruction>(it)) {
                if (dep->getParent() == cloned->getParent()) {
                    // replace the use with 'cloned' instruction
                    for (Instruction::op_iterator ops = it->op_begin(); ops!= it->op_end(); ++ops) {
                        if (inst==*ops) {
                            *ops = cloned;
                            return; // Only do first use
                        }
                    } 
                } // Only if we are in the same BasicBlock
            } //"If it's not instruction, what is it?"<<*it;
        }
        assert(0 && "Unable to find original value to replace");
    }

    unsigned int instructionPriority::getLocalUses(const Instruction* inst, const BasicBlock* BB) {
        unsigned int uses = 0;
        // For all of the users of this instructions
        for (Instruction::const_use_iterator it = inst->use_begin(); it!= inst->use_end(); ++it) {
            // Which are really instructions
            if (const Instruction* d = dyn_cast<Instruction>(*it)) {
                // And are in this basic block
                if  (d->getParent() == BB) uses++;
            }
        }
        return uses;
    }

    unsigned int instructionPriority::getLatencyForInstruction(const Instruction* inst) const {
        return 1;
    }

    set<const Instruction*> instructionPriority::getDependencies(const Instruction* inst) {
        set<const Instruction*> deps;

        // We do not count PHINodes because they may create cyclic dependencies.
        // We only want to get dependencies which are within this BB's iteration.
        if (dyn_cast<PHINode>(inst)) {
            return deps;
        }

        // for all deps
        for (User::const_op_iterator dep_iter = inst->op_begin();
                dep_iter != inst->op_end(); ++dep_iter){
            // if this dep is an instruction rather then a parameter
            if (const Instruction* dep = dyn_cast<Instruction>(*dep_iter)) {
                if (dep->getParent() == BB)
                    deps.insert(dep);
            }
        }
        return deps;
    }


    void instructionPriority::calculateDeps() {
        set<const Instruction*> to_update;

        // Setup map for iterations
        for (BasicBlock::const_iterator I = BB->begin(), E = BB->end(); I != E; ++I) {
            if (0 == getLocalUses(I,BB)) {
                m_depth[I] = 0;
                to_update.insert(I);
            } else {
                // unused instructions are set to layer zero
                // which is the last layer
                m_depth[I] = 0;
            }
        }

        assert (m_depth.size() == BB->size() && "Map and BB are in different sizes");
        // While there are opcodes to update
        // do the following iterations
        while(0 != to_update.size()) {

            // what to update next round
            set<const Instruction*>  next_update;

            // for all instructions in the update list
            for (set<const Instruction*>::iterator it = to_update.begin(); 
                    it!= to_update.end(); ++it) {

                // for all instructions in the dependency list
                set<const Instruction*> deps = getDependencies(*it);
                for (set<const Instruction*>::iterator dp = deps.begin(); 
                        dp!= deps.end(); ++dp) {
                    // Each opcode is assigned to the longest distance
                    // either it's current distance or the path from the
                    // dependency

                    // We create the effect of layers of opcodes where some opcodes
                    // take more cycles by filling the dependency table with varying
                    // dependency length. see getLatencyForInstruction 
                    m_depth[*dp] = std::max(m_depth[*it] + getLatencyForInstruction(*it), m_depth[*dp]);
                    m_maxDepth = std::max(m_depth[*dp], m_maxDepth);
                    // we need to update the children of this opcode 
                    // in the next iteration
                    next_update.insert(*dp);
                }
            }
            // prepare next iteration 
            to_update.clear();
            to_update.insert(next_update.begin(), next_update.end());
        }
    }

} // namespace
