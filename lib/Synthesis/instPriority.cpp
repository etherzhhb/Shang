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

    instructionPriority::instructionPriority(BasicBlock* BB) {
        this->BB = BB;
        calculateDeps();
    }

    unsigned int instructionPriority::getLocalUses(Instruction* inst) {
        unsigned int uses = 0;
        // For all of the users of this instructions
        for (Instruction::use_iterator it = inst->use_begin(); it!= inst->use_end(); ++it) {
            // Which are really instructions
            if (Instruction* d = dyn_cast<Instruction>(*it)) {
                // And are in this basic block
                if  (d->getParent() == BB) uses++;
            }
        }
        return uses;
    }

    set<Instruction*> instructionPriority::getDependencies(Instruction* inst) {
        set<Instruction*> deps;

        // We do not count PHINodes because they may create cyclic dependencies.
        // We only want to get dependencies which are within this BB's iteration.
        if (dyn_cast<PHINode>(inst)) {
            return deps;
        }

        // for all deps
        for (User::op_iterator dep_iter = inst->op_begin();
                dep_iter != inst->op_end(); ++dep_iter){
            // if this dep is an instruction rather then a parameter
            if (Instruction* dep = dyn_cast<Instruction>(*dep_iter)) {
                if (dep->getParent() == BB)
                    deps.insert(dep);
            }
        }
        return deps;
    }


    void instructionPriority::calculateDeps() {
        set<Instruction*> to_update;

        // Setup map for iterations
        for (BasicBlock::iterator I = BB->begin(), E = BB->end(); I != E; ++I) {
            if (0 == getLocalUses(I)) {
                m_depth[I] = 1;
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
            set<Instruction*>  next_update;

            // for all instructions in the update list
            for (set<Instruction*>::iterator it = to_update.begin(); 
                    it!= to_update.end(); ++it) {

                // for all instructions in the dependency list
                set<Instruction*> deps = getDependencies(*it);
                for (set<Instruction*>::iterator dp = deps.begin(); 
                        dp!= deps.end(); ++dp) {
                    // Each opcode is assigned to the longest distance
                    // either it's current distance or the path from the
                    // dependency
                    m_depth[*dp] = std::max(m_depth[*it] + 1, m_depth[*dp]);
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

    InstructionVector instructionPriority::getOrderedInstructions() {
        // a vector for storing th opcodes. The opcode with the highest
        //  dependency height (are in the critical path) come first 
        InstructionVector order; 
        // terminators MUST come at the end of all opcodes
        InstructionVector terminators; 

        assert (m_depth.size() == BB->size() && "Map and BB are in different sizes");

        unsigned int maximum = 0;
        // find the opcode with the highest layer number 
        for (InstPriorityMap::iterator it = m_depth.begin(); it != m_depth.end(); ++it) {
            maximum = std::max(it->second,maximum);
        }

        // start putting them in the layers, from last to first
        for (unsigned int iter=maximum+1; iter>0; --iter) {
            for (InstPriorityMap::iterator it = m_depth.begin(); it != m_depth.end(); ++it) {
                // if it is the turn of this BFS layer
                if ((iter-1) == it->second) {
                    // terminators come at the end
                    if (it->first->isTerminator()) {
                        terminators.push_back(it->first);
                    } else {
                        order.push_back(it->first);
                    }
                }
            }
        }
        // put terminators at the end of the order list
        order.insert(order.end(), terminators.begin(), terminators.end());
        assert (order.size() == BB->size() && "Returning list in different sizes");
        return order;
    }


} // namespace
