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
#include "parallelPass.h" 

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

#include <algorithm>
#include <sstream>
#include "vbe/utils.h"

namespace xVerilog {


    bool ParallelPass::isLikelyToBeFatNode(Instruction* inst) {
         if (dyn_cast<PHINode>(inst->getOperand(0))) return true; 
         if (dyn_cast<PHINode>(inst->getOperand(1))) return true; 
        return false;
    }

    void ParallelPass::findAllChildrenOfSameType(llvm::Instruction::BinaryOps type,
            Value* inst, vector<Instruction*> *nodes, vector<Value*> *leafs) {

        if (dyn_cast<BinaryOperator>(inst) && inst->hasOneUse()) {
            BinaryOperator* bin=(BinaryOperator*) inst;
            if (bin->getOpcode() == type) {
                // if this is the first node we are processing
                // OR is this node is in the save BasicBlock as the first node
                if (0==nodes->size() || (*nodes)[0]->getParent() == bin->getParent()) {
                    nodes->push_back(bin);
                    findAllChildrenOfSameType(type, bin->getOperand(0), nodes, leafs);
                    findAllChildrenOfSameType(type, bin->getOperand(1), nodes, leafs);
                }
            } else leafs->push_back(inst);
        } else leafs->push_back(inst);
    }


    bool ParallelPass::floodParallelCommutative(Instruction *inst,
            llvm::Instruction::BinaryOps binType) {
        vector<Instruction*> nodes;
        vector<Value*> leafs;
        // Is this an instruction of the correct type (ex: Add)?
        if (dyn_cast<BinaryOperator>(inst)) {
            BinaryOperator* bin=(BinaryOperator*) inst;
            if (bin->getOpcode() != binType) {
                return false;
            }
        } else { 
            return false;
        }

        // Find all children nodes
        findAllChildrenOfSameType(binType, inst->getOperand(0), &nodes, &leafs);
        findAllChildrenOfSameType(binType, inst->getOperand(1), &nodes, &leafs);

        // if we only found one node then we can't rearrange 
        // things. return with no change.
        if (nodes.size()<3) return false;       

        // A map used to create the balanced tree structure
        // very much like hoffman tree (just balanced)
        map<Instruction*, unsigned int> rank_map;
        // iterators
        vector<Instruction*>::iterator node_it = nodes.begin();
        vector<Value*>::iterator leaf_it = leafs.begin();
        map<Instruction*, unsigned int>::iterator map_it;

        // take all the leafs and join them with 'bin' nodes
        // creates the first layer of the pyramid
        Instruction* last_bin = NULL; 
        while(leaf_it != leafs.end()) {
            // take two leafs, join then with an arithmetic operation
            // and store them in the hash table with rank one.
            // this is the bottom of the balanced tree
            (*node_it)->setOperand(0,*leaf_it); leaf_it++;
            (*node_it)->setOperand(1,*leaf_it); leaf_it++;
            (*node_it)->setName("lowlevel");

            if (isLikelyToBeFatNode(*node_it)) {
                rank_map[*node_it] = 100;
            } else {
                rank_map[*node_it] = 1;
            }
            last_bin = *node_it;
            ++node_it;

            assert(last_bin && "unable to find last bin");
            assert((*node_it) && "unable to find node_it");
            // If th number of nodes is odd (we have one leaf left),
            // join the last leaf with one of the nodes
            if (leaf_it == --leafs.end()) {
                (*node_it)->setOperand(0,*leaf_it); leaf_it++;
                (*node_it)->setOperand(1,last_bin);
                (*node_it)->setName("lowlevelOdd");
                rank_map[*node_it] = 2;
                rank_map.erase(last_bin);
                node_it++;
            }
        }

        // We now will reconstruct the rest of the balanced tree, 
        // joining each two nodes with the lowest ranks together.
        // repeat until we only have two nodes
        while (node_it != nodes.end()) {
            Instruction* minInst1 = NULL;
            unsigned int minRank1 = 100000;
            Instruction* minInst2 = NULL;
            unsigned int minRank2 = 100000;

            // find first node with the lowest rank
            for (map_it = rank_map.begin(); map_it!=rank_map.end(); map_it++) {
                if ((map_it->second) <= minRank1) {
                    minInst1 = map_it->first;
                    minRank1 = map_it->second;
                }
            } 
            // find the second node with the lowest rank
            for (map_it = rank_map.begin(); map_it!=rank_map.end(); map_it++) {
                if ((map_it->first != minInst1) && (map_it->second) <= minRank2) {
                    minInst2 = map_it->first;
                    minRank2 = map_it->second;
                }
            }


            // assert your findings 
            assert(minInst1 && "minInst1 is NULL");
            assert(minInst2 && "minInst2 is NULL");
            assert(inst && "inst is NULL");
            assert(minInst1->getParent() == inst->getParent() && "min1 not same bb as inst");
            assert(minInst2->getParent() == inst->getParent() && "min1 not same bb as inst");
            assert(*node_it && "node_it  is NULL");

            // set the arithmetic node to point to two of the ranks 
            (*node_it)->setOperand(0,minInst1);  
            (*node_it)->setOperand(1,minInst2);
            (*node_it)->setName("pyramid");// give it a somewhat meaningful name

            // place the new node in the map. remove the two old onces from the map
            rank_map.erase(minInst1);
            rank_map.erase(minInst2);
            rank_map[*node_it] = 1 + std::max(minRank1, minRank2);
            ++node_it;
            minInst2->moveBefore(inst);
            minInst1->moveBefore(inst);
        }

        // After we have only two nodes left, we set them as the children of the 
        // head node. Then we move all of the binary nodes just before the headnode
        // so the order of the dependencies is correct.

        assert(inst && "inst is NULL");
        assert(rank_map.size() == 2 && "more then two add nodes left at the end of the round!");
        inst->setName("headNode");
        inst->setOperand(0,(rank_map.begin())->first);
        inst->setOperand(1,(++rank_map.begin())->first);

        // move all nodes before the last instruction so they
        // domminate all dependencies
        for (node_it = nodes.begin(); node_it != nodes.end(); ++node_it) {
            (*node_it)->moveBefore(inst);
        }

        //logPassMessage(__FUNCTION__,__LINE__,"Balanced tree");
        return true;
    }

    bool ParallelPass::areAllUsersOfOtherType(Instruction* inst, llvm::Instruction::BinaryOps binType) {
        // for all users
        for (Instruction::use_iterator us = inst->use_begin(); us!= inst->use_end(); ++us) {
            // If this user is a binary operator
            if (BinaryOperator* bin = dyn_cast<BinaryOperator>(us)) {
                // of the same type
                if (bin->getOpcode() == binType) {
                    return false;
                }
            }
        }
        return true;
    }


    void ParallelPass::floodParallel(BasicBlock &BB, llvm::Instruction::BinaryOps binType) {
        // start from the end of the basic block. Try to rearrange 
        // the chain of binary operations. 
        for (BasicBlock::iterator it=BB.begin();it!=BB.end();++it) {
            // Is this an instruction of the correct type (ex: Add)?
            BinaryOperator* bin = dyn_cast<BinaryOperator>(it);
            if (bin && (bin->getOpcode() == binType)) {
                // we only start with head of pyramid
                if (areAllUsersOfOtherType(bin,binType)) {
                    floodParallelCommutative(bin, binType);
                }
            } 
        }//BB
    }

    bool ParallelPass::runOnFunction(Function &F) {
        // for each BasicBlock, parallel the instructions
        for (Function::iterator BB = F.begin(), E = F.end(); BB != E; ++BB) {
            ParallelPass::floodParallel(*BB, Instruction::Add);
            ParallelPass::floodParallel(*BB, Instruction::Mul);
            ParallelPass::floodParallel(*BB, Instruction::Or);
            ParallelPass::floodParallel(*BB, Instruction::Xor);
            ParallelPass::floodParallel(*BB, Instruction::And);
        }
        return true;
    }

    char ParallelPass::ID = 0;
    RegisterPass<ParallelPass> PXX("parallel_balance", "extract parallelism from expressions");

} //namespace


