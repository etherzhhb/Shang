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
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/Support/CFG.h"
#include "llvm/DerivedTypes.h"
#include "llvm/Support/InstIterator.h"
#include "llvm/Transforms/Utils/Cloning.h"

#include <algorithm>
#include <sstream>

#include "instPriority.h"

#include "vbe/utils.h"
#include "ModuleSchedulerDriver.h"
#include "subscripts.h"

namespace xVerilog {


    bool ModuloSchedulerDriverPass::loop_is_ms_able(Loop* loop) {
        // must have a pre-header
        if (!loop->getLoopPreheader()) return false;
        // must have only one block
        if (1 != loop->getBlocks().size()) return false;

        // For each BasicBlock in this loop
        for (Loop::block_iterator bbit = loop->block_begin(); bbit != loop->block_end(); ++bbit) {
            // Get all of the Load addresses and all of the Store addresses
            std::vector<Value*> array_stores = getAllArrayAccess(*bbit,true);
            std::vector<Value*> array_loads  = getAllArrayAccess(*bbit,false);
            // Make sure we do not read and write from the same array
            for (std::vector<Value*>::iterator arit = array_stores.begin(); 
                                               arit != array_stores.end(); ++arit) {
                if (std::find(array_loads.begin(), array_loads.end(), *arit) != array_loads.end()) {
                    //cerr<<"Unable to operate MS since we read and write to the same array in:"<<**bbit;
                    return false;
                }
            }
            
            for (BasicBlock::iterator it = (*bbit)->begin(); it!= (*bbit)->end(); ++it) {
                if (PHINode* phi = dyn_cast<PHINode>(it)) {
                    // For each incoming edge on the PHINode
                    unsigned int incoming = phi->getNumIncomingValues();
                    for (unsigned int i=0; i< incoming; i++) {
                        // If our PHINode depends on another PHINode.
                        if (dyn_cast<PHINode>(phi->getIncomingValue(i))) return false;
                    }      
                }
            }

            // we have no bad instructions (call, return, etc)
            for (BasicBlock::iterator it = (*bbit)->begin(); it!= (*bbit)->end(); ++it) {
                if (dyn_cast<CallInst>(it)) return false;
                if (dyn_cast<ReturnInst>(it)) return false;
#if 0
                if (dyn_cast<AllocationInst>(it)) return false;
#endif
                if (dyn_cast<UnwindInst>(it)) return false;
                if (dyn_cast<UnreachableInst>(it)) return false;
#if 0
                if (dyn_cast<FreeInst>(it)) return false;
#endif
                if (dyn_cast<VAArgInst>(it)) return false;
                if (dyn_cast<ExtractElementInst>(it)) return false;
                if (dyn_cast<InsertElementInst>(it)) return false;
                if (dyn_cast<ShuffleVectorInst>(it)) return false;
            }
        }
        return true; 
    }


    Value* ModuloSchedulerDriverPass::copyLoopBodyToHeader(Instruction* inst,
            Instruction* induction, BasicBlock* header, int offset){

        // Holds the body of the interesting loop
        BasicBlock *body = inst->getParent();

        assert(header && "Header is null");
        assert(header->getTerminator() && "Header has no terminator");

        // Maps the old instructions to the new Instructions
        DenseMap<const Value *, Value *>  ValueMap;
        // Do the actual clone
        stringstream iname;
        iname<<"___"<<offset<<"___";
        BasicBlock* newBB = CloneBasicBlock(body, ValueMap, iname.str().c_str());

        // Fixing the dependencies for each of the instructions in the cloned BB
        // They now depend on themselves rather on the old cloned BB.
        for (BasicBlock::iterator it = newBB->begin(); it != newBB->end(); ++it) {
            for (Instruction::op_iterator ops = (it)->op_begin(); ops != (it)->op_end(); ++ops) {
                if (ValueMap.end() != ValueMap.find(*ops)) {
                    //*ops = ValueMap[*ops];
                    it->replaceUsesOfWith(*ops, ValueMap[*ops]);
                }
            }
        }

        // Fixing the PHI nodes since they are no longer needed
        for (BasicBlock::iterator it = newBB->begin(); it != newBB->end(); ++it) {
            if (PHINode *phi = dyn_cast<PHINode>(it)) {
                // Taking the preheader entryfrom the PHI node

                Value* prevalue = phi->getIncomingValue(phi->getBasicBlockIndex(header));
                assert(prevalue && "no prevalue. Don't know what to do");

                // If we are handling a PHI node which is the induction index ? A[PHI(i,0)] ?
                // If so, turn it into A[i + offset]
                if (ValueMap[induction] == phi) {
                    Instruction *add = subscripts::incrementValue(prevalue, offset);
                    //add->insertBefore(phi); This is the same as next line (compiles on LLVM2.1)
                    phi->getParent()->getInstList().insert(phi, add);
                    phi->replaceAllUsesWith(add);
                }  else {
                    // eliminating the PHI node all together
                    // This is just a regular variable or constant. No need to increment
                    // the index.
                    phi->replaceAllUsesWith(prevalue);
                }
            } 
        }

        // Move all non PHI and non terminator instructions into the header.
        while (!newBB->getFirstNonPHI()->isTerminator()) {
            Instruction* inst = newBB->getFirstNonPHI();
            if (dyn_cast<StoreInst>(inst)) {
                inst->eraseFromParent();
            } else {
                inst->moveBefore(header->getTerminator());
            }
        }
        newBB->dropAllReferences();
        return ValueMap[inst];
    }

    bool ModuloSchedulerDriverPass::runOnLoop(Loop *IncomingLoop, LPPassManager &LPM_Ref) {
      
        subscripts subs(IncomingLoop);

        if (!loop_is_ms_able(IncomingLoop) ) return false; 

        // The header before the parallelized loop will be placed here
        BasicBlock* preheader = IncomingLoop->getLoopPreheader();
        assert(preheader && "Unable to get a hold of the preheader");

        // Balance all BasicBlocks in this loop
        for (Loop::block_iterator it=IncomingLoop->block_begin(); it!=IncomingLoop->block_end();++it) {
            duplicateValuesWithMultipleUses(*it,subs.getInductionVar());
        }

        // For each BB in loop
        for (Loop::block_iterator it=IncomingLoop->block_begin(); it!=IncomingLoop->block_end();++it) {
            instructionPriority  ip(*it);
            (*it)->setName("PipelinedLoop");
            
            // ++++++++ Preheader part +++++++++
            // Make a copy of the body for each instruction. Place a pointer to the 
            // parallel cloned instruction in the map below. Later on we will replace it 
            // with a PHINode.
            DenseMap<const Value *, Value *>  InstToPreheader;

            // For each Instruction in body of the loop, clone, store, etc.
            for (BasicBlock::iterator ib = (*it)->begin(), eb = (*it)->end(); ib!=eb; ++ib) {
                // If this is NOT a phi node
                if (!dyn_cast<PHINode>(ib)) {
                    // Get the priority of the instruction
                    unsigned int p = ip.getPriority(ib);
                    // This is the header version of each variable that goes into a PHI node.
                    // The other edge needs to come from the 'prev' iteration
                    // We subtract -1 because this is one iteration before 
                    // Store the result into the map of the cloned
                    InstToPreheader[ib] = copyLoopBodyToHeader(ib, subs.getInductionVar(), preheader, p-1);
                }
            }

            // ++++++++ Loop body part +++++++++
            // For each of the cloned increment the indexs if needed and place the PHINode.
            for (BasicBlock::iterator ib = (*it)->begin(), eb = (*it)->end(); ib!=eb; ++ib) {
                // If this is NOT a phi node
                if (!dyn_cast<PHINode>(ib)) {
                    unsigned int p = ip.getPriority(ib);

                    // If this variable is not dependent on i (not i:=i+1)
                    // then we need to replace each i to i+5 ...
                    // We also do not need to create a PHI node, etc.
                    if (!subs.isUsedByInductionVariable(ib)) {
                        
                        incrementInductionVarIfUsed(ib,subs.getInductionVar(),p);

                        // Create the new PHI Node to replace the node
                        if (!dyn_cast<StoreInst>(ib) && !ib->isTerminator()) {
                            std::string newname = "glue" + (*it)->getNameStr();

                            //PHINode* np = PHINode::Create(ib->getType(), "glue", *it);
                            PHINode* np = PHINode::Create(ib->getType(), newname, *it);
                            ib->replaceAllUsesWith(np);
                            np->reserveOperandSpace(2);
                            np->addIncoming(InstToPreheader[ib], preheader);
                            np->addIncoming(ib, *it);
                            np->moveBefore((*it)->begin());
                        }

                    }// end of if this is not an IV node (i:=i+1) 
                }
            }
        }

        eliminateDuplicatedLoads(preheader);
        for (Loop::block_iterator it=IncomingLoop->block_begin(); it!=IncomingLoop->block_end();++it) {
            eliminateDuplicatedLoads(*it);
            for (BasicBlock::iterator in = (*it)->begin(); in != (*it)->end(); ++in) {
                foldAddInstructions(in);
            }
        }
        return true;
    }

    void ModuloSchedulerDriverPass::incrementInductionVarIfUsed(Instruction* inst, Instruction* ind, int offset) {
        // For each of the operands of our instruction
        for (Instruction::op_iterator op = inst->op_begin(); op!= inst->op_end(); ++op) {
            // if our instruction uses 'i' (the induction var)
            if (*op == ind) {
                // Add an offset to the induction variable
                Instruction *add = subscripts::incrementValue(*op, offset, inst);
                // Now our instruction depends on the new offset induction variable ([i+4]).
                *op = add;
            }
        }
    }

   void ModuloSchedulerDriverPass::duplicateValuesWithMultipleUses(BasicBlock* bb, Instruction* ind) {

       // While we keep duplicating nodes (and create more possible work), keep going
       bool keep_going = false; 
       do {
           keep_going = false; 
           // For each instruction in this BB
           for (BasicBlock::iterator it = bb->begin(); it!= bb->end(); ++it) {
               // if it is not the induction variable and it has more than one use
               if ((!dyn_cast<PHINode>(it)) &&  // Do not clone PHINodes
                       (ind != it) &&  // Do not clone induction pointer
                       // Only clone when you have more than one #uses
                       (instructionPriority::getLocalUses(it,bb) >1)) {

                   Instruction* cloned = it->clone(); // duplicate it
                    it->getParent()->getInstList().insert(it, cloned);
                    //Can also do: cloned->insertBefore(it); // on newer LLVMS
                    cloned->setName("cloned");
                    instructionPriority::replaceFirstUseOfWith(it, cloned);
                    // we may have created potential candidates for duplication. 
                    // you have to keep going
                   keep_going = true; 
               }
           } // foe rach inst
       } while (keep_going);
   }

   std::vector<Value*> ModuloSchedulerDriverPass::getAllArrayAccess(BasicBlock* bb, bool store) {
       std::vector<Value*> arrayBases;
       // for each instruction
       for (BasicBlock::iterator it = bb->begin(); it!= bb->end(); ++it) {
           Instruction* inst = it;
           // If we calculate an address 
           if (dyn_cast<GetElementPtrInst>(inst)) {
               Value* arrayBase = inst->getOperand(0); // This is the array base we access
               // For each of the users of this address (I assume all users are in this BB)
               for (Instruction::use_iterator uit = inst->use_begin(); uit!= inst->use_end(); ++uit) {
                   // If this is STORE or LOAD (and we are asked for S or L)
                   if (dyn_cast<StoreInst>(uit) && store || dyn_cast<LoadInst>(uit) && !store)  {
                       //if (store)  cerr<<"Store Found "<<*inst<<**uit;
                       //if (!store) cerr<<"Load  Found "<<*inst<<**uit;
                       arrayBases.push_back(arrayBase); 
                   }
               } 
           }
       }
       return arrayBases;
   }

   void ModuloSchedulerDriverPass::eliminateDuplicatedLoads(BasicBlock* bb) {
       bool found;
       do {
           found = false;
           for (BasicBlock::iterator it0 = bb->begin(); it0 != bb->end(); ++it0) {
               for (BasicBlock::iterator it1 = it0; it1 != bb->end(); ++it1) {
                   if (it0 != it1 && areInstructionsIdentical(it0,it1)) {
                       it1->replaceAllUsesWith(it0);
                       it1->eraseFromParent();
                       found = true;
                       break;
                   }
               }
           }
       } while (found == true);
   }

  bool ModuloSchedulerDriverPass::areInstructionsIdentical(Instruction* i1, Instruction* i2) {
        // Compare PHINodes
        if (i1 == i2) return false; // Do nothing if they are the same

        if ( dyn_cast<PHINode>(i1) && dyn_cast<PHINode>(i2)) {
             PHINode *p1 =  dyn_cast<PHINode>(i1);
             PHINode *p2 =  dyn_cast<PHINode>(i2);
             unsigned int incoming = p1->getNumIncomingValues();

             // If the number of incoming edges is different
             if (p2->getNumIncomingValues() != incoming) return false;

            // For each edge
            for (unsigned int i=0; i< incoming; i++) {
                // Compare incoming values and incoming blocks
                if (p1->getIncomingValue(i) != p2->getIncomingValue(i)) return false;
                if (p1->getIncomingBlock(i) != p2->getIncomingBlock(i)) return false;
            }
            // PHINodes are identical!!!
            return true; 
        }

        if (i1->getOpcode() == i2->getOpcode()) {
            unsigned int operands = i1->getNumOperands();
            if (i2->getNumOperands() != operands) return false; //Different number of operands
            for (unsigned int i=0; i<operands; i++) {
                if (i1->getOperand(i) != i2->getOperand(i)) return false;
            } 
           
            return true;

        } else return false;
        
  }

  void ModuloSchedulerDriverPass::foldAddInstructions(Instruction* add) {
      if (dyn_cast<BinaryOperator>(add) && add->getOpcode() == Instruction::Add) {
            BinaryOperator *bin = dyn_cast<BinaryOperator>(add);
            unsigned int bitWidth = cast<IntegerType>(bin->getType())->getBitWidth();

            Value *p0 = add->getOperand(0);//param1
            Value *p1 = add->getOperand(1);//param0
            if (dyn_cast<ConstantInt>(p0) && dyn_cast<ConstantInt>(p1)) {
                    ConstantInt *c0 = dyn_cast<ConstantInt>(p0);
                    ConstantInt *c1 = dyn_cast<ConstantInt>(p1);

                    //TODO:May overflow
                    unsigned int val = (c0->getValue().getZExtValue() + c1->getValue().getZExtValue());
                    ConstantInt *ncon =
                      ConstantInt::get(IntegerType::get(add->getContext(), bitWidth), val);
                    add->replaceAllUsesWith(ncon);
                    add->eraseFromParent();
                    return;
            }
            
            if (ConstantInt *c0 = dyn_cast<ConstantInt>(p0)) {
                    if (0 == c0->getValue().getZExtValue()) {
                         add->replaceAllUsesWith(p1);
                         add->eraseFromParent();
                         return;
                    }
            } 
            if (ConstantInt *c1 = dyn_cast<ConstantInt>(p1)) {
                    if (0 == c1->getValue().getZExtValue()) {
                        add->replaceAllUsesWith(p0);
                        add->eraseFromParent();
                        return;
                    }
            }
      }
  }


   char ModuloSchedulerDriverPass::ID = 0;
   RegisterPass<ModuloSchedulerDriverPass> XMSX("ms", "Run the modulo scheduler");

} //namespace


