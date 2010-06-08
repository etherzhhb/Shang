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
#ifndef LLVM_LIST_SCHED_H
#define LLVM_LIST_SCHED_H

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

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <set>

#include "vbe/utils.h"
#include "vbe/ResourceConfig.h"
#include "abstractHWOpcode.h"



using namespace llvm;

using std::vector;
using std::pair;
using std::string;
using std::map;

namespace xVerilog {

    /*
     * Represents a hardware execution unit (such as an instance of an ALU
     *  on a processor)
     */        
    class resourceUnit {
        public:
            /*
             * C'tor
             */
            resourceUnit(string name, unsigned int id, unsigned int streamNum = 2);

            /*
             * @return the length of this resource unit in cycles which are scheduled
             */
            unsigned int length();

            /*
             * @return True if instruction inst is scheduled to run in this 
             *  resource unit
             */
            bool hasInstruction(Instruction* inst);
            /*
             * @return all of the instructions for one given cycle
             */
            InstructionCycle getInstructionForCycle(unsigned int cycleNum);
            /*
             * @return True if opcode 'op' is executable on this instruction unit 
             */
            bool isSameUnitType(abstractHWOpcode* op) { return (op->getName() == this->getName()); }   
            /*
             * @return name of unit
             */
            string getName() {return m_name;}
            /*
             * @return id of unit
             */
            unsigned int getId() {return m_id;}
            /*
             * @return the best possible possition to schedule 'op' in this
             *  instruction unit.
             */
            unsigned int getBestSchedulingCycle(abstractHWOpcode* op);
            /*
             * Place the abstract opcode in possition 'place'
             */
            void place(abstractHWOpcode* op, unsigned int place);
            /*
             * @return a printable ascii image for debug
             */
            string toString();

            /** 
             * @return returns the number of cycles which are already assigned
             */
            unsigned int takenSlots();

            /** 
             * @brief select the resource with the fewest number of resources scheduled to it. 
             * This makes sure the different requests are spread evenly and 
             * it creates less pressure on the MUXs.
             * 
             * @param availableResources the available resources
             * 
             * @return the resource unit which is the smallest. Null if nothing is available
             */
            static resourceUnit* getLeastBusyResource(vector<resourceUnit*> availableResources);
        private:

            /** 
             * @brief Enlarge the resource table by 'count' cycles to make room
             * for scheduling of more instructions. 
             * 
             * @param newSize the new size of the table
             */
            void enlargeResourceTable(unsigned int newSize);

            /** 
             * @return are there any uOp instructions on a stream in a cycle
             */
            bool emptyAt(unsigned int streamID, unsigned int cycle);
            /*
             * @return the length of this resource unit in cycles which are scheduled
             * @param the stream who's length we want
             */
            unsigned int length(unsigned int streamID);

            /** 
             * @return the best possible possition to schedule 'op' in the 
             * given seq (seq_in or seq_out)
             */
            static unsigned int getBestSchedulingCycle(abstractHWOpcode* op, InstructionSequence &seq);

            /// name of hardware unit
            string m_name;
            /// the id of this unit, used for placing opcodes
            unsigned int m_id;
            /// storage for the instructions
            vector<InstructionSequence> m_seq;
    };

    typedef map<std::string, unsigned int> MemportMap;

    /*
     *The class which schedules the hardware opcodes in their 
     * different locations. This class is the main entry point for the
     * scheduling.
     */
    class listScheduler {
        public:
            /*
             *C'tor list scheduler
             */
            listScheduler(BasicBlock* BB,TargetData* TD); //JAWAD
            /*
             * @return the BasicBlock that we are scheduling
             *
             */
            BasicBlock* getBB() {return m_bb;}
            /*
             * @return vector<Instruction*> instructions for a given cycle.
             */
            vector<Instruction*> getInstructionForCycle(unsigned int cycleNum);
            /*
             * @return vector<assignPartEntry*> all assign parts of this basic block
             */
            vector<assignPartEntry*> getAssignParts();
            /*
             * @returns the num of cycles for this BasicBlock schedule
             */
            unsigned int length();
            /*
             * @return the id of the resourceUnit which instruction inst is
             *  scheduled in
             */
            unsigned int getResourceIdForInstruction(Instruction* inst);

            /*
            * Populate the scheduler data structure with micro-commands
            *  this may destroy the BasicBlock. (It will replace well formed
            *  instructions with meaningless loads and stores to virtual
            *  registers.)
            */
            void scheduleBasicBlock(BasicBlock* BB, GVRegistry *GVR);
	    InstructionVector skipped_instructions;
	    TargetData* TD;
        private:
            /*
             * Add resources to the list of units where we schedule instructions.
             */
            void addResource(string name, unsigned int count);


            /** 
             * 
             * @param resourceName name of resource we want to evaluate 
             * 
             * @return the max usage of a certain resource at a single clock. 
             * (Ex: How many mults do we use at the same time)
             */
            unsigned int getMaxResourceUsage(std::string resourceName);

            
            /// Saves the different virtual execution units of the
            // processor
            vector<resourceUnit*> m_units;
            /// name of BasicBlock in printable form
            BasicBlock* m_bb;
            /// lists all of the abstractHWOpcodes which were scheduled
            vector<abstractHWOpcode*> m_ops;
    }; //class

    typedef vector<listScheduler*> ListSchedVector;


class ListSchedDriver : public FunctionPass {
  ListSchedVector ListScheders;

  void clear();

public:
  static char ID;
  explicit ListSchedDriver() : FunctionPass(&ID) {}
  ~ListSchedDriver() { clear(); }

  virtual void getAnalysisUsage(AnalysisUsage &AU) const;
  virtual void releaseMemory() { clear(); }
  bool runOnFunction(Function &F);

  const ListSchedVector &getSchedulers() const { return ListScheders; }

  typedef ListSchedVector::iterator iterator;
  typedef ListSchedVector::const_iterator const_iterator;
  iterator begin() { return ListScheders.begin(); }
  iterator end()   { return ListScheders.end(); }
  const_iterator begin() const { return ListScheders.begin(); }
  const_iterator end()   const { return ListScheders.end(); }

  // Get the total states of the function.
  unsigned getNumStates() const {
    unsigned NumStates = 0;
    for (const_iterator I = begin(), E = end(); I != E; ++I)
      NumStates += (*I)->length();
    
    // Note: We have an "Idle" state
    ++NumStates;

    return NumStates;
  }

  unsigned getStatesBitWidth() const {
    unsigned int numberOfStates = getNumStates();
    return Log2_32_Ceil(numberOfStates+1);
  }

  void getRegisters(InstructionVector &Insts) const; 
};
} //end of namespace
#endif // h guard
