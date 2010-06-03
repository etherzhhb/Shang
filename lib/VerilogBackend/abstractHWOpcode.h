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
#ifndef LLVM_ABSTRACT_HW_OP_H
#define LLVM_ABSTRACT_HW_OP_H

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
#include "llvm/Target/TargetData.h" //JAWAD

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <set>
#include <map>

#include "vbe/params.h"
#include "globalVarsRegistry.h"
using namespace llvm;

using std::set;
using std::map;
using std::vector;
using std::pair;
using std::string;

using llvm::TargetData; //JAWAD
typedef vector<llvm::Instruction*> InstructionVector;
typedef vector<llvm::Instruction*> InstructionCycle;
typedef vector<InstructionCycle> InstructionSequence; 

namespace xVerilog {

    /* A class which represents a usage of a hardware module. For example,
     * connecting two registers to a multiplier unit. The inputs are the
     * two Value classes. You know which hardware unit you are assigned to by 
     * the hardware unit it is schedueld to. 
     */
    class assignPartEntry {
        public:
            assignPartEntry(Value* left, Value* right):
                m_left(left),m_right(right){}

            Value* getLeft(){return m_left;}
            Value* getRight(){return m_right;}

            /*
             * set the unit id and unit name of this assign part. The unit id is
             * the id of the execution unit and the name is the name of the unit.
             * Also, state name is needed so we can assign it to the right state.
             */
            void setUnit(unsigned int unitid, const string& name,
                    const string& stateName, unsigned int cycle);

            /*
             * @return the name of the unit, for example multiply1
             */
            string getUnitName();

            string getUnitType() {return m_unitName;}

            string getState(){return m_state;}

        private:
            unsigned int m_unitId;
            string m_unitName;
            string m_state;

            Value* m_left;
            Value* m_right;
    }; // class

    typedef std::pair<std::string,unsigned int> ArrayInfo;
    /*
     * Represents an original LLVM instruction which is broken down into 
     *  multiple hardware instructions.
     */
    class abstractHWOpcode {
        public:
            /*
             * C'tor
             * @param inst is the LLVM instruction to be represented in the abstract opcode
             *  this may modify the bitcode structure. It may add/remove and change the 
             *  dependencies of instructions. 
             *  @param streamNum the number of streams in this opcode.
             *  There are n streams. 
             *  We define multiple streams so that we can have instructions which use two pipelined 
             *  dependencies such as this one.
             *   --> time 
             *  resource 1a: ..AB...
             *  resource 1b: ...AB..
             */
            abstractHWOpcode(Instruction* inst, std::string stateName, unsigned int streamNum = 2,TargetData* TD = NULL); //JAWAD
            /*
             * A builder function used to define a hardware opcode using LLVM opcodes.
             *  Adds a cycle of instructions to the opcode. The cycle may be empty from any ops. 
             *  This is used in many pipelined ops where only the first cycle contain assignments
             *  while the rest of the cycles are simply delay ones. In other words, empty cycles 
             *  at the end of the opcode means that the data is in the pipeline and will be ready 
             *  as soon these cycles pass. 
             *
             */
            void appendInstructionCycle(InstructionCycle ops, unsigned int streamID) {
                    m_iv[streamID].push_back(ops);
              }
            /*
             * Append an instruction, which is not directly mapped to a uOp 
             *  to a list of instructions which ARE used in the calculation of
             *  dependencies between abstractHWOpcodes. (in the case of assign aprt)
             */
            void appendUsedInstruction(Instruction* inst) {m_usedInst.insert(inst);}

            InstructionCycle cycleAt(unsigned int streamID, unsigned int cycle) {
                if (streamID < m_iv.size())
                    if (cycle < m_iv[streamID].size())
                        return m_iv[streamID][cycle];
                InstructionCycle nop;
                return nop;
            }
            /** 
             * @return are there any uOp instructions on a stream in a cycle
             */
            bool emptyAt(unsigned int streamID, unsigned int cycle) {
                return (0 == cycleAt(streamID, cycle).size());
            }
            /*
             * returns the number of cycles that this instruction take. The
             * length of the longest stream.
             */
            unsigned int getLength();
            /*
             * returns the number of cycles that this instruction take. 
             */
            unsigned int getLength(unsigned int streamID) {return m_iv[streamID].size();}
            /*
             * Returns the hardware instructions at a given clock.
             */
            InstructionCycle getInstructionForCycle(int cycle);
            /*
             * Declare that this opcode depends on the result of another opcode
             */
            void addDependency(abstractHWOpcode* dep);
            /*
             * True if this opcode depend on the opcode 'dep'.
             */
            bool isDepends(abstractHWOpcode* dep);
            /*
             * @returns the index of the cycle which is good to schedule.
             *  scans all of the dependencies and tells the first instruction which does
             *  not depend on any of the other instruction. 
             */
            unsigned int getFirstSchedulableSlot();
            /*
             * Searches for the LLVM instruction 'inst' in this instruction group
             *  This is used for establishing the dependencies between the opcodes.
             */
            Instruction* hasInstruction(Instruction *inst);
            /*
             * 
             * Searches for the ANY of the instructions in this instruction group
             */
            Instruction* hasInstruction(std::vector<Instruction *> &insts);
            /*
             *@return the name of this instruction. 
             * For example:  'memport' if this is a memory access.
             */
            std::string getName() { return m_opcodeName;}
            /*
             * @return this hwop as a string 
             */
            string toString();
            /*
             * Add a dependency of this opcode on 'hwop' if it has an opcode
             *  which is a dependency of our instructions.
             */
            void addDependencyIfDepends(abstractHWOpcode *hwop);
            /*
             * Sets the starting cycle of this opcode in the scheduling table. 
             * The list scheduler uses this to place the opcode in the instruction table. 
             *  This is used to clculate the next available slot for the next opcode. 
             *  It also records to which instance of the execution units it is scheduled
             */
            void place(unsigned int cycle,unsigned int unitid);
            /*
             * returns the place where this opcode was scheduled. see place()
             */
            unsigned int getPlace() {return m_place;}
            /*
             * Returns the first free cycle after the ending offset of this 
             *  opcode in the shcheduling table.
             */
            unsigned int  getFirstAfter() { return m_place + getLength(); }
            /*
             * @returns the assign part of the opcode, NULL if not present
             */
            assignPartEntry* getAssignPart() {return m_assignPart;}
            /** 
             * @param inst the parameter to check
             * 
             * @return a static function to see if this operation can be implemented in
             * hardware using wires only (example shl by a constant)
             */
            static bool isInstructionOnlyWires(Instruction* inst);
            /** 
             * @brief Is this opcode must come last in BasicBlock ?
             * 
             * @return True if yes
             */
            bool isMustBeLastOpcode() { return m_mustBeLast; }

            /** 
             * @brief Finds the width in bits of this element type. May be a pointer, an integer or an array.
             * 
             * @param type Type
             * 
             * @return bit width
             */
            static const StructType* isPtrToStructType(const Value* Op); //JAWAD
	    llvm::TargetData* TD;	 //JAWAD
        private:
            /*
             * Add a dependency of this opcode on 'hwop' if it has an opcode
             *  which is a dependency of our instructions.
             *
             *  @param iv looks for dependencies only in the stream iv
             */
            void addDependencyIfDepends(abstractHWOpcode *hwop, InstructionSequence &iv);
            /*
             * Serves the C'tor in creating a binary operation, which uses assign part,
             * such as 'mul' or 'div'. Creates the needed uOps and instructions.
             */
            void addBinaryInstruction(Instruction* inst, string op, unsigned int delay);
            /** 
             * @brief This method returns all 'incoming' instruction from a BasicBlock
             *  we use this in order to see which values delay the branch instruction. 
             *  It scans all of the PHI nodes and returns Instructions from All BBs, not
             *  only this one. 
             * 
             * @param bb the BasicBlock we want to scan.
             * 
             * @return A vector of instructions.
             */
            static std::vector<Instruction*> getAllIncomingValuesFromPHIs(BasicBlock* bb);
            /** 
             * @brief Return all of the incoming values from the BasicBlocks which
             *  this branch instruction jumps to. 
             *  It uses getAllIncomingValuesFromPHIs.
             * 
             * @param br The BranchInstruction to follow
             * 
             * @return a vector of instruction pointers. 
             *  Same as getAllIncomingValuesFromPHIs
             */
            static std::vector<Instruction*> getAllIncomingValuesFromBranch(BranchInst* br);

            /** 
             * @brief Returns the name of the array we are accessing. This gets a pointer to Store/Load
             *  command and returns the 'name' of the array they are reading or writing to. 
             * 
             * @param inst The Store or Load instruction. 
             * 
             * @return the name of the array we are accessing. For example 'A' and Bitwidth 
             */
            ArrayInfo getVariableNameFromMemoryCommand(Instruction* inst); //JAWAD

            /// is this opcode only virtual and should not cause any delay ?
            bool m_empty;
            /// is this opcode a branch opcode which must come last?
            bool m_last;
            /// name of this opcode
            string m_opcodeName;
            /// it's place in the basic block, after scheduling
            unsigned int m_place;
            /// instructions which are used for dependency calculations
            /// these instructions are not mapped directly to uOps. 
            set<Instruction*> m_usedInst;
            /// dependencies it has on other opcodes
            set<abstractHWOpcode*> m_dependencies;
            /// assign part, if needed
            assignPartEntry* m_assignPart;
            /// name of BasicBlock/State this hw is scheduled at
            string m_stateName;
            /// the actual uOP instructions
            vector<InstructionSequence> m_iv;
            /// Is this abstractHWOpcode has to be last in BB ?
            bool m_mustBeLast;
    }; // class abstractHWOpcode


} //end of namespace
#endif // h guard
