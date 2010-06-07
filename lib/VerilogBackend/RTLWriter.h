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
#ifndef VBE_RTL_WRITER_H
#define VBE_RTL_WRITER_H

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

#include "llvm/Target/Mangler.h"

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <set>

#include "listScheduler.h"
#include "VLang.h"
#include "vbe/utils.h"
#include "vbe/ResourceConfig.h"

using namespace llvm;

using std::vector;
using std::pair;
using std::string;
using std::stringstream;

namespace xVerilog {
    
    class RTLWriter;

    /*
     * This class creates the string which represents the assign part in verilog. This is 
     *the mux which wraps each of the execution units. 
     */
    class assignPartBuilder {
        public:
            assignPartBuilder(const string& name, const string& op): m_name(name),m_op(op) {}

            void addPart(assignPartEntry* part) {m_parts.push_back(part);}
            /*
             * @returns a string with the verilog representation of the assign part added
             */
            string toString(RTLWriter* abop);

        private:
            string m_name;
            string m_op;
            vector<assignPartEntry*> m_parts;

    };

class RTLWriter {
  TargetData* TD; //JAWAD
  VLang &vlang;
  /// The number of memory ports to render in this design
  unsigned int m_memportNum;
  unsigned int m_pointerSize;

  public:
    RTLWriter(VLang &v, TargetData* TD) : vlang(v), TD(TD) {
    m_pointerSize = ResourceConfig::getResConfig("membus_size");
    m_memportNum =  ResourceConfig::getResConfig("memport");
  }

  // print a value as either an expression or as a variable name
  string evalValue(Value* val);

  /// print all instructions which are inlineable
  string printInlinedInstructions(Instruction* inst);

  /// print list scheduler of a single BasicBlock
  string printBasicBlockControl(listScheduler *ls);
  string printBasicBlockDatapath(listScheduler *ls);

  string printStoreInst(Instruction* inst, int unitNum, int cycleNum);

  string printLoadInst(Instruction* inst, int unitNum, int cycleNum);

  string printBinaryOperatorInst(Instruction* inst, int unitNum, int cycleNum);

  string printReturnInst(Instruction* inst);
  string printSelectInst(Instruction* inst);

  string printZxtInst(Instruction* inst);
  string printBitCastInst(Instruction* inst); //JAWAD

  string printIntToPtrInst(Instruction* inst);

  string printAllocaInst(Instruction* inst);

  string printPHICopiesForSuccessor(BasicBlock *CurBlock,BasicBlock *Successor);

  string printBranchInst(Instruction* inst);

  string printCmpInst(Instruction* inst); 

  string printPHINode(Instruction* inst); 
  string getGetElementPtrInst(Instruction* inst);
  string printGetElementPtrInst(Instruction* inst);

  bool isInstructionDatapath(Instruction *inst);

  string printInstruction(Instruction *inst, unsigned int resourceId);

  string getTypeDecl(const Type *Ty, bool isSigned, const std::string &NameSoFar);

  string getFunctionLocalVariables(listSchedulerVector lsv);
  unsigned int getNumberOfStates(listSchedulerVector &lsv);
  string getStateDefs(listSchedulerVector &lsv);
  string printAssignPart(vector<assignPartEntry*> ass, RTLWriter* lang);
  string getAssignmentString(listSchedulerVector lv);
  string getIntrinsic(Instruction* inst);
  string printIntrinsic(Instruction* inst);

  string getFunctionSignature(const Function *F);

  string getBRAMDefinition(unsigned int wordBits, unsigned int addressBits);

  string createBinOpModule(string opName, string symbol, unsigned int stages);
};//class
} //end of namespace
#endif // h guard
