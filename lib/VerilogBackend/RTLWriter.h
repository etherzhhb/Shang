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

#include "VLang.h"
#include "vbe/utils.h"
#include "vbe/ResourceConfig.h"

using namespace llvm;


namespace esyn {
    
    class RTLWriter;

    ///*
    // * This class creates the string which represents the assign part in verilog. This is 
    // *the mux which wraps each of the execution units. 
    // */
    //class assignPartBuilder {
    //    public:
    //        assignPartBuilder(const string& name, const string& op): m_name(name),m_op(op) {}

    //        void addPart(assignPartEntry* part) {m_parts.push_back(part);}
    //        /*
    //         * @returns a string with the verilog representation of the assign part added
    //         */
    //        string toString(RTLWriter* abop);

    //    private:
    //        string m_name;
    //        string m_op;
    //        vector<assignPartEntry*> m_parts;

    //};

//class RTLWriter {
//  TargetData* TD; //JAWAD
//  VLang &vlang;
//  /// The number of memory ports to render in this design
//  unsigned int m_memportNum;
//  unsigned int m_pointerSize;
//
//  public:
//    RTLWriter(VLang &v, TargetData* TD) : vlang(v), TD(TD) {
//    m_pointerSize = ResourceConfig::getResConfig("membus_size");
//    m_memportNum =  ResourceConfig::getResConfig("memport");
//  }
//
//  // print a value as either an expression or as a variable name
//  string evalValue(Value* val);
//
//  /// print all instructions which are inlineable
//  string printInlinedInstructions(Instruction* inst);
//
//  /// print list scheduler of a single BasicBlock
//  template<class StreamTy>
//  StreamTy &emitBBControl(StreamTy &ss, listScheduler *LS) {
//    string name = vlang.GetValueName(LS->getBB());
//    // for each cycle in this basic block
//    for (unsigned int cycle=0, max_cycle = LS->length();
//        cycle < max_cycle; ++cycle) {
//      vlang.emitCaseStateBegin(ss, name + utostr(cycle));
//      InstructionVector inst = LS->getInstructionForCycle(cycle);
//      // for each instruction in cycle, print it ...
//      for (InstructionVector::iterator I = inst.begin(), E = inst.end();
//          I != E; ++I) {
//        Instruction *Inst = *I;
//        unsigned int id = LS->getResourceIdForInstruction(Inst);
//        if (!isInstructionDatapath(Inst)) {
//          emitInstruction(ss, Inst, id);
//        }
//      }
//      // TODO: Do not compare
//      if (cycle + 1 != max_cycle)
//        vlang.indent(ss) << "eip <= " << name << cycle+1 <<";\n"; //header
//
//      vlang.emitEnd(ss);
//    }// for each cycle      
//
//    return ss;
//  }
//
//  template<class StreamTy>
//  StreamTy &emitBBDatapath(StreamTy &ss, listScheduler *LS) {
//    // for each cycle in this basic block
//    for (unsigned int cycle=0, max_cycle = LS->length();
//        cycle < max_cycle; ++cycle) {
//      std::vector<Instruction*> inst = LS->getInstructionForCycle(cycle);
//      // for each instruction in cycle, print it ...
//      for (InstructionVector::iterator I = inst.begin(), E = inst.end();
//          I != E; ++I) {
//        if (isInstructionDatapath(*I))
//          emitInstruction(ss, *I, 0);
//      }
//    }// for each cycle
//    return ss;
//  }
//
//  template<class StreamTy>
//  StreamTy &emitStoreInst(StreamTy &ss, StoreInst *St,
//                          int unitNum, int cycleNum) {
//    vlang.indent(ss)  << evalValue(St->getOperand(1)) <<unitNum<< " <= "
//                      << evalValue(St->getOperand(0)) << ";\n";
//    return ss;
//  }
//
//  template<class StreamTy>
//  StreamTy &emitLoadInst(StreamTy &ss, LoadInst *Ld,
//                          int unitNum, int cycleNum) {
//    /*
//    * If this is a regular load/store command then we just print it
//    * however, if this is a memory port then we need to assign a port
//    * number to it
//    * */
//    vlang.indent(ss) << vlang.GetValueName(Ld) << " <= "
//                     << evalValue(Ld->getOperand(0)) << unitNum << ";\n";
//    return ss;
//  }
//
//  string printBinaryOperatorInst(Instruction* inst, int unitNum, int cycleNum);
//
//  template<class StreamTy>
//  StreamTy &emitReturnInst(StreamTy &ss, ReturnInst *Ret) {
//    if (Ret->getNumOperands())
//      vlang.indent(ss) << "return_value <= "
//                       << evalValue(Ret->getOperand(0))<<";\n";
//    // We are ready!
//    vlang.indent(ss) << "rdy <= 1;\n";
//    // And next state is idle.
//    vlang.indent(ss) << "eip <= state_idle;\n";
//    return ss;
//  }
//
//  template<class StreamTy>
//  StreamTy &emitSelectInst(StreamTy &ss, SelectInst *Sel) {
//    vlang.indent(ss) << vlang.GetValueName(Sel) <<" <= "
//                     << "(" << evalValue(Sel->getOperand(0)) << " ? "
//                     << evalValue(Sel->getOperand(1)) << " : "
//                     << evalValue(Sel->getOperand(2))<<")";
//    return ss;
//  }
//
//  string printZxtInst(Instruction* inst);
//  string printBitCastInst(Instruction* inst); //JAWAD
//
//  string printIntToPtrInst(Instruction* inst);
//
//  string printAllocaInst(Instruction* inst);
//  
//  template<class StreamTy>
//  StreamTy &emitPHICopiesForSucc(StreamTy &ss, 
//                                 BasicBlock *CurBlock,BasicBlock *Succ) {
//    Instruction *NotPhi = Succ->getFirstNonPHI();
//    BasicBlock::iterator I = Succ->begin();
//    while (&*I != NotPhi) {
//      PHINode *PN = cast<PHINode>(I);
//      Value *IV = PN->getIncomingValueForBlock(CurBlock);
//      vlang.indent(ss) << vlang.GetValueName(I)
//                       << " <= " << evalValue(IV) << ";\n";
//      ++I;
//    }
//    return ss;
//  }
//
//  template<class StreamTy>
//  StreamTy &emitBranchInst(StreamTy &ss, BranchInst *Br) {
//    if (Br->isConditional()) {
//      vlang.emitIfBegin(ss, evalValue(Br->getCondition()));
//      emitPHICopiesForSucc(ss, Br->getParent(), Br->getSuccessor(0));
//      // we add a zero because the first entry in the basic block is '0'
//      // i.e we jump to the first state in the basic block
//      vlang.indent(ss) << "eip <= "
//        << vlang.GetValueName(Br->getSuccessor(0))<<"0;\n";
//      vlang.emitIfElse(ss);
//      emitPHICopiesForSucc(ss, Br->getParent(), Br->getSuccessor(1));
//      // we add a zero because the first entry in the basic block is '0'
//      vlang.indent(ss) << "eip <= "
//        <<vlang.GetValueName(Br->getSuccessor(1))<<"0;\n";
//      vlang.emitEnd(ss);
//    } else {
//      emitPHICopiesForSucc(ss, Br->getParent(), Br->getSuccessor(0));
//      vlang.indent(ss) << "eip <= " 
//        << vlang.GetValueName(Br->getSuccessor(0))<<"0;\n";
//    }
//    return ss;
//  }
//
//  template<class StreamTy>
//  StreamTy &emitICmpInst(StreamTy &ss, ICmpInst *ICmp) {
//    vlang.indent(ss) << vlang.GetValueName(ICmp) << " <= (" 
//                     << evalValue(ICmp->getOperand(0));
//
//    switch (ICmp->getPredicate()) {
//      case ICmpInst::ICMP_EQ:  ss << " == "; break;
//      case ICmpInst::ICMP_NE:  ss << " != "; break;
//      case ICmpInst::ICMP_ULE:
//      case ICmpInst::ICMP_SLE: ss << " <= "; break;
//      case ICmpInst::ICMP_UGE:
//      case ICmpInst::ICMP_SGE: ss << " >= "; break;
//      case ICmpInst::ICMP_ULT:
//      case ICmpInst::ICMP_SLT: ss << " < "; break;
//      case ICmpInst::ICMP_UGT:
//      case ICmpInst::ICMP_SGT: ss << " > "; break;
//      default: ss << "Unknown icmppredicate";
//    }
//
//    ss << evalValue(ICmp->getOperand(1));
//    ss << ");\n";
//    return ss;
//  }
//
//  string printPHINode(Instruction* inst); 
//  string getGetElementPtrInst(Instruction* inst);
//  string printGetElementPtrInst(Instruction* inst);
//
//  bool isInstructionDatapath(Instruction *inst);
//
//  template<class StreamTy>
//  StreamTy &emitInstruction(StreamTy &ss, Instruction *Inst,
//                            unsigned int resId) {
//    switch (Inst->getOpcode()) {
//    case Instruction::Store:
//      return emitStoreInst(ss, cast<StoreInst>(Inst), resId, 0);
//    case Instruction::Load:
//      return emitLoadInst(ss, cast<LoadInst>(Inst), resId , 0);
//    case Instruction::Ret:
//      return emitReturnInst(ss, cast<ReturnInst>(Inst));
//    case Instruction::Br:
//      return emitBranchInst(ss, cast<BranchInst>(Inst));
//    case Instruction::PHI:
//      return vlang.emitCommentBegin(ss) << "Ignore Phi Node";
//    case Instruction::Add:
//    case Instruction::Sub:
//    case Instruction::SRem:
//    case Instruction::And:
//    case Instruction::Or:
//    case Instruction::Xor:
//    case Instruction::Mul:
//    case Instruction::AShr:
//    case Instruction::LShr:
//    case Instruction::Shl:
//      return vlang.indent(ss) << printBinaryOperatorInst(Inst, 0, 0) << ";\n";
//    case Instruction::ICmp:
//      return emitICmpInst(ss, cast<ICmpInst>(Inst));
//    case Instruction::GetElementPtr:
//      return vlang.indent(ss) <<
//        printGetElementPtrInst(cast<GetElementPtrInst>(Inst)) << ";\n";
//    case Instruction::Select:
//      return emitSelectInst(ss, cast<SelectInst>(Inst)) << ";\n";
//    case Instruction::ZExt:
//      return vlang.indent(ss) << printZxtInst(cast<ZExtInst>(Inst)) << ";\n";
//    case Instruction::BitCast:
//      return vlang.indent(ss) << printBitCastInst(cast<BitCastInst>(Inst)) << ";\n";
//    case Instruction::Alloca:
//      return vlang.indent(ss) << printAllocaInst(cast<AllocaInst>(Inst)) << ";\n";
//    case Instruction::IntToPtr:
//      return vlang.indent(ss) << printIntToPtrInst(cast<IntToPtrInst>(Inst)) << ";\n";
//    case Instruction::Call:
//      return vlang.indent(ss) << printIntrinsic(cast<CallInst>(Inst)) << ";\n";
//
//    default:
//      return vlang.emitCommentBegin(ss) << "Unsupport inst: " << *Inst;
//    }
//  }
//
//  string getTypeDecl(const Type *Ty, bool isSigned, const std::string &NameSoFar);
//
//  template<class StreamTy>
//  StreamTy &emitRegistersDecl(StreamTy &ss, const ListSchedDriver &LSD) {
//    InstructionVector TempRegs;
//    LSD.getRegisters(TempRegs);
//
//    vlang.emitCommentBegin(ss) << "Number of Registers:" << TempRegs.size() << "\n"; 
//    for (InstructionVector::iterator I = TempRegs.begin(), E = TempRegs.end();
//        I != E; ++I) {
//      Instruction *Inst = *I;
//      // Emit the register name.
//      vlang.indent(ss) << vlang.printType(Inst->getType(), false,
//                                          vlang.GetValueName(Inst), "reg ");
//      
//      // Emit the initial value.
//      const IntegerType *Inty = dyn_cast<IntegerType>(Inst->getType());
//      if (Inty) 
//        ss << " = " << vlang.printConstantInt(0, Inty->getBitWidth(), false);
//      
//      ss << ";\n";
//    }
//    return ss;
//  }
//
//  template<class StreamTy>
//  StreamTy &emitResetAllRegs(StreamTy &ss, Function &F,
//                             const ListSchedDriver &LSD) {
//    vlang.emitCommentBegin(ss) << "Reset output registers\n";
//    for (Function::const_arg_iterator I = F.arg_begin(), E = F.arg_end();
//        I != E; ++I)
//        resetPtrRegs(ss, I, TD->getPointerSizeInBits(), m_pointerSize);
//    resetRetRegs(ss, F);
//    // reset rdy.
//    vlang.emitResetRegister(ss, "rdy", 1);
//    vlang.emitResetRegister(ss, "eip", LSD.getStatesBitWidth());
//
//    InstructionVector TempRegs;
//    LSD.getRegisters(TempRegs);
//
//    vlang.emitCommentBegin(ss) << "Reset local registers\n"; 
//    for (InstructionVector::iterator I = TempRegs.begin(), E = TempRegs.end();
//        I != E; ++I) {
//      Instruction *Inst = *I;
//      // Emit the initial value.
//      if (const IntegerType *Inty = dyn_cast<IntegerType>(Inst->getType()))
//        vlang.emitResetRegister(ss, vlang.GetValueName(Inst),
//                                Inty->getBitWidth());
//
//    }
//    return ss;
//  }
//
//  template<class StreamTy>
//  StreamTy &printPtrDecl(StreamTy &ss, const Argument *Arg, 
//                         unsigned DataWidth, unsigned BusWidth) {
//      std::string name = Arg->getNameStr();
//      const PointerType *PtrTy = dyn_cast<PointerType>(Arg->getType());
//      assert(PtrTy && "Arg is not ptr!");
//      // TODO: whats memport?
//      unsigned i = 0;
//      // Pointer size
//      vlang.indent(ss) << "input wire [" << (DataWidth-1) << ":0] mem_"
//        << name << "_out" << i <<",\n";
//
//      vlang.indent(ss) << "output reg [" << (DataWidth - 1) << ":0] mem_"
//        << name << "_in" << i << ",\n";
//
//      vlang.indent(ss) << "output reg [" << (BusWidth - 1) <<":0] mem_"
//        << name << "_addr"<< i << ",\n";
//
//      vlang.indent(ss) << "output reg mem_" << (name) << "_mode" << i;
//      return ss;
//  }
//
//  template<class StreamTy>
//  StreamTy &resetPtrRegs(StreamTy &ss, const Argument *Arg, 
//                         unsigned DataWidth, unsigned BusWidth) {
//    std::string name = Arg->getNameStr();
//    const PointerType *PtrTy = dyn_cast<PointerType>(Arg->getType());
//    // Do not need to reset.
//    if (!PtrTy)
//      return ss;
//
//    // TODO: whats memport?
//    unsigned i = 0;
//    // Pointer size
//    vlang.emitResetRegister(ss, "mem_" + name + "_in" + utostr(i), DataWidth);
//    vlang.emitResetRegister(ss, "mem_" + name + "_addr" + utostr(i), BusWidth);
//    vlang.emitResetRegister(ss, "mem_" + name + "_mode" + utostr(i), 1);
//    return ss;
//  }
//
//  template<class StreamTy>
//  StreamTy &resetRetRegs(StreamTy &ss, const Function &F) {
//    if (const IntegerType *RetTy = dyn_cast<IntegerType>(F.getReturnType()))
//      vlang.emitResetRegister(ss, "return_value", 0, RetTy->getBitWidth());
//    else
//      vlang.indent(ss) << "/*reset unknown return reg*/";
//    ss << "\n";
//    return ss;
//  }
//
//  template<class StreamTy>
//  StreamTy &emitStateDefs(StreamTy &ss, const ListSchedDriver &LSD) {
//    unsigned int numberOfStates = LSD.getNumStates();
//
//    // Instruction pointer of n bits, n^2 states
//    unsigned int NumOfStateBits = LSD.getStatesBitWidth();
//    // Emit states
//    vlang.emitCommentBegin(ss) << "Number of states:"<<numberOfStates<<"\n";
//    vlang.indent(ss) << "reg ["<< NumOfStateBits - 1<<":0] eip;\n";
//
//    // print the definitions for the values of the EIP values.
//    // for example: 'define start 16'd0  ...
//    // Use other encoding for states?
//    // Do not forget idle state
//    vlang.emitParam(ss, "state_idle", NumOfStateBits, 0);
//    // Start form 1 because the idle state is 0.
//    int stateCounter = 1;
//    for (ListSchedDriver::const_iterator I = LSD.begin(), E = LSD.end();
//        I != E; ++I) {
//      listScheduler *ls = *I;
//      // each cycle in the BB
//      for (unsigned int i=0, e = ls->length();i != e;++i) {
//        vlang.emitParam(ss, vlang.GetValueName(ls->getBB()) + utostr(i),
//                        NumOfStateBits, stateCounter);
//        ++stateCounter;
//      }
//    }
//    ss<<"\n";
//    return ss;
//  }
//
//  template<class StreamTy>
//  StreamTy &emitAssignPart(StreamTy &ss, vector<assignPartEntry*> ass) {
//    vlang.emitCommentBegin(ss) << "Assign part ("<< ass.size() <<")\n";
//    std::map<string,string> unitNames;
//
//    // extract all unit names from assign part
//    for (vector<assignPartEntry*>::iterator it = ass.begin(); it!=ass.end(); ++it)
//      unitNames[(*it)->getUnitName()] = (*it)->getUnitType();
//
//    // for each uniqe name 
//    for (map<string,string>::iterator nm = unitNames.begin(); nm!=unitNames.end(); ++nm) {
//      assignPartBuilder apb(nm->first, nm->second);
//      // for all assign parts with this name
//      for (vector<assignPartEntry*>::iterator it = ass.begin(); it!=ass.end(); ++it) {
//        if (nm->first==(*it)->getUnitName()) {
//          apb.addPart(*it);
//        }
//      }
//      vlang.indent(ss) << apb.toString(this);
//    }
//    ss << "\n\n";
//    return ss;
//  }
//  
//  template<class StreamTy>
//  StreamTy &emitAssignPart(StreamTy &ss, const ListSchedDriver &LSD) {
//    std::vector<assignPartEntry*> parts;
//    for (ListSchedDriver::const_iterator I = LSD.begin(), E = LSD.end();
//        I != E; ++I) {
//      vector<assignPartEntry*> p = (*I)->getAssignParts();
//      parts.insert(parts.begin(),p.begin(),p.end());
//    }
//    return emitAssignPart(ss, parts);
//  }
//
//  string getIntrinsic(Instruction* inst);
//  string printIntrinsic(Instruction* inst);
//
//  template<class StreamTy>
//  StreamTy &emitModuleDecl(StreamTy &ss, const Function *F) {
//    vlang.emitModuleBegin(ss, F->getNameStr());
//
//    vlang.indent(ss) << "input wire " << "clk" << ",\n";
//    vlang.indent(ss) << "input wire " << "rstN" << ",\n";
//    vlang.indent(ss) << "input wire " << "start" << ",\n";
//
//    const AttrListPtr &PAL = F->getAttributes();
//    unsigned Idx = 1;
//    for (Function::const_arg_iterator I = F->arg_begin(), E = F->arg_end();
//      I != E; ++I) {
//        // integers:
//        const Type *ArgTy = I->getType();
//        if (ArgTy->isPointerTy()) {
//          printPtrDecl(ss, I, TD->getPointerSizeInBits(),
//            m_pointerSize) << ",\n";
//        } else if(ArgTy->isIntegerTy()) {
//          vlang.indent(ss) <<
//            VLang::printType(ArgTy,
//            /*isSigned=*/PAL.paramHasAttr(Idx, Attribute::SExt),
//            vlang.GetValueName(I), "wire ", "input ") << ",\n";
//          ++Idx;
//        } else {
//          vlang.indent(ss) << "/*unsupport*type/\n";
//        }
//    }
//
//    // Ready signal
//
//    vlang.indent(ss) << "output reg " << "rdy";
//    const Type *RetTy = F->getReturnType();
//    if (RetTy->isVoidTy()) {
//      // Do something?
//      //vlang.indent(ss) << "/*return void*/";
//    } else {
//      // End rdy declare.
//      ss << ",\n";
//      assert(RetTy->isIntegerTy() && "Only support return integer now!");
//      vlang.indent(ss) << VLang::printType(RetTy, false, "return_value", "reg ", "output ");
//    }
//    vlang.emitEndModuleDecl(ss);
//    return ss;
//  }
//
//  string getBRAMDefinition(unsigned int wordBits, unsigned int addressBits);
//
//  string createBinOpModule(string opName, string symbol, unsigned int stages);
//};//class
} //end of namespace
#endif // h guard
