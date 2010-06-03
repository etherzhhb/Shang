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
#include "llvm/DerivedTypes.h"
#include "llvm/Instructions.h"
#include "llvm/ADT/SmallString.h"
#include "llvm/Support/CallSite.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/InstIterator.h"
#include "llvm/Support/GetElementPtrTypeIterator.h"
#include "llvm/Support/MathExtras.h"

#include "RTLWriter.h"
#include "intrinsics.h"
#include <algorithm> //JAWAD

namespace xVerilog {

string assignPartBuilder::toString(RTLWriter* vl) {
stringstream sb;

sb << "wire [31:0] "<<m_name<<"_in_a"<<";\n";
sb << "wire [31:0] "<<m_name<<"_in_b"<<";\n";
sb <<" assign " <<m_name<<"_in_a"<<" = ";
for (vector<assignPartEntry*>::iterator it=m_parts.begin(); it!=m_parts.end();++it) {
assignPartEntry *part = *it;
sb <<"\n (eip == "<<part->getState()<<") ? "<<
vl->evalValue(part->getLeft())<<" :";
}   
sb <<"0;\n";

sb <<" assign " <<m_name<<"_in_b"<<" = ";
for (vector<assignPartEntry*>::iterator it=m_parts.begin(); it!=m_parts.end();++it) {
assignPartEntry *part = *it;
sb <<"\n (eip == "<<part->getState()<<") ? "<<
vl->evalValue(part->getRight())<<" :";
}   
sb <<"0;\n\n";

sb<<"wire [31:0] out_"<<m_name<<";\n";
sb<<m_op<<"  "<<m_name<<"_instance (.clk(clk), .a("<<
m_name<<"_in_a)"<<", .b("<<m_name<<"_in_b), .p(out_"<<m_name<<"));\n\n";
return sb.str();
}


/// Verilog printer below

string RTLWriter::printBasicBlockDatapath(listScheduler *ls) {
stringstream ss;
// for each cycle in this basic block
for (unsigned int cycle=0; cycle<ls->length();cycle++) {
vector<Instruction*> inst = ls->getInstructionForCycle(cycle);
// for each instruction in cycle, print it ...
for (vector<Instruction*>::iterator ii = inst.begin(); ii != inst.end(); ++ii) {
if (isInstructionDatapath(*ii)) {
ss<<printInstruction(*ii, 0);
}
}
}// for each cycle      
return ss.str();
}

string RTLWriter::printBasicBlockControl(listScheduler *ls) {
stringstream ss;
const string space("\t");
string name = toPrintable(ls->getBB()->getName());
// for each cycle in this basic block
for (unsigned int cycle=0; cycle<ls->length();cycle++) {
ss<<""<<name<<cycle<<":\n"; //header
ss<<"begin\n";
vector<Instruction*> inst = ls->getInstructionForCycle(cycle);
// for each instruction in cycle, print it ...
for (vector<Instruction*>::iterator ii = inst.begin(); ii != inst.end(); ++ii) {
unsigned int id = ls->getResourceIdForInstruction(*ii);
if (!isInstructionDatapath(*ii)) {
ss<<space<<printInstruction(*ii, id);
}
}

if (cycle+1 != ls->length()) { 
ss<<"\teip <= "<<name<<cycle+1<<";\n"; //header
}
ss<<"end\n";
}// for each cycle      

return ss.str();
}


string RTLWriter::printLoadInst(Instruction* inst, int unitNum, int cycleNum) {
LoadInst* load = (LoadInst*) inst; // make the cast
/*
* If this is a regular load/store command then we just print it
* however, if this is a memory port then we need to assign a port
* number to it
* */
stringstream ss;
ss<<vlang.GetValueName(load)<<" <= "<<evalValue(load->getOperand(0))<<unitNum;
return ss.str();
}


string RTLWriter::printStoreInst(Instruction* inst, int unitNum, int cycleNum) {
stringstream ss;
StoreInst* store = (StoreInst*) inst; // make the cast
string first = evalValue(store->getOperand(0));
string second = evalValue(store->getOperand(1)); 
ss << second<<unitNum<< " <= " << first;
return ss.str();
}



string RTLWriter::printBinaryOperatorInst(Instruction* inst, int unitNum, int cycleNum) {
stringstream ss;
BinaryOperator* bin = (BinaryOperator*) inst;

string rec = vlang.GetValueName(bin);
Value* val0 = bin->getOperand(0);
Value* val1 = bin->getOperand(1);
ss << rec <<" <= ";
// state, src1, src2, output
switch (bin->getOpcode()) {
case Instruction::Add:{ss<<evalValue(val0)<<"+"<<evalValue(val1);break;}
case Instruction::Sub:{ss<<evalValue(val0)<<"-"<<evalValue(val1);break;}
case Instruction::SRem:{ss<<evalValue(val0)<<"%"<<evalValue(val1);break;} // THIS IS SLOW
case Instruction::And:{ss<<evalValue(val0)<<"&"<<evalValue(val1);break;}
case Instruction::Or: {ss<<evalValue(val0)<<"|"<<evalValue(val1);break;}
case Instruction::Xor:{ss<<evalValue(val0)<<"^"<<evalValue(val1);break;}
case Instruction::Mul:{ss<<evalValue(val0)<<"*"<<evalValue(val1);break;}
case Instruction::AShr:{ss<<evalValue(val0)<<" >> "<<evalValue(val1);break;}
case Instruction::LShr:{ss<<evalValue(val0)<<" >> "<<evalValue(val1);break;}
case Instruction::Shl:{ss<<evalValue(val0)<<" << "<<evalValue(val1);break;}
default: {
errs()<<"Unhandaled: "<<*inst;
abort();
}
}
return ss.str();
}


string RTLWriter::printReturnInst(Instruction* inst) {
stringstream ss;
ReturnInst* ret = (ReturnInst*) inst; // make the cast
if (ret->getNumOperands()) {
ss << " rdy <= 1;\n";
ss << " return_value <= ";
ss << evalValue(ret->getOperand(0))<<";\n";
ss << " $display($time, \" Return (0x%x) \","<<evalValue(ret->getOperand(0))<<");";
ss << "\n $finish()";
} else  {
// if ret void
ss << " rdy <= 1;\n";
ss << " return_value <= 0;";
ss << "\n $finish()";
}
return ss.str();
}


string RTLWriter::printSelectInst(Instruction* inst) {
stringstream ss;
SelectInst* sel = (SelectInst*) inst; // make the cast
// (cond) ? i_b : _ib;
ss << vlang.GetValueName(sel) <<" <= ";
ss << "(" << evalValue(sel->getOperand(0)) << " ? ";
ss << evalValue(sel->getOperand(1)) << " : ";
ss << evalValue(sel->getOperand(2))<<")";
return ss.str();
}

string RTLWriter::printAllocaInst(Instruction* inst) {
stringstream ss;
AllocaInst* alca = (AllocaInst*) inst; // make the cast
// a <= b;
ss << vlang.GetValueName(alca) <<" <= 0";
ss << "/* AllocaInst hack, fix this by giving a stack address */"; 
return ss.str();
}

string RTLWriter::getIntrinsic(Instruction* inst) {
stringstream ss;
CallInst *cl = dyn_cast<CallInst>(inst);
assert(cl && "inst is not a CallInst Intrinsi");
vector<string> params; 
for(CallSite::arg_iterator I = cl->op_begin(); I != cl->op_end(); ++I) {
string s = evalValue(*I);
params.push_back(s); 
}
ss << getIntrinsicForInstruction(cl, params);
return ss.str();
}

string RTLWriter::printIntrinsic(Instruction* inst) {
return vlang.GetValueName(inst)  + string(" <= ") + getIntrinsic(inst);
}

string RTLWriter::printIntToPtrInst(Instruction* inst) {
stringstream ss;
IntToPtrInst* i2p = (IntToPtrInst*) inst; // make the cast
// a <= b;
ss << vlang.GetValueName(i2p) <<" <= ";
ss <<  evalValue(i2p->getOperand(0));
return ss.str();
}

string RTLWriter::printZxtInst(Instruction* inst) {
stringstream ss;
ZExtInst* zxt = (ZExtInst*) inst; // make the cast
// a <= b;
ss << vlang.GetValueName(zxt) <<" <= ";
ss <<  evalValue(zxt->getOperand(0));
return ss.str();
}
string RTLWriter::printBitCastInst(Instruction* inst) { //JAWAD
stringstream ss;
BitCastInst* btcst = (BitCastInst*) inst; // make the cast
// a <= b;
ss << vlang.GetValueName(btcst) <<" <= ";
ss <<  evalValue(btcst->getOperand(0));
return ss.str();
}

string RTLWriter::printPHICopiesForSuccessor(BasicBlock *CurBlock,BasicBlock *Successor){
stringstream ss;

for (BasicBlock::iterator I = Successor->begin(); isa<PHINode>(I); ++I) {
PHINode *PN = cast<PHINode>(I);
//Now we have to do the printing.
Value *IV = PN->getIncomingValueForBlock(CurBlock);
if (!isa<UndefValue>(IV)) {
ss <<"\t\t"<< vlang.GetValueName(I) << " <= " << evalValue(IV);
ss << ";\n";
}
}
return ss.str();
}

string RTLWriter::printBranchInst(Instruction* inst) {

stringstream ss;
BranchInst* branch = (BranchInst*) inst; // make the cast

if (branch->isConditional()) {
ss << "if (";
ss << evalValue(branch->getCondition());
ss << ") begin\n";
ss<<printPHICopiesForSuccessor(branch->getParent(),branch->getSuccessor(0));
// we add a zero because the first entry in the basic block is '0'
// i.e we jump to the first state in the basic block
ss << "\t\teip <= " << toPrintable(branch->getSuccessor(0)->getName())<<"0;\n";
ss << "\tend else begin\n";
ss<<printPHICopiesForSuccessor(branch->getParent(),branch->getSuccessor(1));
// we add a zero because the first entry in the basic block is '0'
ss << "\t\teip <= "<<toPrintable(branch->getSuccessor(1)->getName())<<"0;\n";
ss << "\tend\n";
} else {
ss<<printPHICopiesForSuccessor(branch->getParent(),branch->getSuccessor(0));
ss << "\t\teip <= " << toPrintable(branch->getSuccessor(0)->getName())<<"0;\n";
}
return ss.str();
}


string RTLWriter::printCmpInst(Instruction* inst) {
stringstream ss;
CmpInst* cmp = (CmpInst*) inst; // make the cast
ss << vlang.GetValueName(inst) << " <= ";
ss << "(";
ss<< evalValue(cmp->getOperand(0));

switch (cmp->getPredicate()) {
case ICmpInst::ICMP_EQ:  ss << " == "; break;
case ICmpInst::ICMP_NE:  ss << " != "; break;
case ICmpInst::ICMP_ULE:
case ICmpInst::ICMP_SLE: ss << " <= "; break;
case ICmpInst::ICMP_UGE:
case ICmpInst::ICMP_SGE: ss << " >= "; break;
case ICmpInst::ICMP_ULT:
case ICmpInst::ICMP_SLT: ss << " < "; break;
case ICmpInst::ICMP_UGT:
case ICmpInst::ICMP_SGT: ss << " > "; break;
default: errs() << "Invalid icmp predicate!"; abort();
}

ss << evalValue(cmp->getOperand(1));
ss << ")";
return ss.str();
}

string RTLWriter::getGetElementPtrInst(Instruction* inst) {
stringstream ss;
GetElementPtrInst* get = (GetElementPtrInst *) inst;
if (2 == get->getNumOperands()) {
// We have a regular array
ss << evalValue(get->getPointerOperand()); // + 
ss << " + ";
ss << evalValue(get->getOperand(get->getNumOperands()-1)); // get last dimention
} else {
// We have a struct or a multi dimentional array
// TODO: Assume we only have two elements. This is lame ... 
ss << evalValue(get->getPointerOperand()); // + 
ss << " + ";		//JAWAD
ss << evalValue(get->getOperand(get->getNumOperands()-2)); // JAWAD 
ss << " + 2*";
ss << evalValue(get->getOperand(get->getNumOperands()-1)); // get last dimention
ss << "/* Struct */";

}
return ss.str();
}

string RTLWriter::printGetElementPtrInst(Instruction* inst) {
stringstream ss;
ss << vlang.GetValueName(inst) <<" <= ";
ss<< getGetElementPtrInst(inst);
return ss.str();
}

bool RTLWriter::isInstructionDatapath(Instruction *inst) {
if (isa<BinaryOperator>(inst))     return true;
if (isa<CmpInst>(inst))            return true;
if (isa<GetElementPtrInst>(inst))  return true;
if (isa<SelectInst>(inst))         return true;
if (isa<ZExtInst>(inst))           return true;
if (isa<IntToPtrInst>(inst))       return true;
return false;
}
string RTLWriter::printInstruction(Instruction *inst, unsigned int resourceId) {
const string colon(";\n");
if (isa<StoreInst>(inst))      return printStoreInst(inst, resourceId, 0) + colon;
if (isa<LoadInst>(inst))       return printLoadInst(inst, resourceId , 0) + colon;
if (isa<ReturnInst>(inst))     return printReturnInst(inst) + colon;
if (isa<BranchInst>(inst))     return printBranchInst(inst);
if (isa<PHINode>(inst))        return "" ; // we do not print PHINodes 
if (isa<BinaryOperator>(inst)) return printBinaryOperatorInst(inst, 0, 0) + colon;
if (isa<CmpInst>(inst))        return printCmpInst(inst) + colon;
if (isa<GetElementPtrInst>(inst)) return printGetElementPtrInst(inst)+ colon;
if (isa<SelectInst>(inst))     return printSelectInst(inst) + colon;
if (isa<ZExtInst>(inst))       return printZxtInst(inst) + colon;
if (isa<BitCastInst>(inst))    return printBitCastInst(inst) + colon; //JAWAD
if (isa<AllocaInst>(inst))     return printAllocaInst(inst) + colon;
if (isa<IntToPtrInst>(inst))   return printIntToPtrInst(inst) + colon;
if (isa<CallInst>(inst))       return printIntrinsic(inst) + colon;
if (isa<Instruction>(inst)) errs()<<"Unable to process "<<*inst<<"\n";
assert(0 && "Unhandaled instruction");
abort();
return colon;
}


string RTLWriter::getTypeDecl(const Type *Ty, bool isSigned, const std::string &NameSoFar) {
assert((Ty->isPrimitiveType() || Ty->isIntegerTy() || Ty->isSized()) && "Invalid type decl");
std::stringstream ss;
switch (Ty->getTypeID()) {
case Type::VoidTyID: { 
return std::string(" reg /*void*/") +  NameSoFar;
}
case Type::PointerTyID: { // define the verilog pointer type
unsigned NBits = m_pointerSize; // 32bit for our pointers
ss<< "reg ["<<NBits-1<<":0] "<<NameSoFar;
return ss.str();
}
case Type::IntegerTyID: { // define the verilog integer type
unsigned NBits = cast<IntegerType>(Ty)->getBitWidth();
if (NBits == 1) return std::string("reg ")+NameSoFar;
ss<< "reg ["<<NBits-1<<":0] "<<NameSoFar;
return ss.str();
}
default :
errs() << "Unknown primitive type: " << *Ty << "\n";
abort();
}
}

string RTLWriter::getMemDecl(Function *F) {
stringstream ss;
MemportMap memports = listScheduler::getMemoryPortDeclerations(F,TD);//JAWAD  

// For each of the instances of each memory port)
for (unsigned int i=0; i<m_memportNum; i++) {
for (MemportMap::iterator it = memports.begin(); it != memports.end(); ++it) {
std::string name = it->first;
int width = it->second;
ss<<"input wire ["<<width-1<<":0] mem_"<<name<<"_out"<<i<<";\n";
ss<<"output reg ["<<width-1<<":0] mem_"<<name<<"_in"<<i<<";\n";
ss<<"output reg ["<<m_pointerSize-1<<":0] mem_"<<name<<"_addr"<<i<<";\n";
ss<<"output reg mem_"<<name<<"_mode"<<i<<";\n";
} 
}

ss<<"\n\n";
return ss.str();
}

std::string RTLWriter::getFunctionLocalVariables(listSchedulerVector lsv) {

  std::stringstream ss;
  // for each listScheduler of a basic block
  for (listSchedulerVector::iterator lsi=lsv.begin(); lsi!=lsv.end();++lsi) {
    // for each cycle in each LS
    for (unsigned int cycle=0; cycle<(*lsi)->length();cycle++) {
      std::vector<Instruction*> inst = (*lsi)->getInstructionForCycle(cycle);
      // for each instruction in each cycle in each LS
      for (std::vector<Instruction*>::iterator I = inst.begin(); I!=inst.end(); ++I) {
        // if has a return type, print it as a variable name
        if (!(*I)->getType()->isVoidTy()) {
          ss << " ";
          ss << getTypeDecl((*I)->getType(), false, vlang.GetValueName(*I));
          ss << ";   /*local var*/\n";
        }    
      }
    }// for each cycle    

    // Print all PHINode variables as well
    BasicBlock *bb= (*lsi)->getBB(); 
    for (BasicBlock::iterator bit = bb->begin(); bit != bb->end(); bit++) { 
      if (isa<PHINode>(bit)) {
        // if has a return type, print it as a variable name 
        if (!(bit)->getType()->isVoidTy()) { 
          ss << " "; 
          ss << getTypeDecl((bit)->getType(), false, vlang.GetValueName(bit)); 
          ss << ";   /*phi var*/\n"; 
        }     
      }
    } 
  }// for each listScheduler

return ss.str();
}

unsigned int RTLWriter::getNumberOfStates(listSchedulerVector &lsv){
int numberOfStates = 0;
for (listSchedulerVector::iterator it = lsv.begin(); it!=lsv.end();it++) {
numberOfStates += (*it)->length();
}
return numberOfStates;
}

string RTLWriter::getStateDefs(listSchedulerVector &lsv)  {
std::stringstream ss;

unsigned int numberOfStates = getNumberOfStates(lsv);

// Instruction pointer of n bits, n^2 states
unsigned int NumOfStateBits = Log2_32_Ceil(numberOfStates+1) -1;

ss<<"\n // Number of states:"<<numberOfStates<<"\n";
ss << " reg ["<< NumOfStateBits<<":0] eip;\n";

int stateCounter = 0;
// print the definitions for the values of the EIP values.
//     // for example: 'define start 16'd0  ...
for (listSchedulerVector::iterator it = lsv.begin(); it!=lsv.end(); it++) {
// each cycle in the BB
for (unsigned int i=0;i<(*it)->length();i++) {
ss << " parameter "<<toPrintable((*it)->getBB()->getName())<<i
<<" = "<<NumOfStateBits+1<<"'d"<<stateCounter<<";\n";
stateCounter++;
}
}
ss<<"\n";
return ss.str();
}



string RTLWriter::printAssignPart(vector<assignPartEntry*> ass, RTLWriter* lang) {
stringstream sb;
map<string,string> unitNames;

sb <<"// Assign part ("<< ass.size() <<")\n";

// extract all unit names from assign part
for (vector<assignPartEntry*>::iterator it = ass.begin(); it!=ass.end(); ++it) {
unitNames[(*it)->getUnitName()] = (*it)->getUnitType();
}

// for each uniqe name 
for (map<string,string>::iterator nm = unitNames.begin(); nm!=unitNames.end(); ++nm) {
assignPartBuilder apb(nm->first, nm->second);
// for all assign parts with this name
for (vector<assignPartEntry*>::iterator it = ass.begin(); it!=ass.end(); ++it) {
if (nm->first==(*it)->getUnitName()) {
apb.addPart(*it);
}
}
sb<<apb.toString(this);
}
sb << "\n\n";
return sb.str();
}


string RTLWriter::getAssignmentString(listSchedulerVector lv) {
stringstream sb;

vector<assignPartEntry*> parts;
for (listSchedulerVector::iterator it=lv.begin(); it!=lv.end(); ++it) {
vector<assignPartEntry*> p = (*it)->getAssignParts();
parts.insert(parts.begin(),p.begin(),p.end());
}

return printAssignPart(parts, this); 

}

string RTLWriter::evalValue(Value* val) {
if (Instruction* inst = dyn_cast<Instruction>(val)) {
if (abstractHWOpcode::isInstructionOnlyWires(inst)) return printInlinedInstructions(inst);
}
return vlang.GetValueName(val);
}

string RTLWriter::printInlinedInstructions(Instruction* inst) {
std::stringstream ss;
ss<<"(";

if (BinaryOperator *calc = dyn_cast<BinaryOperator>(inst)) {
Value *v0 = (calc->getOperand(0));
Value *v1 = (calc->getOperand(1));
// if second parameter is a constant (shift by constant is wiring)
if (calc->getOpcode() == Instruction::Shl) {
ss<<evalValue(v0)<<" << "<<evalValue(v1);
} else if (calc->getOpcode() == Instruction::LShr || calc->getOpcode() == Instruction::AShr) {
ss<<evalValue(v0)<<" >> "<<evalValue(v1);
} else if (calc->getOpcode() == Instruction::Add) {
ss<<evalValue(v0)<<" + "<<evalValue(v1);
} else if (calc->getOpcode() == Instruction::Xor) {
ss<<evalValue(v0)<<" ^ "<<evalValue(v1);
} else if (calc->getOpcode() == Instruction::And) {
ss<<evalValue(v0)<<" & "<<evalValue(v1);
} else if (calc->getOpcode() == Instruction::Or) {
ss<<evalValue(v0)<<" | "<<evalValue(v1);
} else {
errs()<<"Unknown Instruction "<<*calc;
abort();
}
} else if (isa<TruncInst>(inst)) { 
// make the cast
ss <<  evalValue(inst->getOperand(0));
} else if (isa<SExtInst>(inst)) { 
// make the cast
ss <<  evalValue(inst->getOperand(0));
} else if (isa<ZExtInst>(inst)) { 
// make the cast
ss <<  evalValue(inst->getOperand(0));
} else if (isa<IntToPtrInst>(inst)) { 
// make the cast
ss <<  evalValue(inst->getOperand(0));
} else if (isa<GetElementPtrInst>(inst)) { 
// make the cast
ss <<  getGetElementPtrInst(inst);
} else if (isa<CallInst>(inst)) { 
// make the cast
ss <<  getIntrinsic(dyn_cast<CallInst>(inst));
} else if (isa<PtrToIntInst>(inst)) { 
// make the cast
ss <<  evalValue(inst->getOperand(0));
} else {
errs()<<"Unknown wire instruction "<<*inst;
abort();
}

ss<<")";
return ss.str();
}

string RTLWriter::getFunctionSignature(const Function *F) {
  std::stringstream ss;

  MemportMap memports = listScheduler::getMemoryPortDeclerations(F,TD);//JAWAD  

  ss << "module ";

  // Print out the name...
  ss << vlang.GetValueName(F) << "(\n"
     << "input wire clk, \n"
     << "input wire reset,\n"
     << "output reg rdy,// control \n\t";

  // For each of the instances of each memory port)
  for (unsigned int i=0; i<m_memportNum; i++) {
    // print memory port decl
    for (MemportMap::iterator it = memports.begin(); it != memports.end(); ++it) {
    std::string name = it->first;
    ss<<"mem_"<<name<<"_out"<<i
      <<", mem_"<<name<<"_in"<<i
      <<", mem_"<<name<<"_addr"<<i
      <<", mem_"<<name<<"_mode"<<i
      <<", // memport for: "<<name<<" \n\t";
  }
}
  const AttrListPtr &PAL = F->getAttributes();
  unsigned Idx = 1;
  for (Function::const_arg_iterator I = F->arg_begin(), E = F->arg_end();
    I != E; ++I) {
      // integers:
      const Type *ArgTy = I->getType();
      ss << VLang::printType(ArgTy,
        /*isSigned=*/PAL.paramHasAttr(Idx, Attribute::SExt),
        vlang.GetValueName(I), "wire ", "input ");
      ++Idx;
      ss << ",\n";
  }

  const Type *RetTy = F->getReturnType();
  if (RetTy->isVoidTy()) {
    // Do something?
  } else {
    assert(RetTy->isIntegerTy() && "Only support return integer now!");
    ss << VLang::printType(RetTy, false, "return_value", "reg ", "output ") << ");\n";
  }

  return ss.str();
}

string RTLWriter::getBRAMDefinition(unsigned int wordBits, unsigned int addressBits) {
std::stringstream ss;

#if 0
ss<<
"// Dual port memory block\n"\
"module xram (out0, din0, addr0, we0, clk0,\n"\
"           out1, din1, addr1, we1, clk1);\n"\
"  parameter ADDRESS_WIDTH = "<<addressBits<<";\n";
ss<<"  parameter WORD_WIDTH = "<<wordBits<<";\n";
ss<<
"  output [WORD_WIDTH-1:0] out0;\n"\
"  input [WORD_WIDTH-1:0] din0;\n"\
"  input [ADDRESS_WIDTH-1:0] addr0;\n"\
"  input we0;\n"\
"  input clk0;\n"\
"  output [WORD_WIDTH-1:0] out1;\n"\
"  input [WORD_WIDTH-1:0] din1;\n"\
"  input [ADDRESS_WIDTH-1:0] addr1;\n"\
"  input we1;\n"\
"  input clk1;\n"\
"  reg [WORD_WIDTH-1:0] mem[1<<ADDRESS_WIDTH-1:0];\n"\
"   integer i;\n"\
"   initial begin\n"\
"       for (i = 0; i < (1<<(ADDRESS_WIDTH-1)); i = i + 1) begin\n"\
"       mem[i] <= i;\n"\
"     end\n"\
"   end\n"\
"  assign out0 = mem[addr0];\n"\
"  assign out1 = mem[addr1];\n"\
"  always @(posedge clk0)begin\n"\
"      if (we0) begin\n"\
"          mem[addr0] = din0;\n"\
"          $display($time,\"w mem[%d] == %d; in=%d\",addr0, mem[addr0],din0);\n"\
"      end\n"\
"  end\n"\
"  always @(posedge clk1)begin\n"\
"      if (we1) begin\n"\
"          mem[addr1] = din1;\n"\
"          $display($time,\"w mem[%d] == %d; in=%d\",addr0, mem[addr0],din0);\n"\
"      end \n"\
"  end\n"\
"endmodule\n";
#endif
return ss.str();
}

string RTLWriter::createBinOpModule(string opName, string symbol, unsigned int stages) {

std::stringstream ss;

ss<<"\nmodule "<<opName<<" (clk, a, b, p);\n";
ss<<"output reg [31:0] p;\ninput [31:0] a;\ninput [31:0] b;\ninput clk;";
for (unsigned int i=0; i<stages-1; i++) {
ss<<"reg [31:0] t"<<i<<";\n";
}
ss<<"always @(posedge clk)begin\n";
ss<<"t0 <= a "<<symbol<<" b;\n";
for (unsigned int i=1; i<stages-1; i++) {
ss<<"t"<<(i)<<" <= t"<<(i-1)<<";\n";
}
ss<<"p <=t"<<stages-2<<";\nend\nendmodule\n\n";   
return ss.str();
}




}//namespace

