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
//if eip=m_state,choose the operand.
// TODO: Wire witdh!
sb << "wire [31:0] "<<m_name<<"_in_a"<<";\n";
sb << "wire [31:0] "<<m_name<<"_in_b"<<";\n";
sb <<" assign " <<m_name<<"_in_a"<<" = ";
for (vector<assignPartEntry*>::iterator it=m_parts.begin(); it!=m_parts.end();++it) {
assignPartEntry *part = *it;
sb <<"\n (eip == "<<part->getState()<<") ? "<<
vl->evalValue(part->getLeft())<<" :";    //left operand
}   

sb <<"0;\n";
sb <<" assign " <<m_name<<"_in_b"<<" = ";
for (vector<assignPartEntry*>::iterator it=m_parts.begin(); it!=m_parts.end();++it) {
  assignPartEntry *part = *it;
  sb <<"\n (eip == "<<part->getState()<<") ? "<<
  vl->evalValue(part->getRight())<<" :";
}   //right operand
sb <<"0;\n\n";

sb<<"wire [31:0] out_"<<m_name<<";\n";
sb<<m_op<<"  "<<m_name<<"_instance (.clk(clk), .a("<<
m_name<<"_in_a)"<<", .b("<<m_name<<"_in_b), .p(out_"<<m_name<<"));\n\n";
return sb.str();
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

string RTLWriter::evalValue(Value* val) {
  if (Instruction* inst = dyn_cast<Instruction>(val)) {
    if (abstractHWOpcode::isInstructionOnlyWires(inst))
      return printInlinedInstructions(inst);
  }

  // If val is just a constant?
  if (Constant *C = dyn_cast<Constant>(val))
    return vlang.printConstant(C);

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

