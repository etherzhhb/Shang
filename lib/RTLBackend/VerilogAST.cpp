//===------------- VLang.h - Verilog HDL writing engine ---------*- C++ -*-===//
//
//                            The Verilog Backend
//
// Copyright: 2010 by Hongbin Zheng. all rights reserved.
// IMPORTANT: This software is supplied to you by Hongbin Zheng in consideration
// of your agreement to the following terms, and your use, installation, 
// modification or redistribution of this software constitutes acceptance
// of these terms.  If you do not agree with these terms, please do not use, 
// install, modify or redistribute this software. You may not redistribute, 
// install copy or modify this software without written permission from 
// Hongbin Zheng. 
//
//===----------------------------------------------------------------------===//
//
// This file implement the VLang class, with provide funtions to complete
// common Verilog HDL writing task.
//
//===----------------------------------------------------------------------===//

#include "vtm/VerilogAST.h"
#include "vtm/Passes.h"

#include "llvm/Constants.h"
#include "llvm/GlobalVariable.h"
#include "llvm/DerivedTypes.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/ADT/SmallString.h"
#include "llvm/Support/ErrorHandling.h"

#include <algorithm>
#include <sstream>

using namespace llvm;

//===----------------------------------------------------------------------===//
// Value and type printing

std::string llvm::verilogBitRange(unsigned UB, unsigned LB, bool printOneBit) {
  std::stringstream bw;
  --UB;
  if (UB != LB) 
    bw << "[" << UB << ":" << LB << "]";
  else if(printOneBit)
    bw << "[" << LB << "]";

  return bw.str();
}

std::string llvm::verilogConstToStr(Constant *CPV) {
  if (ConstantInt* CI=dyn_cast<ConstantInt>(CPV)) {
    bool isMinValue=CI->isMinValue(true);
    uint64_t v = isMinValue ? CI->getZExtValue() :
                             (uint64_t)CI->getSExtValue();
    return verilogConstToStr(v,CI->getBitWidth(),isMinValue);
  } else if (ConstantExpr *CE = dyn_cast<ConstantExpr>(CPV)) {
    switch (CE->getOpcode()) {
    case Instruction::PtrToInt:
    case Instruction::IntToPtr:
    case Instruction::BitCast:
      return verilogConstToStr(CE->getOperand(0));
    }
    
  }
  
  return "??Constant??";
}

std::string llvm::verilogConstToStr(uint64_t value, unsigned bitwidth,
                                    bool isMinValue) {
  std::stringstream pc;
  pc <<bitwidth<< "'h";
  if(isMinValue)
    pc<<std::hex<<value;
  else{
    std::stringstream ss;
    ss<<std::hex<<value;
    unsigned int uselength = (bitwidth/4) + (((bitwidth&0x3) == 0) ? 0 : 1);
    std::string sout=ss.str();
    if(uselength<sout.length())
      sout=sout.substr(sout.length()-uselength,uselength);
    pc<<sout;
  }
  return pc.str();
}

//std::string VLang::GetValueName(const Value *Operand) {
//  // Mangle globals with the standard mangler interface for LLC compatibility.
//  if (const GlobalValue *GV = dyn_cast<GlobalValue>(Operand)) {
//    SmallString<128> Str;
//    Mang->getNameWithPrefix(Str, GV, false);
//    return VLangMangle(Str.str().str());
//  }
//
//  std::string Name = Operand->getName();
//
//  // Constant
//
//  if (Name.empty()) { // Assign unique names to local temporaries.
//    unsigned &No = AnonValueNumbers[Operand];
//    if (No == 0)
//      No = ++NextAnonValueNumber;
//    Name = "tmp__" + utostr(No);
//  }
//
//  std::string VarName;
//  VarName.reserve(Name.capacity());
//
//  for (std::string::iterator I = Name.begin(), E = Name.end();
//      I != E; ++I) {
//    char ch = *I;
//
//  if (!((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
//      (ch >= '0' && ch <= '9') || ch == '_')) {
//    char buffer[5];
//    sprintf(buffer, "_%x_", ch);
//    VarName += buffer;
//  } else
//    VarName += ch;
//  }
//
//  return "esyn_" + VarName;
//}

raw_ostream &llvm::verilogAlwaysBegin(raw_ostream &ss, unsigned ind,
                                      const std::string &Clk /*= "clk"*/,
                                      const std::string &ClkEdge /*= "posedge"*/,
                                      const std::string &Rst /*= "rstN"*/,
                                      const std::string &RstEdge /*= "negedge"*/){
  // TODO: Support Sync reset
  // TODO: SystemVerilog always_ff?
  ss.indent(ind) << "always @("
    << ClkEdge << " "<< Clk <<", "
    << RstEdge << " " << Rst
    <<") begin\n";
  ind += 2;
  ss.indent(ind) << "if (";
  // negative edge reset?
  if (RstEdge == "negedge")
    ss << "!";
  ss << Rst << ") begin\n";
  ind += 2;
  ss.indent(ind) << "// reset registers\n";
  return ss;
}

raw_ostream &llvm::verilogEndModule(raw_ostream &ss) {
  ss << "endmodule\n\n";
  return ss;
}

raw_ostream &llvm::verilogEndSwitch(raw_ostream &ss) {
  ss << "endcase\n";
  return ss;
}

raw_ostream &llvm::verilogBegin(raw_ostream &ss) {
  ss << "begin\n";
  return ss;
}

raw_ostream &llvm::verilogEnd(raw_ostream &ss) {
  ss << "end\n";
  return ss;
}

raw_ostream &llvm::verilogIfElse(raw_ostream &ss, bool Begin) {
  ss << "end else" << (Begin ? " begin\n" : "\n");
  return ss;
}

raw_ostream &llvm::verilogIfBegin(raw_ostream &ss,
                                  const std::string &Condition) {
  ss << "if (" << Condition << ") begin\n";
  return ss;
}

raw_ostream &llvm::verilogMatchCase(raw_ostream &ss,
                                    const std::string &StateName) {
  ss << StateName << ": begin\n";
  return ss;
}

raw_ostream &llvm::verilogSwitchCase(raw_ostream &ss,
                                     const std::string &StateName) {
  ss << "case (" << StateName <<")\n";
  return ss;
}

raw_ostream &llvm::verilogAlwaysEnd(raw_ostream &ss, unsigned ind) {
  ss.indent(ind + 2) << "end //else reset\n";
  ss.indent(ind) << "end //always @(..)\n\n";
  return ss;
}

raw_ostream &llvm::verilogParam(raw_ostream &ss, const std::string &Name,
                                unsigned BitWidth, unsigned Val) {
  ss << "parameter " << Name
    << " = " << verilogConstToStr(Val, BitWidth, false) << ";\n";
  return ss;
}

raw_ostream &llvm::verilogCommentBegin(raw_ostream &ss) {
  return ss << "// ";
}

VASTModule::~VASTModule() {
  // Release all ports.
  std::for_each(Ports.begin(), Ports.end(), deleter<VASTPort>);
  Ports.clear();
  std::for_each(Signals.begin(), Signals.end(), deleter<VASTSignal>);
  Signals.clear();

  delete &(StateDecl.str());
  delete &(DataPath.str());
  delete &(ControlBlock.str());
}

void VASTModule::clear() {
  // Clear buffers
  StateDecl.str().clear();
  DataPath.str().clear();
  ControlBlock.str().clear();
  SeqCompute.str().clear();
}

void VASTModule::printModuleDecl(raw_ostream &OS) const {
  OS << "module " << getName() << "(\n";
  Ports.front()->print(OS.indent(4));
  for (PortVector::const_iterator I = Ports.begin() + 1, E = Ports.end();
       I != E; ++I) {
    OS << ",\n";
    (*I)->print(OS.indent(4));
  }
  OS << ");\n";
}

void VASTModule::printSignalDecl(raw_ostream &OS, unsigned indent /*= 2*/) {
  for (SignalVector::const_iterator I = Signals.begin(), E = Signals.end();
       I != E; ++I) {
    (*I)->printDecl(OS.indent(indent));
    OS << "\n";
  }
  
}

void VASTModule::printRegisterReset(raw_ostream &OS, unsigned indent /*= 6*/) {
  // Reset output registers.
  for (PortVector::const_iterator I = Ports.begin(), E = Ports.end();
       I != E; ++I) {
    VASTPort *port = *I;
    if (port->isRegister()) {
      port->printReset(OS.indent(indent));
      OS << "\n";
    }
  }

  for (SignalVector::const_iterator I = Signals.begin(), E = Signals.end();
       I != E; ++I) {
    VASTSignal *signal = *I;
    if (signal->isRegister()) {
      signal->printReset(OS.indent(indent));
      OS << "\n";
    }
  }
}

void VASTModule::print(raw_ostream &OS) const {
  // Print the verilog module?
}

void VASTValue::printReset( raw_ostream &OS ) const {
  OS << getName()  << " <= " 
     << verilogConstToStr(InitVal, getBitWidth(), false) << ";";
}

void VASTPort::print(raw_ostream &OS) const {
  if (isInput())
    OS << "input ";
  else
    OS << "output ";

  if (isRegister())
    OS << "reg";
  else
    OS << "wire";
  
  if (getBitWidth() > 1)
    OS << "[" << (getBitWidth() - 1) << ":0]";

   OS << ' ' << getName();
}

void VASTPort::printExternalDriver(raw_ostream &OS, uint64_t InitVal) const {
  if (isInput())
    // We need a reg to drive input port.
    OS << "reg";
  else
    // We need a wire to accept the output value from dut.
    OS << "wire";

  if (getBitWidth() > 1)
    OS << "[" << (getBitWidth() - 1) << ":0]";

  OS << ' ' << getName()
     << " = " << verilogConstToStr(InitVal, getBitWidth(), false) << ';';
}

void VASTSignal::print(raw_ostream &OS) const {

}

void VASTSignal::printDecl(raw_ostream &OS) const {
  if (isRegister())
    OS << "reg";
  else
    OS << "wire";

  if (getBitWidth() > 1)
    OS << "[" << (getBitWidth() - 1) << ":0]";

  OS << ' ' << getName();

  if (isRegister())
    OS << " = " << verilogConstToStr(0, getBitWidth(), false);

  OS << ";";
}
