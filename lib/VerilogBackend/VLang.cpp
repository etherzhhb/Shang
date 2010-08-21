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

#include "VLang.h"
#include "llvm/Constants.h"
#include "llvm/GlobalVariable.h"
#include "llvm/DerivedTypes.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/ADT/SmallString.h"
#include "llvm/Support/ErrorHandling.h"

#include <sstream>

using namespace llvm;
using namespace esyn;

namespace {
  class VBEMCAsmInfo : public MCAsmInfo {
  public:
    VBEMCAsmInfo() {
      GlobalPrefix = "";
      PrivateGlobalPrefix = "";
    }
  };
}

// Helper functions
static std::string VLangMangle(const std::string &S) {
  std::string Result;

  for (unsigned i = 0, e = S.size(); i != e; ++i)
  if (isalnum(S[i]) || S[i] == '_') {
  Result += S[i];
  } else {
  Result += '_';
  Result += 'A'+(S[i]&15);
  Result += 'A'+((S[i]>>4)&15);
  Result += '_';
  }
  return Result;
}

//===----------------------------------------------------------------------===//
// Value and type printing

std::string VLang::printBitWitdh(unsigned BitWidth, int LowestBit,
                                 bool printOneBit) {
  std::stringstream bw;
  if (BitWidth !=1) 
    bw << "[" << (BitWidth - 1 + LowestBit) << ":" << LowestBit << "] ";
  else if(printOneBit)
    bw << "[" << LowestBit << "] ";
  bw << " ";
  return bw.str();
}

std::string VLang::printConstant(Constant *CPV) {
  if (ConstantInt* CI=dyn_cast<ConstantInt>(CPV)) {
    bool isMinValue=CI->isMinValue(true);
    uint64_t v = isMinValue ? CI->getZExtValue() :
                             (uint64_t)CI->getSExtValue();
    return printConstantInt(v,CI->getBitWidth(),isMinValue);
  } else if (const GlobalVariable *GVar = dyn_cast<GlobalVariable>(CPV))
    return GetValueName(GVar);

  return "??Constant??";
}

std::string VLang::printConstantInt(uint64_t value,
                                    int bitwidth, bool isMinValue) {
  std::stringstream pc;
  pc<<bitwidth<<"'h";
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

std::string VLang::GetValueName(const Value *Operand) {
  // Mangle globals with the standard mangler interface for LLC compatibility.
  if (const GlobalValue *GV = dyn_cast<GlobalValue>(Operand)) {
    SmallString<128> Str;
    Mang->getNameWithPrefix(Str, GV, false);
    return VLangMangle(Str.str().str());
  }

  std::string Name = Operand->getName();

  // Constant

  if (Name.empty()) { // Assign unique names to local temporaries.
    unsigned &No = AnonValueNumbers[Operand];
    if (No == 0)
      No = ++NextAnonValueNumber;
    Name = "tmp__" + utostr(No);
  }

  std::string VarName;
  VarName.reserve(Name.capacity());

  for (std::string::iterator I = Name.begin(), E = Name.end();
      I != E; ++I) {
    char ch = *I;

  if (!((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
      (ch >= '0' && ch <= '9') || ch == '_')) {
    char buffer[5];
    sprintf(buffer, "_%x_", ch);
    VarName += buffer;
  } else
    VarName += ch;
  }

  return "esyn_" + VarName;
}

void VLang::initializePass() {
  TD = getAnalysisIfAvailable<TargetData>();
  TAsm = new VBEMCAsmInfo();
  TCtx = new MCContext(*TAsm);
  Mang = new Mangler(*TCtx, *TD);
}

char VLang::ID = 0;

static RegisterPass<VLang> X("vlang", "vbe - Verilog language writer",
                             false, true);

raw_ostream &VLang::comment(raw_ostream &ss) const {
  ss <<  "//  ";
  return ss;
}

raw_ostream &VLang::indent(raw_ostream &ss) const {
  return ss.indent(ind_level);
}

raw_ostream &VLang::moduleBegin(raw_ostream &ss, std::string &ModuleName) {
  ss << "module " << ModuleName << "(\n";
  return ss;
}

raw_ostream &VLang::endModuleDecl(raw_ostream &ss) {
  ss <<  ");\n";
  return ss;
}

raw_ostream &VLang::alwaysBegin(raw_ostream &ss, unsigned ind,
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

raw_ostream &VLang::resetRegister(raw_ostream &ss, const std::string &Name,
                                  unsigned BitWidth, unsigned InitVal){
  ss << Name << " <=  "
     << printConstantInt(InitVal, BitWidth, false)
     << ";\n";
  return ss;
}

raw_ostream &VLang::endModule(raw_ostream &ss) {
  ss << "endmodule\n\n";
  return ss;
}

raw_ostream &VLang::endSwitch(raw_ostream &ss) {
  ss << "endcase\n";
  return ss;
}

raw_ostream &VLang::begin(raw_ostream &ss) {
  ss << "begin\n";
  return ss;
}

raw_ostream &VLang::end(raw_ostream &ss) {
  ss << "end\n";
  return ss;
}

raw_ostream &VLang::ifElse(raw_ostream &ss, bool Begin) {
  ss << "end else" << (Begin ? " begin\n" : "\n");
  return ss;
}

raw_ostream &VLang::ifBegin(raw_ostream &ss, const std::string &Condition) {
  ss << "if (" << Condition << ") begin\n";
  return ss;
}

raw_ostream &VLang::matchCase(raw_ostream &ss, const std::string &StateName) {
  ss << StateName << ": begin\n";
  return ss;
}

raw_ostream &VLang::switchCase(raw_ostream &ss, const std::string &StateName) {
  ss << "case (" << StateName <<")\n";
  return ss;
}

raw_ostream &VLang::alwaysEnd(raw_ostream &ss, unsigned ind) {
  ss.indent(ind + 2) << "end //else reset\n";
  ss.indent(ind) << "end //always @(..)\n\n";
  return ss;
}

raw_ostream &VLang::param(raw_ostream &ss, const std::string &Name,
                          unsigned BitWidth, unsigned Val) {
  ss << "parameter " << Name
    << " = " << printConstantInt(Val, BitWidth, false) << ";\n";
  return ss;
}

raw_ostream &VLang::declSignal(raw_ostream &ss, const std::string &Name,
                               unsigned BitWidth, unsigned Val,
                               bool isReg, bool isSigned,
                               const std::string &Term) {
  ss << (isReg ? "reg " : "wire ");
  
  if(isSigned) ss << "signed ";

  if (BitWidth != 1) ss << "[" << BitWidth - 1 << ":0]";

  ss << " " << Name;

  // Dirty Hack: We not always assign the initialize for register! 
  if (isReg && Term == ";")
    ss << " = " << printConstantInt(0, BitWidth, false);

  ss << Term <<'\n';
  return ss;
}
