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
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallString.h"
#include "llvm/Support/ErrorHandling.h"

#include <sstream>

using namespace llvm;
using namespace xVerilog;

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

std::stringstream &indent(std::stringstream &ss, unsigned NumSpaces) {
  static const char Spaces[] = "                                "
    "                                "
    "                ";

  // Usually the indentation is small, handle it with a fastpath.
  if (NumSpaces < array_lengthof(Spaces)) {
    ss.write(Spaces, NumSpaces);
    return ss;
  }

  while (NumSpaces) {
    unsigned NumToWrite = std::min(NumSpaces,
      (unsigned)array_lengthof(Spaces)-1);
    ss.write(Spaces, NumToWrite);
    NumSpaces -= NumToWrite;
  }
  return ss;
}


//===----------------------------------------------------------------------===//
// Value and type printing

std::string VLang::printBitWitdh(const Type *Ty, int LowestBit,
                                           bool printOneBit) {
  std::stringstream bw;
  int BitWitdh = cast<IntegerType>(Ty)->getBitWidth();
  if (BitWitdh !=1) 
    bw << "[" << (BitWitdh - 1 + LowestBit) << ":" << LowestBit << "] ";
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
    unsigned int uselength=(bitwidth/4)+(((bitwidth%3)==0)?0:1);
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

  return "llvm_vbe_" + VarName;
}

std::string VLang::printType(const Type *Ty,
                             bool isSigned /* = false */,
                             const std::string &VariableName /* =  */,
                             const std::string &SignalType /* =  */,
                             const std::string &Direction /* =  */,
                             bool IgnoreName /* = false */,
                             const AttrListPtr &PAL /* = AttrListPtr */) {
  std::stringstream ss;
  ss << Direction;
  if (Ty->isIntegerTy()) {
    ss << printSimpleType(Ty, isSigned, VariableName, SignalType);
    return ss.str();
  }
  return "???";
}

std::string VLang::printSimpleType(const Type *Ty, bool isSigned,
                                   const std::string &NameSoFar /* =  */,
                                   const std::string &SignalType /* = */){
  std::stringstream ss;
  //wire or reg?
	ss << SignalType;
	//signed?
	if(isSigned)
		ss << "signed ";

  switch (Ty->getTypeID()) {
  case Type::IntegerTyID:
    ss << printBitWitdh(Ty) << NameSoFar;
    return ss.str();
  default:
    llvm_unreachable("Unsupport type!");
  }
}

void VLang::initializePass() {
  TD = getAnalysisIfAvailable<TargetData>();
  TAsm = new VBEMCAsmInfo();
  TCtx = new MCContext(*TAsm);
  Mang = new Mangler(*TCtx, *TD);
}

std::string VLang::emitAlwaysffBegin(unsigned level /* = 0 */,
                                const std::string &Clk /* =  */,
                                const std::string &ClkEdge /* =  */,
                                const std::string &Rst /* = */,
                                const std::string &RstEdge /* =  */) const {
  level *= 2;
  std::stringstream ss;
  // TODO: Support Sync reset
  // TODO: SystemVerilog always_ff?
  indent(ss, level) << "always @("
                    << ClkEdge << " "<< Clk <<", "
                    << RstEdge << " " << Rst
                    <<") begin\n";
  level += 2;
  indent(ss, level) << "if (";
  // negative edge reset?
  if (RstEdge == "negedge")
    ss << "!";
  ss << Rst << ") begin\n";
  level += 2;
  indent(ss, level) << "eip<=0;\n";
  indent(ss, level) << "rdy<=0;\n";
  // TODO: Reset other registers!
  level -= 2;
  indent(ss, level) << "end\n";
  // TODO: clock enable
  indent(ss, level) << "else begin //else reset\n";
  return ss.str();
}

std::string VLang::emitEndAlwaysff(unsigned level /* = 0 */) const {
  std::stringstream ss;
  level *= 2;
  indent(ss, level + 2) << "end //else reset\n";
  indent(ss, level) << "end //always @(..)\n\n";
  return ss.str();
}

std::string VLang::emitCaseBegin(unsigned level /* = 0 */) const {
  std::stringstream ss;
  level *= 2;
  indent(ss, level) << "case (eip)\n";
  return ss.str();
}
std::string VLang::emitEndCase(unsigned level /* = 0 */) const {
  std::stringstream ss;
  level *= 2;
  indent(ss, level) << " endcase //eip\n";
  return ss.str();
}

std::string VLang::emitEndModule(unsigned level /* = 0 */) const {
  std::stringstream ss;
  level *= 2;
  indent(ss, level) << "endmodule\n\n";
  return ss.str();
}

char VLang::ID = 0;

static RegisterPass<VLang> X("vlang", "vbe - Verilog language writer",
                             false, true);
