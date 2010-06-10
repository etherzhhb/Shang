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
// The VLang provide funtions to complete common Verilog HDL writing task.
//
//===----------------------------------------------------------------------===//
#ifndef VBE_VLANG_H
#define VBE_VLANG_H

#include "llvm/Pass.h"
#include "llvm/Function.h"
#include "llvm/Target/Mangler.h"
#include "llvm/Target/TargetData.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/Format.h"

using namespace llvm;

namespace esyn {

class VLang : public ImmutablePass {
  const TargetData* TD;
  Mangler *Mang;
  const MCAsmInfo* TAsm;
  MCContext *TCtx;
  // For unname value
  DenseMap<const Value*, unsigned> AnonValueNumbers;
  unsigned NextAnonValueNumber;

  //
  unsigned ind_level;

  void clear() {
    AnonValueNumbers.clear();
  }

public:
  static char ID;
  explicit VLang() 
    : ImmutablePass(&ID), NextAnonValueNumber(0) {}

  ~VLang() {
    clear();
    delete Mang;
    delete TAsm;
    delete TCtx;
  }

  /// @name Value and Type printing
  //{
  /// @brief Get the name of Value, if the Value have no name, 
  ///       just create one for it.
  ///
  /// @param Operand The Value to get the name.
  ///
  /// @return The unique name of the Value.
  std::string GetValueName(const Value *Operand); 

  std::string printConstant(Constant *C);
  std::string printConstantInt(uint64_t value,int bitwidth, bool isMinValue);

  static std::string printBitWitdh(const Type *Ty, int LowestBit = 0, 
    bool printOneBit = false);
  static std::string printType(const Type *Ty, 
    bool isSigned = false,
    const std::string &VariableName = "", 
    const std::string &SignalType = "wire",
    const std::string &Direction = "",
    bool IgnoreName = false,
    const AttrListPtr &PAL = AttrListPtr());
  static std::string printSimpleType(const Type *Ty, 
    bool isSigned, 
    const std::string &NameSoFar = "",
    const std::string &SignalType = "wire");
  //}

  template<class StreamTy>
  StreamTy &indent(StreamTy &ss) {
    //ss << "ind(" << format("%02d", ind_level) << ")";
    return static_cast<StreamTy&>(ss.indent(ind_level));
  }
  template<class StreamTy>
  StreamTy &emitCommentBegin(StreamTy &ss){
    ss << "\n";
    indent(ss) << "//  ";
    return ss;
  }

  template<class StreamTy>
  StreamTy &emitModuleBegin(StreamTy &ss, std::string &ModuleName,
                            const std::string &Clk = "clk",
                            const std::string &Rst = "rstN",
                            unsigned ind = 0) {
    ind_level = ind;
    indent(ss) << "module " << ModuleName << "(\n";
    ind_level+=4;
    return ss;
  }

  template<class StreamTy>
  StreamTy &emitEndModuleDecl(StreamTy &ss) {
    ss <<  ");\n";
    ind_level-=2;
    return ss;
  }

  template<class StreamTy>
  StreamTy &emitAlwaysffBegin(StreamTy &ss,
                              const std::string &Clk = "clk",
                              const std::string &ClkEdge = "posedge",
                              const std::string &Rst = "rstN",
                              const std::string &RstEdge = "negedge") {

    // TODO: Support Sync reset
    // TODO: SystemVerilog always_ff?
    indent(ss) << "always @("
                      << ClkEdge << " "<< Clk <<", "
                      << RstEdge << " " << Rst
                      <<") begin\n";
    ind_level += 2;
    indent(ss) << "if (";
    // negative edge reset?
    if (RstEdge == "negedge")
      ss << "!";
    ss << Rst << ") begin\n";
    ind_level += 2;
    indent(ss) << "// reset registers\n";
    // TODO: Reset other registers!
    return ss;
  }

  template<class StreamTy>
  StreamTy &emitResetRegister(StreamTy &ss,
                              const std::string &Name,
                              unsigned BitWidth,
                              unsigned InitVal = 0) {
    indent(ss) << Name << " <=  "
      << printConstantInt(InitVal, BitWidth, false)
      << ";\n";;
    return ss;
  }

  template<class StreamTy>
  StreamTy &emitParam(StreamTy &ss,
                      const std::string &Name,
                      unsigned BitWidth,
                      unsigned Val) {
    indent(ss) << "parameter " << Name
      << " = " << printConstantInt(Val, BitWidth, false) << "\n";
    return ss;
  }

  template<class StreamTy>
  StreamTy &emitEndReset(StreamTy &ss) {
    ind_level -= 2;
    indent(ss) << "end\n";
    // TODO: clock enable
    indent(ss) << "else begin //else reset\n";
    ind_level += 2;
    return ss;
  }

  template<class StreamTy>
  StreamTy &emitEndAlwaysff(StreamTy &ss) {

    ind_level -=2;
    indent(ss) << "end //else reset\n";
    ind_level -=2;
    indent(ss) << "end //always @(..)\n\n";
    return ss;
  }
  template<class StreamTy>
  StreamTy &emitCaseBegin(StreamTy &ss) {

    indent(ss) << "case (eip)\n";
    // Do not need to indent
    //ind_level += 2;
    return ss;
  }
  template<class StreamTy>
  StreamTy &emitCaseStateBegin(StreamTy &ss, const std::string &StateName) {
    indent(ss) << StateName << ": begin\n";
    ind_level += 2;
    return ss;
  }

  template<class StreamTy>
  StreamTy &emitIfBegin(StreamTy &ss, const std::string &Condition) {
    indent(ss) << "if (" << Condition << ") begin\n";
    ind_level += 2;
    return ss;
  }

  template<class StreamTy>
  StreamTy &emitIfElse(StreamTy &ss) {
    ind_level -=2;
    indent(ss) << "end else begin\n";
    ind_level +=2;
    return ss;
  }


  template<class StreamTy>
  StreamTy &emitEnd(StreamTy &ss) {
    ind_level -= 2;
    indent(ss) << "end\n";
    return ss;
  }

  template<class StreamTy>
  StreamTy &emitEndCase(StreamTy &ss) {
    // Do "case" dose not indent
    //ind_level -= 2;
    indent(ss) << " endcase //eip\n";
    return ss;
  }

  template<class StreamTy>
  StreamTy &emitEndModule(StreamTy &ss) {

    ind_level -=2;
    indent(ss) << "endmodule\n\n";
    return ss;
  }

  virtual void initializePass();

  virtual void getAnalysisUsage(AnalysisUsage &AU) {
    AU.addRequired<TargetData>();
  }
};
} // end namespace

#endif
