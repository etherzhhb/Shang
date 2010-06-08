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

using namespace llvm;

namespace xVerilog {

class VLang : public ImmutablePass {
  const TargetData* TD;
  Mangler *Mang;
  const MCAsmInfo* TAsm;
  MCContext *TCtx;
  // For unname value
  DenseMap<const Value*, unsigned> AnonValueNumbers;
  unsigned NextAnonValueNumber;

  //
  static unsigned ind_level;

  void clear() {
    AnonValueNumbers.clear();
  }
  
  template<class StreamTy>
  static StreamTy &indent_imp(StreamTy &ss, unsigned NumSpaces) {
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

  template<class StreamTy>
  StreamTy &printPtrDecl(StreamTy &ss, const Argument *Arg, 
                           unsigned DataWidth, unsigned BusWidth,
                           unsigned level = ind_level) {
    ind_level = level;
    std::string name = Arg->getNameStr();
    const PointerType *PtrTy = dyn_cast<PointerType>(Arg->getType());
    assert(PtrTy && "Arg is not ptr!");
    // TODO: whats memport?
    unsigned i = 0;
    // Pointer size
    indent_imp(ss, ind_level) << "input wire [" << (DataWidth-1) << ":0] mem_"
                    <<name<< "_out" << i <<",\n";

    indent_imp(ss, ind_level) << "output reg [" << (DataWidth - 1) << ":0] mem_"
                    << name << "_in" << i << ",\n";

    indent_imp(ss, ind_level) << "output reg [" << (BusWidth - 1) <<":0] mem_"
                    << name << "_addr"<< i << ",\n";

    indent_imp(ss, ind_level) << "output reg mem_" << (name) << "_mode" << i;
    return ss;
  }

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
  StreamTy &indent(StreamTy &ss, unsigned level = ind_level) {
    return indent_imp(ss, (ind_level = level));
  }

  template<class StreamTy>
  StreamTy &emitModuleBegin(StreamTy &ss, std::string &ModuleName,
                              const std::string &Clk = "clk",
                              const std::string &Rst = "rstN",
                              const std::string &Rdy = "rdy",
                              unsigned level = ind_level) {
    ind_level = level;
    indent_imp(ss, ind_level) << "module " << ModuleName << "(\n";
    ind_level+=4;
    // FIXME: mutiple clk!
    if (!Clk.empty())
      indent_imp(ss, ind_level) << "input wire " << Clk << ",\n";
    if (!Rst.empty())
      indent_imp(ss, ind_level) << "input wire " << Rst << ",\n";
    if (!Rdy.empty())
      indent_imp(ss, ind_level) << "output reg " << Rdy << ",\n";
    return ss;
  }

  template<class StreamTy>
  StreamTy &emitEndModuleDecl(StreamTy &ss, unsigned level = ind_level) {
    ind_level=level;
    ss <<  ");\n";
    ind_level-=2;
    return ss;
  }

  template<class StreamTy>
  StreamTy &emitAlwaysffBegin(StreamTy &ss, const std::string &Clk = "clk",
                                const std::string &ClkEdge = "posedge",
                                const std::string &Rst = "rstN",
                                const std::string &RstEdge = "negedge",
                                unsigned level = ind_level) {
    ind_level = level;
    // TODO: Support Sync reset
    // TODO: SystemVerilog always_ff?
    indent_imp(ss, ind_level) << "always @("
                      << ClkEdge << " "<< Clk <<", "
                      << RstEdge << " " << Rst
                      <<") begin\n";
    ind_level += 2;
    indent_imp(ss, ind_level) << "if (";
    // negative edge reset?
    if (RstEdge == "negedge")
      ss << "!";
    ss << Rst << ") begin\n";
    ind_level += 2;
    indent_imp(ss, ind_level) << "eip<=0;\n";
    indent_imp(ss, ind_level) << "rdy<=0;\n";
    // TODO: Reset other registers!
    level -= 2;
    indent_imp(ss, ind_level) << "end\n";
    // TODO: clock enable
    indent_imp(ss, ind_level) << "else begin //else reset\n";
    return ss;
  }
  template<class StreamTy>
  StreamTy &emitEndAlwaysff(StreamTy &ss, unsigned level = ind_level) {
    ind_level = level;
    indent_imp(ss, ind_level) << "end //else reset\n";
    ind_level -=2;
    indent_imp(ss, ind_level) << "end //always @(..)\n\n";
    ind_level +=2;
    return ss;
  }
  template<class StreamTy>
  StreamTy &emitCaseBegin(StreamTy &ss, unsigned level = ind_level) {
    ind_level = level;
    indent_imp(ss, ind_level) << "case (eip)\n";
    ind_level += 2;
    return ss;
  }
  template<class StreamTy>
  StreamTy &emitEndCase(StreamTy &ss, unsigned level = ind_level) {
    ind_level = level;
    indent_imp(ss, ind_level) << " endcase //eip\n";
    ind_level -= 2;
    return ss;
  }

  template<class StreamTy>
  StreamTy &emitEndModule(StreamTy &ss, unsigned level = ind_level) {
    ind_level = level;
    indent_imp(ss, ind_level) << "endmodule\n\n";
    return ss;
  }

  virtual void initializePass();

  virtual void getAnalysisUsage(AnalysisUsage &AU) {
    AU.addRequired<TargetData>();
  }
};
} // end namespace

#endif
