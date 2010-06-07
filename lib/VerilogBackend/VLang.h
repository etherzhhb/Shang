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

  std::string printPtrDecl(const Argument *Arg, 
                           unsigned DataWidth, unsigned BusWidth,
                           unsigned level = ind_level);

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

  
  std::stringstream &indent(std::stringstream &ss,
                            unsigned level = ind_level);

  std::string emitModuleBegin(std::string &ModuleName,
                              const std::string &Clk = "clk",
                              const std::string &Rst = "rstN",
                              const std::string &Rdy = "rdy",
                              unsigned level = ind_level);
  std::string emitEndModuleDecl(unsigned level = ind_level);

  std::string emitAlwaysffBegin(const std::string &Clk = "clk",
                                const std::string &ClkEdge = "posedge",
                                const std::string &Rst = "rstN",
                                const std::string &RstEdge = "negedge",
                                unsigned level = ind_level);
  std::string emitEndAlwaysff(unsigned level = ind_level);
  std::string emitCaseBegin(unsigned level = ind_level);
  std::string emitEndCase(unsigned level = ind_level);
  std::string emitEndModule(unsigned level = ind_level);

  virtual void initializePass();

  virtual void getAnalysisUsage(AnalysisUsage &AU) {
    AU.addRequired<TargetData>();
  }
};
} // end namespace

#endif
