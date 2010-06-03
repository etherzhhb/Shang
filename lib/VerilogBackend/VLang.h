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
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

namespace xVerilog {
class VLang : public ImmutablePass {
  const TargetData* TD;
  Mangler* Mang;
  // For unname value
  DenseMap<const Value*, unsigned> AnonValueNumbers;
  unsigned NextAnonValueNumber;

  void clear() {
    AnonValueNumbers.clear();
  }
public:
  static char ID;
  explicit VLang() : ImmutablePass(&ID) {}

  /// @name Value and Type printing
  //{
  /// @brief Get the name of Value, if the Value have no name, 
  ///       just create one for it.
  ///
  /// @param Operand The Value to get the name.
  ///
  /// @return The unique name of the Value.
  std::string GetValueName(const Value *Operand); 

  static std::string VLangMangle(const std::string &S);

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

  virtual void initializePass();

  virtual void getAnalysisUsage(AnalysisUsage &AU) {
    AU.addRequired<TargetData>();
  }
};
} // end namespace

#endif