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
#ifndef LLVM_GLOBAL_VAR_REG_H
#define LLVM_GLOBAL_VAR_REG_H

#include "llvm/Pass.h"
#include "llvm/Function.h"
#include "llvm/Instructions.h"
#include "llvm/Constants.h"
#include "llvm/LLVMContext.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/Support/Allocator.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Module.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/Support/CFG.h"
#include "llvm/DerivedTypes.h"

#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <map>

using namespace llvm;

using std::map;
using std::set;
using std::vector;
using std::pair;
using std::string;

namespace xVerilog {
    /*
     * This class holds a mapping between the names of the global variables and the
     * pointer to the actual data. This allows control of all of the global variables
     * in a single location. 
     */
  /// XXX: Implement as a ImmutablePass
class GVRegistry : public ImmutablePass {

public:
  static char ID;

  explicit GVRegistry() : ImmutablePass(&ID) {}
  ~GVRegistry();

  ConstantInt *getZero(unsigned NumBits) {
    return ConstantInt::get(IntegerType::get(getGlobalContext(), NumBits), 0);
  }

  ConstantInt *getOne(unsigned NumBits) {
    return ConstantInt::get(IntegerType::get(getGlobalContext(), NumBits), 1);
  }

  /*
  * add this variable to a list of instructions to be destructed on exit
  */
  // Use bumpAllXXX
  void trashWhenDone(Instruction* val) { CreateInsts.push_back(val); }

  /*
  * gets or creates a global variable which will be destructed on exit
  * @params number of bits, it this a pointer, default value, name
  */
  GlobalVariable* getGVByName(string varName, int bits = 32,
                              bool pointer = false, int val = 0);

  GlobalVariable* getGVByName(string varName, const Type* type);

  /*
  * Creates a type of the needed bit number. For example Int64Ty
  */
  const Type* bitNumToType(int bitnum);

  BumpPtrAllocator &getAllocator() { return GVRAllocator; }
private:
  std::map<string, GlobalVariable*> m_map;
  BumpPtrAllocator GVRAllocator;
  std::vector<Instruction*> CreateInsts;
};


} //end of namespace

//inline llvm::Instruction *operator new(size_t Size, xVerilog::globalVarRegistry &Allocator) {
//  
//}
////
//inline void operator delete(void *, xVerilog::globalVarRegistry &) {}

#endif // h guard
