//===-- TestbenchWriter - Library for converting LLVM code to C ----------===//
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
// This class generate Testbench for verilog design automatically. 
//
//===----------------------------------------------------------------------===//

#include "TestbenchWriter.h"

#include "VLang.h"
#include "llvm/DerivedTypes.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/Support/ErrorHandling.h"

#define DEBUG_TYPE "vbe-tb-writer"
#include "llvm/Support/Debug.h"

#include <sstream>

using namespace llvm;
using namespace esyn;


char TestbenchWriter::ID = 0;

//TestbenchWriter -- testBench(Function&F,unsigned level)
std::string TestbenchWriter::testBech(Function &F) {
  raw_string_ostream ss(*(new std::string()));
  
  int k=0;
  int x=0;
  int y=0;
  
  ss<<"\n";
  ss<<"module tb;\n";

  ss<<"import \"DPI-C\" function ";
  printFunctionSignature( ss,&F);
  ss<<";\n";
  
  ss<<"\n";
  ss<<"reg clk = 1'b0;\n";
  ss<<"reg rstN = 1'b0;\n";
  ss<<"reg start = 1'b0;\n";
  ss<<"\n";
  ss<<"wire fin; \n";
  ss<<"wire [31:0] r1;\n";//??
  ss<<"reg[31:0]  r0 = 32'b0,";//??

//testbench initial
  for (Function::const_arg_iterator I = F.arg_begin(), E = F.arg_end();
      I != E; ++I) {
    x++;
    assert(!I->getNameStr().empty() && "Arg without name?");
    ss<<I->getNameStr();
    ss<<"=";
    const IntegerType *Ty = dyn_cast<IntegerType>(I->getType());
    //
    if (!Ty)
      continue;
    
    assert(Ty && "Expect Ty is int!");
    ss << Ty->getBitWidth() << "'b0";
    if (x!=F.arg_size())
      ss<<",";
  }

  ss<<";\n";
  ss<<"\n";
  ss<<"always\n";
  ss<<" #5ns clk = ~clk;\n";
  ss<<"\n";
  ss<<"initial \n begin\n";
  ss<<" #6ns rstN = 1'b1;\n";
  ss<<" #5ns ;\n";
  ss<<" forever begin\n";
  for (Function::const_arg_iterator I = F.arg_begin(), E = F.arg_end();I != E; ++I) {
  assert(!I->getNameStr().empty() && "Arg without name?");
  ss<<" "<<I->getNameStr();
  ss<<"<= $random();\n";
  }
  
  ss<<" @(negedge clk)\n";
  ss<<" start = 1'b1;\n";
  ss<<" @(negedge clk)\n";
  ss<<" start = 1'b0;\n";
  ss<<" @(negedge clk);\n";
  ss<<" @(negedge fin) ;\n";
  ss<<"\n";
  ss<<" r0 <= ";
  ss<<F.getNameStr();
  ss<<"(";
  for (Function::const_arg_iterator I = F.arg_begin(), E = F.arg_end();I != E; ++I) {
     y++;
    assert(!I->getNameStr().empty() && "Arg without name?");
    ss<<I->getNameStr();
    if (y!=F.arg_size())
      ss<<",";
  }
  ss<<");\n";
  ss<<" end\n";
  ss<<"end\n\n";
 
  ss<<F.getNameStr();
  ss<<" dut(";

  Function::const_arg_iterator I = F.arg_begin(), E = F.arg_end();
     
      std::string ArgName;
      for (; I != E; ++I) {
        ArgName = vlang->GetValueName(I);
        ss<<"."<<ArgName<<"(";
        assert(!I->getNameStr().empty() && "Arg without name?");
        ss<<I->getNameStr()<<"), ";
      }
        
  ss<<".return_value(r1),.clk(clk),.reset(rstN),.start(start),.fin(fin));\n\n";
  ss<<"endmodule\n";

  return ss.str();
}


//TestbenchWriter -- virtual bool runOnFunction(Function &F)
bool TestbenchWriter::runOnFunction(Function &F) {
  vlang = &getAnalysis<VLang>();
  // Dirty Hack
  Out<<testBech(F);

  return false;
}


//TestbenchWriter -- virtual void getAnalysisUsage(AnalysisUsage &AU) const;
void TestbenchWriter::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<VLang>();
}



//
static raw_ostream &printSimpleType(raw_ostream &Out, const Type *Ty, bool isSigned = false,
                         const std::string &NameSoFar = "") {
  assert((Ty->isPrimitiveType() || Ty->isIntegerTy() || Ty->isVectorTy()) && 
         "Invalid type for printSimpleType");
  switch (Ty->getTypeID()) {
  case Type::VoidTyID:   return Out << "void " << NameSoFar;
  case Type::IntegerTyID: {
    unsigned NumBits = cast<IntegerType>(Ty)->getBitWidth();
    if (NumBits == 1) 
      return Out << "bool " << NameSoFar;
    else if (NumBits <= 8)
      return Out << (isSigned?"signed":" ") << " char " << NameSoFar;
    else if (NumBits <= 16)
      return Out << (isSigned?"signed":" ") << " short " << NameSoFar;
    else if (NumBits <= 32)
      return Out << (isSigned?"signed":" ") << " int " << NameSoFar;
    else if (NumBits <= 64)
      return Out << (isSigned?"signed":" ") << " long long "<< NameSoFar;
    else { 
      assert(NumBits <= 128 && "Bit widths > 128 not implemented yet");
      return Out << (isSigned?"llvmInt128":"llvmUInt128") << " " << NameSoFar;
    }
  }
  default:
#ifndef NDEBUG
    errs() << "Unknown primitive type: " << *Ty << "\n";
#endif
    llvm_unreachable(0);
  }
}

//TestbenchWriter -- void printFunctionSignature(raw_ostream &Out, const Function *F)
void TestbenchWriter::printFunctionSignature(raw_ostream &Out,
                                              const Function *F) {
  /// isStructReturn - Should this function actually return a struct by-value?
  bool isStructReturn = F->hasStructRetAttr();
  
  if (F->hasLocalLinkage()) Out << "static ";
  if (F->hasDLLImportLinkage()) Out << "__declspec(dllimport) ";
  if (F->hasDLLExportLinkage()) Out << "__declspec(dllexport) ";  
  switch (F->getCallingConv()) {
   case CallingConv::X86_StdCall:
    Out << "__attribute__((stdcall)) ";
    break;
   case CallingConv::X86_FastCall:
    Out << "__attribute__((fastcall)) ";
    break;
   default:
    break;
  }
  
  // Loop over the arguments, printing them...
  const FunctionType *FT = cast<FunctionType>(F->getFunctionType());
  const AttrListPtr &PAL = F->getAttributes();

  std::string tstr;
  raw_string_ostream FunctionInnards(tstr);

  // Print out the name...
  FunctionInnards << vlang->GetValueName(F) << '(';

  bool PrintedArg = false;
  if (!F->isDeclaration()) {
    if (!F->arg_empty()) {
      Function::const_arg_iterator I = F->arg_begin(), E = F->arg_end();
      unsigned Idx = 1;
      
      // If this is a struct-return function, don't print the hidden
      // struct-return argument.
      if (isStructReturn) {
        assert(I != E && "Invalid struct return function!");
        ++I;
        ++Idx;
      }
      
      std::string ArgName;
      for (; I != E; ++I) {
        if (PrintedArg) FunctionInnards << ", ";
        if (I->hasName())
          ArgName = vlang->GetValueName(I);
        else
          ArgName = "";
        const Type *ArgTy = I->getType();
        //if (PAL.paramHasAttr(Idx, Attribute::ByVal)) {
        //  ArgTy = cast<PointerType>(ArgTy)->getElementType();
        //  ByValParams.insert(I);
        //}
        printSimpleType(FunctionInnards, ArgTy,
            /*isSigned=*/PAL.paramHasAttr(Idx, Attribute::SExt),
            ArgName);
        PrintedArg = true;
        ++Idx;
      }
    }
  } else {
    // Loop over the arguments, printing them.
    FunctionType::param_iterator I = FT->param_begin(), E = FT->param_end();
    unsigned Idx = 1;
    
    // If this is a struct-return function, don't print the hidden
    // struct-return argument.
    if (isStructReturn) {
      assert(I != E && "Invalid struct return function!");
      ++I;
      ++Idx;
    }
    
    for (; I != E; ++I) {
      if (PrintedArg) FunctionInnards << ", ";
      const Type *ArgTy = *I;
      if (PAL.paramHasAttr(Idx, Attribute::ByVal)) {
        assert(ArgTy->isPointerTy());
        ArgTy = cast<PointerType>(ArgTy)->getElementType();
      }
      printSimpleType(FunctionInnards, ArgTy,
             /*isSigned=*/PAL.paramHasAttr(Idx, Attribute::SExt));
      PrintedArg = true;
      ++Idx;
    }
  }

  if (!PrintedArg && FT->isVarArg()) {
    FunctionInnards << "int vararg_dummy_arg";
    PrintedArg = true;
  }

  // Finish printing arguments... if this is a vararg function, print the ...,
  // unless there are no known types, in which case, we just emit ().
  //
  if (FT->isVarArg() && PrintedArg) {
    FunctionInnards << ",...";  // Output varargs portion of signature!
  } else if (!FT->isVarArg() && !PrintedArg) {
    FunctionInnards << "void"; // ret() -> ret(void) in C.
  }
  FunctionInnards << ')';
  
  // Get the return tpe for the function.
  const Type *RetTy;
  if (!isStructReturn)
    RetTy = F->getReturnType();
  else {
    // If this is a struct-return function, print the struct-return type.
    RetTy = cast<PointerType>(FT->getParamType(0))->getElementType();
  }
    
  // Print out the return type and the signature built above.
  printSimpleType(Out, RetTy, 
            /*isSigned=*/PAL.paramHasAttr(0, Attribute::SExt),
            FunctionInnards.str());
}






