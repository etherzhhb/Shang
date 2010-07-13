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
#include "llvm/DerivedTypes.h"
#include "llvm/ADT/StringExtras.h"
//#include "llvm/Argument.h"

#define DEBUG_TYPE "vbe-tb-writer"
#include "llvm/Support/Debug.h"

#include <sstream>

using namespace llvm;
using namespace esyn;

std::string TestbenchWriter::testBech(Function&F,unsigned level) {
  std::stringstream ss;
  
  int k=0;
  int x=0;
  int y=0;
  
  ss<<"module tb;\n";

  const Type *RetTy = F.getReturnType(); 
  ss<<"import \"DIP-C\" funtion "<<F.getReturnType()<<" "<<F.getNameStr()<<"(";

  if (!F.arg_empty()) {
    for (Function::const_arg_iterator I = F.arg_begin(), E = F.arg_end();
      I != E; ++I) {
        k++;
        Argument *Arg=J;
        
        ss<<J->getType()<<" ";
     
        assert(!I->getNameStr().empty() && "Arg without name?");
        ss<<I->getNameStr();
        if (k!=F.arg_size())
          ss<<",";
    }
  }
  ss<<");\n";

  ss<<"reg clk = 1'b0,\n";
  ss<<"   rstN = 1'b0,\n";
  ss<<"  start = 1'b0,\n";
  ss<<"    fin = 1'b0;\n";
  ss<<"wire [31:0] r1;\n";//??
  ss<<"reg[31:0]  r0 = 32'b0,";//??
  //why reg clk,rstN,start,wire[31:0] r1,reg[31:0] r0?? Are they common? Or should be common??
  //clk,rst,start,fin maybe always needed
  
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
  ss<<"always\n";
  ss<<" #5ns clk = ~clk;\n";
  ss<<"initial begin\n";
  ss<<" #6ns rstN = 1'b1;\n";
  ss<<"end\n";
  ss<<" always begin\n";
  ss<<" @(negedge clk) begin\n";

  for (Function::const_arg_iterator I = F.arg_begin(), E = F.arg_end();I != E; ++I) {
    assert(!I->getNameStr().empty() && "Arg without name?");
    ss<<I->getNameStr();
    ss<<"<= $random();\n";
  }
  ss<<"end\n";

  for(unsigned i = 0; i < level; ++i)//??
    ss<<"@(negedge clk)\n;\n";

  ss<<"@(negedge clk)\n";
  ss<<"r0 <= add(";
  
  for (Function::const_arg_iterator I = F.arg_begin(), E = F.arg_end();I != E; ++I) {
    y++;
    assert(!I->getNameStr().empty() && "Arg without name?");
    ss<<I->getNameStr();
    if (y!=F.arg_size())
      ss<<",";
  }

  ss<<");\n";
  ss<<"end\n";
  ss<<F.getReturnType();
  ss<<" dut(";

  for (Function::const_arg_iterator I = F.arg_begin(), E = F.arg_end();I != E; ++I) {
    ss<<".";
    assert(!I->getNameStr().empty() && "Arg without name?");
    ss<<I->getNameStr()<<"(";
    ss<<I->getNameStr()<<"),";
  }

  ss<<".clk(clk),.reset(rstN),start(start),.fin(fin),.Ret_add(r1));\n";
  ss<<"endmodule\n";

  return ss.str();
}

char TestbenchWriter::ID = 0;
