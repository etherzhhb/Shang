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

#include "RTLWriter.h"
#include "vbe/HWAtomPasses.h"

using namespace llvm;
using namespace esyn;

namespace {
struct TestbenchWriter : public FunctionPass{
  raw_ostream &Out;
  static char ID; 

  explicit TestbenchWriter(raw_ostream &O) : FunctionPass(ID),Out(O) {}

  TestbenchWriter() :FunctionPass(ID), Out(nulls()) {}

  void getAnalysisUsage(AnalysisUsage &AU) const {
    AU.addRequired<RTLWriter>();
    AU.setPreservesAll();
  }

  bool runOnFunction(Function &F);
};
}

char TestbenchWriter::ID = 0;

bool TestbenchWriter::runOnFunction(Function &F) {
  RTLWriter &RW = getAnalysis<RTLWriter>();
  VModule *VM = RW.getVerilogModule();

  Out << "module tb;\n\n";

  Out.indent(2) << "// DUT port driver.\n";

  for (VModule::port_iterator I = VM->ports_begin(), E = VM->ports_end();
       I != E; ++I) {
    VASTPort *port = *I;
    port->printExternalDriver(Out.indent(2));
    Out << '\n';
  }

  // Create the clock logic.
  Out << "\n\n";
  Out.indent(2) << "// Clock\n";
  Out.indent(2) <<  "always\n";
  Out.indent(4) <<    "#5ns clk = ~clk;\n\n\n";

  // And the initialize block.
  Out.indent(2) <<  "initial begin\n";
  Out.indent(4) <<    "integer starttime = 0;\n";
  Out.indent(4) <<    "#6ns rstN = 1'b1;\n";
  Out.indent(4) <<    "#5ns;\n";
  Out.indent(4) <<    "forever begin\n";

  for (VModule::port_iterator I = VM->ports_begin(), E = VM->ports_end();
       I != E; ++I) {
    VASTPort *port = *I;
    if (!port->isInput())
      continue;

    const std::string &Name = port->getName();
    if (Name == "clk" || Name == "rstN" || Name == "start")
      continue;

    Out.indent(6) << Name << " = $random();\n";
  }

  Out.indent(4) <<    "@(negedge clk) begin\n";
  Out.indent(6) <<      "start = 1'b1;\n";
  Out.indent(6) <<      "starttime = $time;\n";
  Out.indent(4) <<    "end\n";

  Out.indent(4) <<    "@(negedge clk) start <= 1'b0;\n";

  Out.indent(4) <<    "while (!fin) begin\n";
  Out.indent(6) <<      "#1ns;\n";
  Out.indent(6) <<      "if ($time > 100000) $finish();\n";
  Out.indent(4) <<    "end\n";

  Out.indent(4) <<    "$display(\"total time: %t\\n\", $time - starttime);\n";
  Out.indent(4) <<    "$finish();\n";

  Out.indent(4) <<    "end\n";
  Out.indent(2) << "end\n\n\n";

  // Design instance.
  Out.indent(2) << "// Design instance\n";
  Out.indent(2) << VM->getName() << " dut_" << VM->getName() << "(\n";
  Out.indent(6) << "." << VM->getPorts().front()->getName() << '('
                << VM->getPorts().front()->getName() << ')';
  for (VModule::port_iterator I = VM->ports_begin() + 1, E = VM->ports_end();
       I != E; ++I) {
      VASTPort *port = *I;
      Out << ",\n";
      Out.indent(6) << '.' << port->getName() << '(' << port->getName() << ')';
  }

  Out << ");\n";

  Out << "endmodule\n";

  return false;
}


Pass *esyn::createTestBenchWriterPass(raw_ostream &O) {
  return new TestbenchWriter(O);
}
