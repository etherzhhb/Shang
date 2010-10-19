//===---------- VInstrFormats.td - VTM Instruction Formats-----*- tblgen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source 
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
//  Describe CTV instructions format here
//
// Pseudo instructions
class InstVTM<dag outs, dag ins, string asmstr, list<dag> pattern>
  : Instruction {
  field bits<32> Inst;

  let Namespace = "VTM";

  dag OutOperandList = outs;
  dag InOperandList = ins;
  let AsmString   = asmstr;
  let Pattern = pattern;
}
