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
// The corresponding Scheduling Unit type of the instruction.
class SUType<bits<2> val> {
  bits<2> Value = val;
}

def SUOthers	: SUType<0>;
def SUOpFU		: SUType<1>;
def SUVRoot		: SUType<2>;

// The corresponding Function Unit type of the instruction.
class FUType<bits<3> val> {
  bits<3> Value = val;
}

def FUMemBus	: FUType<1>;
def FUSHL		: FUType<2>;
def FUASR		: FUType<3>;
def FULSR		: FUType<4>;
def FUAddSub	: FUType<5>;
def FUMult		: FUType<6>;
def FUTrivial	: FUType<7>;

// Pseudo instructions
class InstVTM<dag outs, dag ins, string asmstr, list<dag> pattern,
              SUType SU, FUType FU>
  : Instruction {
  field bits<32> Inst;

  let Namespace = "VTM";

  dag OutOperandList = outs;
  dag InOperandList = ins;
  let AsmString   = asmstr;
  let Pattern = pattern;

  SUType su = SU;
  FUType fu = FU;

  let TSFlags{1-0} = su.Value;
  let TSFlags{4-2} = fu.Value;
}

class InstPassive<dag outs, dag ins, string asmstr, list<dag> pattern>
	: InstVTM<outs, ins, asmstr, pattern, SUOthers, FUTrivial>;

class InstOpFU<dag outs, dag ins, string asmstr, list<dag> pattern,
               FUType ResType>
	: InstVTM<outs, ins, asmstr, pattern, SUOpFU, ResType>;

class InstVRoot<dag outs, dag ins, string asmstr, list<dag> pattern>
	: InstVTM<outs, ins, asmstr, pattern, SUVRoot, FUTrivial>;
