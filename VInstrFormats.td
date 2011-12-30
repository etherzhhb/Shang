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

// The corresponding Function Unit type of the instruction.
class FUType<bits<4> val> {
  bits<4> Value = val;
}

def FUTrivial	: FUType<0>;
def FUAddSub	: FUType<1>;
def FUSHIFT		: FUType<2>;
def FUMult		: FUType<3>;
def FUICmp		: FUType<4>;
def FUMemBus	: FUType<5>;
def FUBRam		: FUType<6>;
def FUMUX		: FUType<7>;
def FUCalleeFN	: FUType<8>;

// Bit-width information operand.
def BitWidthAnnotator : PredicateOperand<i64, (ops DR), (ops (i64 zero_reg))>
 {}

// Trace information operand.
def TraceNum : PredicateOperand<i64, (ops DR), (ops (i64 0))> {}

// Pseudo instructions
class InstVTM<dag oops, dag iops, string asmstr, list<dag> pattern, FUType FU,
			  bit datapath = 0, bits<4> latency = 0,
			  bit readAtEmit = 0, bit writeUntilFinish = 0>
  : Instruction {
  let Namespace = "VTM";

  dag OutOperandList = oops;
  // Append the bit-width flag.
  // TODO: We need predicate as well.
  dag InOperandList = !con(iops, (ins BitWidthAnnotator:$Annotator,
									  TraceNum:$Trace));
  let isPredicable = 1;
  let AsmString   = asmstr;
  let Pattern = pattern;

  FUType fu = FU;

  let TSFlags{3-0} = fu.Value;

  let TSFlags{4} = readAtEmit;
  let TSFlags{5} = writeUntilFinish;
  let TSFlags{6} = datapath;
}

class CtrlInst<dag outs, dag ins, string asmstr, list<dag> pattern,
			   bits<4> latency = 0, bit writeUntilFinish = 0>
	: InstVTM<outs, ins, asmstr, pattern, FUTrivial, 0, latency,
			  1, writeUntilFinish>;

class FUInst<dag outs, dag ins, string asmstr, list<dag> pattern,
                FUType ResType, bit writeUntilFinish = 0>
	: InstVTM<outs, ins, asmstr, pattern, ResType, 0, 0,
			  1, writeUntilFinish>;

class DPTvaInst<dag outs, dag ins, string asmstr, list<dag> pattern>
	: InstVTM<outs, ins, asmstr, pattern, FUTrivial, 1, 0, 0, 0>;

class BinOpFrag<dag res> : PatFrag<(ops node:$LHS, node:$RHS), res>;

class UnOpFrag <dag res> : PatFrag<(ops node:$Src), res>;
