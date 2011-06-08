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
class FUType<bits<3> val, InstrItinClass InstItin> {
  bits<3> Value = val;
  InstrItinClass Itinerary = InstItin;
}

def FUTrivial	: FUType<0, IIC_Trivial>;
def FUAddSub	: FUType<1, IIC_AddSub>;
def FUSHIFT		: FUType<2, IIC_SHIFT>;
def FUMult		: FUType<3, IIC_Mult>;
def FUMemBus	: FUType<4, IIC_MemBus>;
def FUBRam	  : FUType<5, IIC_BRam>;
def FUCalleeFN: FUType<6, IIC_CalleeFN>;

// Bit-width information operand.
// Dirty Hack: This is not a Predicate Operand.
def BitWidthAnnotator : PredicateOperand<i64, (ops PredR), (ops (i64 zero_reg))>
 {}

// Pseudo instructions
class InstVTM<dag oops, dag iops, string asmstr, list<dag> pattern, FUType FU,
			  bit datapath = 0, bits<4> latency = 0, 
			  bit readAtEmit = 0, bit writeUntilFinish = 0>
  : Instruction {
  let Namespace = "VTM";

  dag OutOperandList = oops;
  // Append the bit-width flag.
  // TODO: We need predicate as well.
  dag InOperandList = !con(iops, (ins BitWidthAnnotator:$Annotator));
  let isPredicable = 1;
  let AsmString   = asmstr;
  let Pattern = pattern;

  FUType fu = FU;

  let TSFlags{2-0} = fu.Value;
  let Itinerary = fu.Itinerary;
  
  let TSFlags{3} = readAtEmit;
  let TSFlags{4} = writeUntilFinish;
  let TSFlags{5} = datapath;
  
  let TSFlags{9-6} = latency;
}

class CtrlInst<dag outs, dag ins, string asmstr, list<dag> pattern,
			   bits<4> latency = 0,
			   bit readAtEmit = 0, bit writeUntilFinish = 0>
	: InstVTM<outs, ins, asmstr, pattern, FUTrivial, 0, latency,
			  readAtEmit, writeUntilFinish>;

class FUInst<dag outs, dag ins, string asmstr, list<dag> pattern,
                FUType ResType,
                bit readAtEmit = 0, bit writeUntilFinish = 0,
                bit datapath = 1>
	: InstVTM<outs, ins, asmstr, pattern, ResType, datapath, 0,
			  readAtEmit, writeUntilFinish>;

class DPTvaInst<dag outs, dag ins, string asmstr, list<dag> pattern,
		        bits<4> latency = 0,
			      bit readAtEmit = 0, bit writeUntilFinish = 0>
	: InstVTM<outs, ins, asmstr, pattern, FUTrivial, 1, latency,
			  readAtEmit, writeUntilFinish>;

class BinOpFrag<dag res> : PatFrag<(ops node:$LHS, node:$RHS), res>;

class UnOpFrag <dag res> : PatFrag<(ops node:$Src), res>;
