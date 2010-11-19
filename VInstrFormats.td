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
def FUMemBus	: FUType<1, IIC_MemBus>;
def FUSHL		: FUType<2, IIC_SHL>;
def FUASR		: FUType<3, IIC_ASR>;
def FULSR		: FUType<4, IIC_LSR>;
def FUAddSub	: FUType<5, IIC_AddSub>;
def FUMult		: FUType<6, IIC_Mult>;

// Pseudo instructions
class InstVTM<dag outs, dag ins, string asmstr, list<dag> pattern, FUType FU,
			  bit datapath = 0, bits<4> latency = 0, 
			  bit readAtEmit = 0, bit writeUntilFinish = 0>
  : Instruction {
  let Namespace = "VTM";

  dag OutOperandList = outs;
  dag InOperandList = ins;
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

class DPFUInst<dag outs, dag ins, string asmstr, list<dag> pattern,
                FUType ResType,
                bit readAtEmit = 0, bit writeUntilFinish = 0>
	: InstVTM<outs, ins, asmstr, pattern, ResType, 1, 0,
			  readAtEmit, writeUntilFinish>;

class DPTvaInst<dag outs, dag ins, string asmstr, list<dag> pattern,
		        bits<4> latency = 0,
			      bit readAtEmit = 0, bit writeUntilFinish = 0>
	: InstVTM<outs, ins, asmstr, pattern, FUTrivial, 1, latency,
			  readAtEmit, writeUntilFinish>;

class BinOpFrag<dag res> : PatFrag<(ops node:$LHS, node:$RHS), res>;

class UnOpFrag <dag res> : PatFrag<(ops node:$Src), res>;
