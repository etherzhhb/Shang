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

def FUMemBus	: FUType<1, IIC_MemBus>;
def FUSHL		: FUType<2, IIC_SHL>;
def FUASR		: FUType<3, IIC_ASR>;
def FULSR		: FUType<4, IIC_LSR>;
def FUAddSub	: FUType<5, IIC_AddSub>;
def FUMult		: FUType<6, IIC_Mult>;
def FUTrivial	: FUType<7, IIC_Trivial>;

// Pseudo instructions
class InstVTM<dag outs, dag ins, string asmstr, list<dag> pattern, FUType FU,
			  bits<4> latency = 0, bit readAtEmit = 0>
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
  
  let TSFlags{7-4} = latency;
}

class InstPassive<dag outs, dag ins, string asmstr, list<dag> pattern,
				  bits<4> latency = 0, bit readAtEmit = 0>
	: InstVTM<outs, ins, asmstr, pattern, FUTrivial, latency, readAtEmit>;

class InstActive<dag outs, dag ins, string asmstr, list<dag> pattern,
               FUType ResType, bit readAtEmit = 0>
	: InstVTM<outs, ins, asmstr, pattern, ResType, 0, readAtEmit>;
