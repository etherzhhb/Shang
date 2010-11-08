//===- VTMSchedule.td - VTM Scheduling Definitions ---------*- tablegen -*-===//
// 
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
// 
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Instruction Itinerary classes used for ARM
//
def IIC_Trivial     : InstrItinClass;
def IIC_MemBus      : InstrItinClass;
def IIC_SHL			: InstrItinClass;
def IIC_ASR			: InstrItinClass;
def IIC_LSR			: InstrItinClass;
def IIC_AddSub		: InstrItinClass;
def IIC_Mult		: InstrItinClass;

// Functional units
def FU_Trivial	: FuncUnit;
def FU_MemBus	: FuncUnit;
def FU_SHL		: FuncUnit;
def FU_ASR		: FuncUnit;
def FU_LSR		: FuncUnit;
def FU_AddSub	: FuncUnit;
def FU_Mult		: FuncUnit;


//===----------------------------------------------------------------------===//
//

def VTMItineraries : ProcessorItineraries<
  [FU_Trivial, FU_MemBus, FU_SHL, FU_ASR, FU_LSR, FU_AddSub, FU_Mult], [], [
  //
  InstrItinData<IIC_Trivial, [InstrStage<1, [FU_Trivial]>]>,
  InstrItinData<IIC_MemBus,	[InstrStage<1, [FU_MemBus]>]>,
  InstrItinData<IIC_SHL,	[InstrStage<1, [FU_SHL]>]>,
  InstrItinData<IIC_ASR,	[InstrStage<1, [FU_ASR]>]>,
  InstrItinData<IIC_LSR,	[InstrStage<1, [FU_LSR]>]>,
  InstrItinData<IIC_AddSub,	[InstrStage<1, [FU_AddSub]>]>,
  InstrItinData<IIC_Mult,	[InstrStage<1, [FU_Mult]>]>
]>;
