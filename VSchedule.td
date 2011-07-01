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
def IIC_AddSub		  : InstrItinClass;
def IIC_SHIFT	      : InstrItinClass;
def IIC_Mult		    : InstrItinClass;
def IIC_MemBus      : InstrItinClass;
def IIC_BRam		    : InstrItinClass;
def IIC_CalleeFN    : InstrItinClass;

// Functional units
def FU_Trivial	: FuncUnit;
def FU_AddSub	  : FuncUnit;
def FU_SHIFT	  : FuncUnit;
def FU_Mult		  : FuncUnit;
def FU_MemBus	  : FuncUnit;
def FU_BRam	    : FuncUnit;
def FU_CalleeFN : FuncUnit;


//===----------------------------------------------------------------------===//
//

def VTMItineraries : ProcessorItineraries<
  [FU_Trivial, FU_AddSub, FU_SHIFT, FU_Mult, FU_MemBus, FU_BRam, FU_CalleeFN],
  [],
  [
  InstrItinData<IIC_Trivial,  [InstrStage<1, [FU_Trivial]>]>,
  InstrItinData<IIC_AddSub,	  [InstrStage<1, [FU_AddSub]>]>,
  InstrItinData<IIC_Mult,	    [InstrStage<1, [FU_Mult]>]>,
  InstrItinData<IIC_SHIFT,	  [InstrStage<1, [FU_SHIFT]>]>,
  InstrItinData<IIC_MemBus,	  [InstrStage<1, [FU_MemBus]>]>,
  InstrItinData<IIC_BRam,	    [InstrStage<1, [FU_BRam]>]>,
  InstrItinData<IIC_CalleeFN, [InstrStage<1, [FU_CalleeFN]>]>
]>;
