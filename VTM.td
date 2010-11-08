//===------ VTM.td - Describe the Verilog Target Machine ---*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Target-independent interfaces which we are implementing
//===----------------------------------------------------------------------===//

include "llvm/Target/Target.td"


def FeatureGeneric : SubtargetFeature<"generic", "vtmattr", "true",
									  "Feature description">;

include "VSchedule.td"
//===----------------------------------------------------------------------===//
// VTM supported processors.
//===----------------------------------------------------------------------===//
class Proc<string Name, list<SubtargetFeature> Features>
 : Processor<Name, VTMItineraries, Features>;

def : Proc<"generic",         [FeatureGeneric]>;

//===----------------------------------------------------------------------===//
// Register File, Calling Conv, Instruction Descriptions
//===----------------------------------------------------------------------===//

include "VRegisterInfo.td"

include "VInstrInfo.td"

def VInstrInfo : InstrInfo {}

//===----------------------------------------------------------------------===//
// Declare the target which we are implementing
//===----------------------------------------------------------------------===//

def VTM : Target {
  // Pull in Instruction Info:
  let InstructionSet = VInstrInfo;
}
