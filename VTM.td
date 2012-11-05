//===------ VTM.td - Describe the Verilog Target Machine ---*- tablegen -*-===//
//
//                      The Shang HLS frameowrk                               //
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

//===----------------------------------------------------------------------===//
// Register File, Calling Conv, Instruction Descriptions
//===----------------------------------------------------------------------===//

include "VRegisterInfo.td"
include "VIntrinsics.td"
include "VInstrInfo.td"

def VInstrInfo : InstrInfo {}

//===----------------------------------------------------------------------===//
// Declare the target which we are implementing
//===----------------------------------------------------------------------===//

def VTM : Target {
  // Pull in Instruction Info:
  let InstructionSet = VInstrInfo;
}
