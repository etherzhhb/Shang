//===---------- VInstrInfo.td - VTM Instruction defs -----------*- tblgen-*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source 
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file describes the VTM instructions in TableGen format.
//
//===----------------------------------------------------------------------===//

include "VInstrFormats.td"

//===----------------------------------------------------------------------===//
// Type Constraints.
//===----------------------------------------------------------------------===//


//  Specific Node Definitions.
//===----------------------------------------------------------------------===//
def VTMInArg	:	SDNode<"VTMISD::InArg", SDTIntUnaryOp, [SDNPHasChain]>;

def VTMFnRet	:	SDNode<"VTMISD::FnRet", SDTNone, [SDNPHasChain]>;


// TODO: multiclass
let isReturn     = 1,
    isTerminator = 1,
    isBarrier    = 1  in
def VTMRet: InstVTM<(outs), (ins), "ret;", [(VTMFnRet)]>;
