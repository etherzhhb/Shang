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

def SDT_InArg : SDTypeProfile<1, 1, [SDTCisVT<1, i8>]>;
def SDT_FnRet : SDTypeProfile<0, -1, []>;

//  Specific Node Definitions.
//===----------------------------------------------------------------------===//
def VTMInArg	:	SDNode<"VTMISD::InArg", SDT_InArg, [SDNPHasChain]>;

def VTMFnRet	:	SDNode<"VTMISD::FnRet", SDT_FnRet,
							[SDNPVariadic, SDNPHasChain]>; 


//===----------------------------------------------------------------------===//
// Immediates
//===----------------------------------------------------------------------===//

def simm8 : PatLeaf<(imm), [{return isInt<3>(N->getSExtValue());}]>;
def uimm8 : PatLeaf<(imm), [{return isUInt<3>(N->getZExtValue());}]>;

let isReturn     = 1,
    isTerminator = 1,
    isBarrier    = 1  in
def VTMRet : InstVTM<(outs), (ins variable_ops), "ret;", [(VTMFnRet)]>;

multiclass InArgInsts {
  def i8 : InstVTM<(outs DR8:$arg), (ins i8imm:$arg_no), "arg $arg_no;",
                    [(set DR8:$arg, (VTMInArg uimm8:$arg_no))]>;
  def i16 : InstVTM<(outs DR16:$arg), (ins i8imm:$arg_no), "arg $arg_no;",
                    [(set DR16:$arg, (VTMInArg uimm8:$arg_no))]>;
  def i32 : InstVTM<(outs DR32:$arg), (ins i8imm:$arg_no), "arg $arg_no;",
                    [(set DR32:$arg, (VTMInArg uimm8:$arg_no))]>;
  def i64 : InstVTM<(outs DR64:$arg), (ins i8imm:$arg_no), "arg $arg_no;",
                    [(set DR64:$arg, (VTMInArg uimm8:$arg_no))]>;
}

let isBarrier	= 1 in
defm VTMArg : InArgInsts;

// TODO: Implement ADDE for add and sub. 
def VTMADD: InstVTM<(outs DR32:$dst), (ins DR32:$src1, DR32:$src2),
					"$dst = $src1 + $src2;",
					[(set DR32:$dst, (add DR32:$src1, DR32:$src2))]>;
