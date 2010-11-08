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
def SDT_RetVal : SDTypeProfile<0, 2, [SDTCisVT<1, i8>]>;
def SDT_FnRet : SDTypeProfile<0, 0, []>;

//  Specific Node Definitions.
//===----------------------------------------------------------------------===//
def VTMInArg	:	SDNode<"VTMISD::InArg", SDT_InArg, [SDNPHasChain]>;

def VTMRetVal	:	SDNode<"VTMISD::RetVal", SDT_RetVal, [SDNPHasChain]>;

def VTMFnRet	:	SDNode<"VTMISD::FnRet", SDT_FnRet,
							[SDNPVariadic, SDNPHasChain]>;


//===----------------------------------------------------------------------===//
// Immediates
//===----------------------------------------------------------------------===//

def simm8 : PatLeaf<(imm), [{return isInt<3>(N->getSExtValue());}]>;
def uimm8 : PatLeaf<(imm), [{return isUInt<3>(N->getZExtValue());}]>;

let isReturn     = 1,
    isTerminator = 1,
    isBarrier    = 1 in
def VOpRet : InstPassive<(outs), (ins), "ret;", [(VTMFnRet)]>;

multiclass InArgOps {
  def i8 : InstPassive<(outs DR8:$arg), (ins i8imm:$arg_no), "arg $arg_no;",
                    [(set DR8:$arg, (VTMInArg uimm8:$arg_no))], 0, 1>;
  def i16 : InstPassive<(outs DR16:$arg), (ins i8imm:$arg_no), "arg $arg_no;",
                    [(set DR16:$arg, (VTMInArg uimm8:$arg_no))], 0, 1>;
  def i32 : InstPassive<(outs DR32:$arg), (ins i8imm:$arg_no), "arg $arg_no;",
                    [(set DR32:$arg, (VTMInArg uimm8:$arg_no))], 0, 1>;
  def i64 : InstPassive<(outs DR64:$arg), (ins i8imm:$arg_no), "arg $arg_no;",
                    [(set DR64:$arg, (VTMInArg uimm8:$arg_no))], 0, 1>;
}

defm VOpArg : InArgOps;

multiclass RetValOps {
  def i8 : InstPassive<(outs), (ins DR8:$var, i8imm:$ret_no), "ret $var at $ret_no;",
                    [(VTMRetVal DR8:$var, uimm8:$ret_no)], 0, 1>;
  def i16 : InstPassive<(outs), (ins DR16:$var, i8imm:$ret_no), "ret $var at $ret_no;",
                    [(VTMRetVal DR16:$var, uimm8:$ret_no)], 0, 1>;
  def i32 : InstPassive<(outs), (ins DR32:$var, i8imm:$ret_no), "ret $var at $ret_no;",
                    [(VTMRetVal DR32:$var, uimm8:$ret_no)], 0, 1>;
  def i64 : InstPassive<(outs), (ins DR64:$var, i8imm:$ret_no), "ret $var at $ret_no;",
                    [(VTMRetVal DR64:$var, uimm8:$ret_no)], 0, 1>;
}

let hasSideEffects	= 1,
    isBarrier		= 1  in
defm VOpRetVal : RetValOps;

// TODO: Implement ADDE for add and sub.
def VOpADD : InstActive<(outs DR32:$dst), (ins DR32:$src1, DR32:$src2),
					  "$dst = $src1 + $src2;",
					  [(set DR32:$dst, (add DR32:$src1, DR32:$src2))], FUAddSub>;

let isBarrier    = 1 in
def VOpBundle : InstPassive<(outs variable_ops), (ins variable_ops), "bundle;",
                            []>;
