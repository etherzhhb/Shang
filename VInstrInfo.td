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
def SDT_InArg : SDTypeProfile<1, 1, [SDTCisVT<1, i8>]>;
def VTMInArg	:	SDNode<"VTMISD::InArgDAG", SDT_InArg, [SDNPHasChain]>;

def SDT_RetVal : SDTypeProfile<0, 2, [SDTCisVT<1, i8>]>;
def VTMRetVal	:	SDNode<"VTMISD::RetValDAG", SDT_RetVal, [SDNPHasChain]>;

def SDT_FnRet : SDTypeProfile<0, 0, []>;
def VTMFnRet	:	SDNode<"VTMISD::RetDAG", SDT_FnRet,
							[SDNPVariadic, SDNPHasChain]>;

def SDT_ADDC : SDTypeProfile<2, 3, [
  SDTCisSameAs<0, 2>, SDTCisSameAs<0, 3>, SDTCisInt<0>,
  SDTCisVT<1, i1>,  SDTCisVT<4, i1>
]>;
def VTMADD		:	SDNode<"VTMISD::ADDDAG", SDT_ADDC>;

def SDT_LdConst : SDTypeProfile<1, 1, [SDTCisInt<0>, SDTCisSameAs<0, 1>]>;
def VTMLdConst	:	SDNode<"VTMISD::LdConst", SDT_LdConst>;

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
  def i1 : InstPassive<(outs DR1:$arg), (ins i8imm:$arg_no), "arg $arg_no;",
                    [(set DR1:$arg, (VTMInArg uimm8:$arg_no))], 0, 1>;
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
  def i1 : InstPassive<(outs), (ins DR1:$var, i8imm:$ret_no), "ret $var at $ret_no;",
                    [(VTMRetVal DR1:$var, uimm8:$ret_no)], 0, 1>;
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

multiclass LdConstOps {
  def i1  : InstPassive<(outs DR1:$r), (ins i1imm:$c), "$r = $c;",
                        [(set DR1:$r, imm:$c)]>;
  def i8  : InstPassive<(outs DR8:$r), (ins i8imm:$c), "$r = $c;",
                        [(set DR8:$r, imm:$c)]>;
  def i16 : InstPassive<(outs DR16:$r), (ins i16imm:$c), "$r = $c;",
                        [(set DR16:$r, imm:$c)]>;
  def i32 : InstPassive<(outs DR32:$r), (ins i32imm:$c), "$r = $c;",
                        [(set DR32:$r, imm:$c)]>;
  def i64 : InstPassive<(outs DR64:$r), (ins i64imm:$c), "$r = $c;",
                        [(set DR64:$r, imm:$c)]>;
}

let isReMaterializable = 1,
    isAsCheapAsAMove   = 1 in 
defm VOpLdConst : LdConstOps;

class BinOpFrag<dag res> : PatFrag<(ops node:$LHS, node:$RHS), res>;

let isCommutable = 1 in
defm VOpXor	: ActiveBinOps<"xor", BinOpFrag<(xor node:$LHS, node:$RHS)>,
                           FUTrivial>;


// TODO: Implement ADDE for add and sub.
def VOpADD : InstActive<(outs DR32:$dst, DR1:$co),
						(ins DR32:$src1, DR32:$src2, DR1:$ci),
					    "{$dst, $co} = $src1 + $src2 + $ci;", [], FUAddSub>;

let isBarrier    = 1 in
def VOpBundle : InstPassive<(outs variable_ops), (ins i32imm:$slot, variable_ops),
							 "bundle;", []>;

// Opcode for micro operation.
def VOpWriteReg : InstPassive<(outs), (ins), "write register;", []>;
