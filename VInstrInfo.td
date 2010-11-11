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

//===----------------------------------------------------------------------===//
// Communication operations.
let isReturn     = 1,
    isTerminator = 1,
    isBarrier    = 1 in
def VOpRet : InstPassive<(outs), (ins), "ret;", [(VTMFnRet)]>;

def VOpArg : InstPassive<(outs dyn_rc:$arg), (ins i8imm:$arg_no),
		 				"$arg = arg[$arg_no]", [], 0, 1>;

let hasSideEffects	= 1,
    isBarrier		= 1  in
def VOpRetVal : InstPassive<(outs), (ins dyn_rc:$var, i8imm:$ret_no),
							 "ret $var at $ret_no;",
							 [], 0, 1>;

let isCommutable       = 1,
	isReMaterializable = 1,
    isAsCheapAsAMove   = 1 in {
defm VOpXor	: InstBinOps<"xor", BinOpFrag<(xor node:$LHS, node:$RHS)>,
                         FUTrivial>;

}

//===----------------------------------------------------------------------===//
// Computation operations.
defm VOpSHL	: InstShOps<"shl", BinOpFrag<(shl node:$LHS, node:$RHS)>,
                         FUSHL>;

multiclass AddOps {
  def i1  : InstActive<(outs DR1:$dst, DR1:$co),
			           (ins DR1:$src1, DR1:$src2, DR1:$ci),
					   "{$dst, $co} = $src1 + $src2 + $ci;", [], FUAddSub>;
  def i8  : InstActive<(outs DR8:$dst, DR1:$co),
			           (ins DR8:$src1, DR8:$src2, DR1:$ci),
					   "{$dst, $co} = $src1 + $src2 + $ci;", [], FUAddSub>;
  def i16  : InstActive<(outs DR16:$dst, DR1:$co),
			           (ins DR16:$src1, DR16:$src2, DR1:$ci),
					   "{$dst, $co} = $src1 + $src2 + $ci;", [], FUAddSub>;
  def i32  : InstActive<(outs DR32:$dst, DR1:$co),
			           (ins DR32:$src1, DR32:$src2, DR1:$ci),
					   "{$dst, $co} = $src1 + $src2 + $ci;", [], FUAddSub>;
  def i64  : InstActive<(outs DR64:$dst, DR1:$co),
			           (ins DR64:$src1, DR64:$src2, DR1:$ci),
					   "{$dst, $co} = $src1 + $src2 + $ci;", [], FUAddSub>;
}

defm VOpAdd : AddOps;


multiclass LoadOps {
  def i1    :  InstActive<(outs DR1:$dst), (ins ptr_rc:$addr),
			     	      "$dst = load $addr",
						  [(set DR1:$dst, (load ptr_rc:$addr))],
						  FUMemBus, 1>;
  def i8    :  InstActive<(outs DR8:$dst), (ins ptr_rc:$addr),
						  "$dst = load $addr",
						  [(set DR8:$dst, (load ptr_rc:$addr))],
						  FUMemBus, 1>;
  def i16   :  InstActive<(outs DR16:$dst), (ins ptr_rc:$addr),
						  "$dst = load $addr",
						  [(set DR16:$dst, (load ptr_rc:$addr))],
						  FUMemBus, 1>;
  def i32   :  InstActive<(outs DR32:$dst), (ins ptr_rc:$addr),
						  "$dst = load $addr",
						  [(set DR32:$dst, (load ptr_rc:$addr))],
						  FUMemBus, 1>;
  def i64   :  InstActive<(outs DR64:$dst), (ins ptr_rc:$addr),
						  "$dst = load $addr",
						  [(set DR64:$dst, (load ptr_rc:$addr))],
						  FUMemBus, 1>;
}

let canFoldAsLoad      = 1 in
defm VOpLoad : LoadOps;

//===----------------------------------------------------------------------===//
// FIXME: Perform bit level optimization on these operations.
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

//multiclass ExtOps {}

let isReMaterializable = 1,
    isAsCheapAsAMove   = 1 in {
def VOpCast : InstPassive<(outs dyn_rc:$dst), (ins dyn_rc:$src, i1imm:$signed),
						  "$dst = ext $src, $signed", []>;
}

//===----------------------------------------------------------------------===//

let isBarrier    = 1 in
def VOpBundle : InstPassive<(outs variable_ops), (ins i32imm:$slot, variable_ops),
							 "bundle;", []>;

// Opcode for micro operation.
def VOpWriteReg : InstPassive<(outs), (ins), "write register;", []>;
