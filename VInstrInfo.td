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
def VTMInArg	:	SDNode<"VTMISD::InArg", SDT_InArg, [SDNPHasChain]>;

def SDT_RetVal : SDTypeProfile<0, 2, [SDTCisVT<1, i8>]>;
def VTMRetVal	:	SDNode<"VTMISD::RetVal", SDT_RetVal, [SDNPHasChain]>;

def SDT_FnRet : SDTypeProfile<0, 0, []>;
def VTMFnRet	:	SDNode<"VTMISD::Ret", SDT_FnRet,
							[SDNPVariadic, SDNPHasChain]>;

def SDT_ADDC : SDTypeProfile<2, 3, [
  SDTCisSameAs<0, 2>, SDTCisSameAs<0, 3>, SDTCisInt<0>,
  SDTCisVT<1, i1>,  SDTCisVT<4, i1>
]>;
def VTMADD		:	SDNode<"VTMISD::ADD", SDT_ADDC>;


def SDTMemAccess : SDTypeProfile<1, 3, [        
  SDTCisPtrTy<1>, // Address
  SDTCisVT<3, i1>, // Is load?
  // The Store value type must have the same type as the load value.
  SDTCisSameAs<0, 2>
]>;
def VTMMemAccess : SDNode<"VTMISD::MemAccess", SDTMemAccess,
                    [SDNPHasChain, SDNPMayStore, SDNPMayLoad, SDNPMemOperand]>;

// Select bit slice, Src operand, upper_bound, lower_bound
def SDT_BitSlice : SDTypeProfile<1, 3, [
  SDTCisInt<0>, SDTCisInt<1>, SDTCisOpSmallerThanOp<0, 1>,
  SDTCisVT<2, i8>, SDTCisVT<3, i8>
]>;
def VTMBitSlice : SDNode<"VTMISD::BitSlice", SDT_BitSlice>;

// Bit concat 
def SDT_BitCat : SDTypeProfile<1, -1, [
  SDTCisInt<0>
]>;
def VTMBitCat : SDNode<"VTMISD::BitCat", SDT_BitCat>;

// Bit repeat, Src operand, repeat times.
def SDT_BitRepeat : SDTypeProfile<1, 2, [
  SDTCisInt<0>, SDTCisOpSmallerThanOp<1, 0>, SDTCisVT<3, i8>  
]>;
def VTMBitRepeat : SDNode<"VTMISD::BitRepeat", SDT_BitRepeat>;

def SDT_ReductionOp : SDTypeProfile<1, 1, [SDTCisVT<0, i1>, SDTCisInt<1>]>;
def VTMReductionAnd : SDNode<"VTMISD::ReductionAnd", SDT_ReductionOp>;
def VTMReductionOr  : SDNode<"VTMISD::ReductionOr", SDT_ReductionOp>;
def VTMReductionXor : SDNode<"VTMISD::ReductionXor", SDT_ReductionOp>;

def SDT_NOT : SDTypeProfile<1, 1, [
  SDTCisSameAs<0, 1>, SDTCisInt<0>
]>;
def VTMNot : SDNode<"VTMISD::Not", SDT_NOT>;

//===----------------------------------------------------------------------===//
// Immediates
//===----------------------------------------------------------------------===//

def simm8 : PatLeaf<(imm), [{return isInt<3>(N->getSExtValue());}]>;
def uimm8 : PatLeaf<(imm), [{return isUInt<3>(N->getZExtValue());}]>;

def State : Operand<OtherVT>;

//===----------------------------------------------------------------------===//
// Communication operations.
let hasCtrlDep = 1 in {
let isReturn     = 1,
    isTerminator = 1,
    isBarrier    = 1 in
def VOpRet : InstPassive<(outs), (ins), "ret;", [(VTMFnRet)]>;

let isBranch = 1,
	isTerminator=1 in
def VOpToState : InstPassive<(outs), (ins DR1:$cnd, State:$target),
							 "if ($cnd) state $target;",
							[(brcond DR1:$cnd, bb:$target)]>;

// Prevent MachineCSE delete any VOpArg.
// let neverHasSideEffects = 1 in
def VOpArg : InstPassive<(outs dyn_rc:$arg), (ins i8imm:$arg_no),
		 				"$arg = arg[$arg_no]", [], 0, 1>;

let isBarrier		= 1  in
def VOpRetVal : InstPassive<(outs), (ins dyn_rc:$var, i8imm:$ret_no),
							 "ret $var at $ret_no;",
							 [], 0, 1>;
}

let isReMaterializable = 1,
    isAsCheapAsAMove   = 1,
    neverHasSideEffects = 1 in {
// Bitwise logic operation.
let isCommutable       = 1 in
def VOpXor	 : InstPassive<(outs dyn_rc:$dst), (ins dyn_rc:$src0, dyn_rc:$src1),
						   "$dst = xor $src0, $src1", []>;
def VOpAnd	 : InstPassive<(outs dyn_rc:$dst), (ins dyn_rc:$src0, dyn_rc:$src1),
						   "$dst = and $src0, $src1", []>;
def VOpNot	 : InstPassive<(outs dyn_rc:$dst), (ins dyn_rc:$src), 
						   "$dst = ~ $src", []>;


// Reduction operation.
def VOpRAnd	 : InstPassive<(outs DR1:$dst), (ins dyn_rc:$src), 
						   "$dst = rand $src", []>;
def VOpROr	 : InstPassive<(outs DR1:$dst), (ins dyn_rc:$src), 
						   "$dst = ror $src", []>;
def VOpRXor	 : InstPassive<(outs DR1:$dst), (ins dyn_rc:$src), 
						   "$dst = rxor $src", []>;

// Bit level operations.
def VOpBitSlice	 : InstPassive<(outs dyn_rc:$dst),
                              (ins dyn_rc:$src, DR8:$ub, DR8:$lb),
							  "$dst = bitslice $src, $ub, $lb", []>;
def VOpBitRepeat : InstPassive<(outs dyn_rc:$dst),
                              (ins dyn_rc:$src, DR8:$times),
							  "$dst = bitrepeat $src, $times", []>;
def VOpBitCat	 : InstPassive<(outs dyn_rc:$dst), (ins variable_ops),
							  "$dst = bitcat", []>;
}

//===----------------------------------------------------------------------===//
// Computation operations.
let neverHasSideEffects = 1,
	isReMaterializable	= 1 in {
let isCommutable       = 1 in
def VOpAdd : InstActive<(outs dyn_rc:$dst, DR1:$co),
			            (ins dyn_rc:$src1, dyn_rc:$src2, DR1:$ci),
					    "{$dst, $co} = $src1 + $src2 + $ci;", [], FUAddSub>;

def VOpSHL : InstActive<(outs dyn_rc:$dst), (ins dyn_rc:$src0, dyn_rc:$src1),
                        "$dst = shl $src0, $src1", [], FUSHL>;

def VOpSRA : InstActive<(outs dyn_rc:$dst), (ins dyn_rc:$src0, dyn_rc:$src1),
                        "$dst = sra $src0, $src1", [], FUSHL>;
}

let mayLoad		  = 1,
	mayStore	  = 1 in
def VOpMemAccess : InstActive<(outs dyn_rc:$dst),
							  (ins dyn_rc:$src, ptr_rc:$addr, DR1:$isLoad),
							  "$dst = memaccess $src, $addr, $isLoad",
							  [], FUMemBus, 1, 1>;


//===----------------------------------------------------------------------===//

let isBarrier    = 1 in
def VOpBundle : InstPassive<(outs variable_ops), (ins i32imm:$slot, variable_ops),
							 "bundle;", []>;

// Opcode for micro operation.
def VOpWriteReg : InstPassive<(outs), (ins), "write register;", []>;
