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

//===----------------------------------------------------------------------===//
// Immediates
//===----------------------------------------------------------------------===//

def simm8 : PatLeaf<(imm), [{return isInt<3>(N->getSExtValue());}]>;
def uimm8 : PatLeaf<(imm), [{return isUInt<3>(N->getZExtValue());}]>;

//===----------------------------------------------------------------------===//
// Communication operations.
let hasCtrlDep = 1 in {
let isReturn     = 1,
    isTerminator = 1,
    isBarrier    = 1 in
def VOpRet : InstPassive<(outs), (ins), "ret;", [(VTMFnRet)]>;

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

let isCommutable       = 1 in
def VOpXor	 : InstPassive<(outs dyn_rc:$dst), (ins dyn_rc:$src0, dyn_rc:$src1),
						   "$dst = xor $src0, $src1", []>;

// Cast value to difference bitwidth.
def VOpCast	 : InstPassive<(outs dyn_rc:$dst), (ins dyn_rc:$src, i1imm:$signed),
						  "$dst = cast $src, $signed", []>;

// Load immediate into registers.
def VOpLdImm : InstPassive<(outs dyn_rc:$r), (ins i64imm:$c), "$r = $c;", []>;

}

//===----------------------------------------------------------------------===//
// Computation operations.
let neverHasSideEffects = 1,
	isReMaterializable	= 1 in {
def VOpAdd : InstActive<(outs dyn_rc:$dst, DR1:$co),
			            (ins dyn_rc:$src1, dyn_rc:$src2, DR1:$ci),
					    "{$dst, $co} = $src1 + $src2 + $ci;", [], FUAddSub>;

def VOpSHL : InstActive<(outs dyn_rc:$dst), (ins dyn_rc:$src0, dyn_rc:$src1),
                        "$dst = shl $src0, $src1", [], FUSHL>;
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
