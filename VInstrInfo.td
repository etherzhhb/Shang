//===---------- VInstrInfo.td - VTM Instruction defs -----------*- tblgen-*-===//
//
//                      The Shang HLS frameowrk                               //
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
// take port name, Function address, and dependence token as parameter
def SDT_ReadReturn : SDTypeProfile<1, 2, []>;
def VTMReadReturn  :  SDNode<"VTMISD::ReadReturn", SDT_ReadReturn, []>;

def SDT_LoadArgument : SDTypeProfile<1, 1, [SDTCisInt<1>]>;
def VTMLoadLoadArgument	:	SDNode<"VTMISD::LoadArg", SDT_LoadArgument,
								   [SDNPHasChain]>;

def SDT_RetVal : SDTypeProfile<0, 2, [SDTCisVT<1, i8>]>;
def VTMRetVal  :  SDNode<"VTMISD::RetVal", SDT_RetVal, [SDNPHasChain]>;

def SDT_FnRet : SDTypeProfile<0, 0, []>;
def VTMFnRet  :  SDNode<"VTMISD::Ret", SDT_FnRet,
              [SDNPVariadic, SDNPHasChain]>;

def VTMICmp : SDNode<"VTMISD::ICmp", SDTSetCC>;

def SDT_VBinIntOp : SDTypeProfile<1, 2, [     // MULHiLo.
  SDTCisSameAs<0, 1>, SDTCisSameAs<0, 2>, SDTCisInt<0>
]>;

def VTMUMUL_HiLo : SDNode<"VTMISD::UMUL_HiLo", SDT_VBinIntOp>;

def SDT_VTernaryIntOp : SDTypeProfile<1, 3, [     // MULHiLo.
  SDTCisSameAs<0, 1>, SDTCisSameAs<0, 2>, SDTCisInt<0>, SDTCisInt<3>
]>;
def VTMADDCS     : SDNode<"VTMISD::ADDCS", SDT_VTernaryIntOp>;

// 'Calling' internal module, which instantiated inside the current module.
// Return the result token.
def SDT_InternalCall : SDTypeProfile<1, -1, []>;
def VTMInternalCall  :  SDNode<"VTMISD::InternalCall", SDT_InternalCall,
                             [SDNPHasChain]>;

def SDT_MemTrans : SDTypeProfile<1, 4, [
  SDTCisPtrTy<1>, // Address
  // The Store value type must have the same type as the load value.
  SDTCisSameAs<0, 2>,
  SDTCisVT<3, i1>, // Is load?
  // Size Info, the rtl backend should interpret this according the bus
  // standard.
  SDTCisVT<4, i8>
]>;
def VTMMemTrans : SDNode<"VTMISD::MemTrans", SDT_MemTrans,
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
  def VOpReadReturn : CtrlInst<(outs DR:$val),
                                (ins DR:$token, i8imm:$ret_no),
                                "$val = $token[$ret_no]", [],
                                0 /*Latency*/, 1 /*writeUntilFinish*/>;
  let isBarrier    = 1 in {
    let isReturn     = 1,
        isTerminator = 1 in
    def VOpRet : CtrlInst<(outs), (ins), "ret;", [(VTMFnRet)],
                          0 /*Latency*/, 0 /*writeUntilFinish*/>;

    def VOpRetVal : CtrlInst<(outs), (ins DR:$var, i8imm:$ret_no),
                             "ret $var at $ret_no;",
                             [], 0 /*Latency*/, 0 /*writeUntilFinish*/>;

  }

  let isCall = 1 in {
    def VOpInternalCall : FUInst<(outs DR:$token),
                                 (ins i8imm:$CalleeNum, variable_ops),
                                 "call internal;", [],
                                 FUCalleeFN,
                                 0 /*writeUntilFinish*/>;
  }

  let isBranch   = 1,
      isBarrier  = 1,
    isTerminator = 1 in {
    def VOpToStateb : CtrlInst<(outs), (ins DR:$cnd, State:$target),
                              "to $target;",
                              [], 0 /*Latency*/, 0 /*writeUntilFinish*/>;
  let isBarrier = 0 in
    def VOpToState : CtrlInst<(outs), (ins DR:$cnd, State:$target),
                              "to $target;",
                              [], 0 /*Latency*/, 0 /*writeUntilFinish*/>;

    def VOpUnreachable : CtrlInst<(outs), (ins), "unreachable;", [],
                                  0 /*Latency*/, 0 /*writeUntilFinish*/>;
  }
}

// Terminator instuctions inside bundle, the isTerminator flag is 0.
def VOpToState_nt : CtrlInst<(outs), (ins DR:$cnd, State:$target),
                             "to $target;",
                             [], 0 /*Latency*/, 0 /*writeUntilFinish*/>;
def VOpRet_nt : CtrlInst<(outs), (ins), "ret;", [],
						 0 /*Latency*/, 0 /*writeUntilFinish*/>;

let isReMaterializable = 1,
    isAsCheapAsAMove   = 1,
    neverHasSideEffects = 1 in {
  def VOpMove  : CtrlInst<(outs DR:$dst), (ins DR:$src), "$dst=$src", [],
                           0 /*Latency*/, 1 /*writeUntilFinish*/>;

  def VOpMoveArg  : CtrlInst<(outs DR:$dst), (ins DR:$src), "$dst=$src", [],
							 0 /*Latency*/, 1 /*writeUntilFinish*/>;

  def VOpDefPhi : CtrlInst<(outs DR:$dst), (ins DR:$src),
                           "$dst <= $src", [],
                           0 /*Latency*/, 1 /*writeUntilFinish*/>;
  // Bitwise logic operation.
  let isCommutable       = 1 in {
    def VOpXor   : DPTvaInst<(outs DR:$dst), (ins DR:$src0, DR:$src1),
                              "$dst = xor $src0, $src1", []>;
    def VOpAnd   : DPTvaInst<(outs DR:$dst), (ins DR:$src0, DR:$src1),
                              "$dst = and $src0, $src1", []>;
    def VOpOr     : DPTvaInst<(outs DR:$dst), (ins DR:$src0, DR:$src1),
                              "$dst = or $src0, $src1", []>;
  }

  def VOpNot   : DPTvaInst<(outs DR:$dst), (ins DR:$src),
                            "$dst = ~ $src", []>;

  def VOpLUT : DPTvaInst<(outs DR:$dst), (ins DR:$table, variable_ops),
                          "$dst = lut$size[$table]", []>;

  def VOpSel   : DPTvaInst<(outs DR:$dst), (ins DR:$cnd, DR:$src0, DR:$src1),
                           "$dst = $cnd?$src0:$src1", []>;

  def VOpReadFU: CtrlInst<(outs DR:$dst), (ins DR:$src, i8imm:$fuid),
                          "$dst = read fu $src $fuid", [],
                          0 /*Latency*/, 1 /*writeUntilFinish*/>;

  // Reduction operation.
  def VOpRAnd   : DPTvaInst<(outs DR:$dst), (ins DR:$src),
                            "$dst = rand $src", []>;
  def VOpROr   : DPTvaInst<(outs DR:$dst), (ins DR:$src),
                            "$dst = ror $src", []>;
  def VOpRXor   : DPTvaInst<(outs DR:$dst), (ins DR:$src),
                            "$dst = rxor $src", []>;

  // Bit level operations.
  def VOpBitSlice   : DPTvaInst<(outs DR:$dst),
                                (ins DR:$src, i8imm:$ub, i8imm:$lb),
                                "$dst = bitslice $src, $ub, $lb", []>;
  def VOpBitRepeat : DPTvaInst<(outs DR:$dst),
                                (ins DR:$src, i8imm:$times),
                                "$dst = bitrepeat $src, $times", []>;
  def VOpBitCat     : DPTvaInst<(outs DR:$dst), (ins DR:$lhs, DR:$rhs),
                                "$dst = bitcat $lhs, $rhs", []>;
}

// Distributed (in time domain) multiplexer, have side effects,
// and should not be CSEed.
// Already set by default: let hasSideEffects = 1 in
def VOpDstMux : FUInst<(outs DR:$dst),
                        (ins DR:$src, i8imm:$muxnum, i8imm:$size),
                        "$dst($muxnum, $size) = $src",
                        [], FUMUX, 1 /*writeUntilFinish*/>;

// Special instruction for copy source register to phi incomming register,
// we may need to forward the source register.
// The PHI related instruction should not be CSEed, too.
// Already set by default: let hasSideEffects = 1 in
def VOpMvPhi : CtrlInst<(outs DR:$dst), (ins DR:$src, State:$target),
                           "$dst <= $src", [],
                           0 /*Latency*/, 1 /*writeUntilFinish*/>;
// Move the pipeline, this is unconditional.
def VOpMvPipe : CtrlInst<(outs DR:$dst), (ins DR:$src, State:$target),
                         "$dst <= $src", [],
                         0 /*Latency*/, 1 /*writeUntilFinish*/>;

def VOpDisableFU: CtrlInst<(outs), (ins DR:$src, i8imm:$fuid),
                           "disable fu $src $fuid", [],
                           0 /*Latency*/, 0 /*writeUntilFinish*/>;
//===----------------------------------------------------------------------===//
// Computation operations.
let isReMaterializable = 1,
    neverHasSideEffects = 1 in {
  let isCommutable        = 1 in {
    def VOpAdd : FUInst<(outs DR:$dst),(ins DR:$src1, DR:$src2, DR:$ci),
                         "$dst = $src1 + $src2 + $ci;",
                         [], FUAddSub, 0 /*writeUntilFinish*/>;

	// Data-path only version.
    def VOpAdd_c : DPTvaInst<(outs DR:$dst),(ins DR:$src1, DR:$src2, DR:$ci),
                             "$dst = $src1 + $src2 + $ci;", []>;

    def VOpMult : FUInst<(outs DR:$dst), (ins DR:$src1, DR:$src2),
                         "$dst = $src1 * $src2;",
                         [], FUMult, 0 /*writeUntilFinish*/>;
    def VOpMultLoHi : FUInst<(outs DR:$dst), (ins DR:$src1, DR:$src2),
                             "$dst = $src1 * $src2;",
                             [], FUMult, 0 /*writeUntilFinish*/>;

    def VOpMult_c : DPTvaInst<(outs DR:$dst), (ins DR:$src1, DR:$src2),
                         "$dst = $src1 * $src2;",
                         []>;
    def VOpMultLoHi_c : DPTvaInst<(outs DR:$dst), (ins DR:$src1, DR:$src2),
                             "$dst = $src1 * $src2;",
                             []>;
  }

  def VOpSHL : FUInst<(outs DR:$dst), (ins DR:$src0, DR:$src1),
                       "$dst = shl $src0, $src1", [], FUSHIFT,
                       0 /*writeUntilFinish*/>;
  def VOpSRL : FUInst<(outs DR:$dst), (ins DR:$src0, DR:$src1),
                       "$dst = lsr $src0, $src1", [], FUSHIFT,
                       0 /*writeUntilFinish*/>;

  def VOpSRA : FUInst<(outs DR:$dst), (ins DR:$src0, DR:$src1),
                       "$dst = sra $src0, $src1", [], FUSHIFT,
                       0 /*writeUntilFinish*/>;

  def VOpSHL_c : DPTvaInst<(outs DR:$dst), (ins DR:$src0, DR:$src1),
                           "$dst = shl $src0, $src1", []>;
  def VOpSRL_c : DPTvaInst<(outs DR:$dst), (ins DR:$src0, DR:$src1),
                           "$dst = lsr $src0, $src1", []>;

  def VOpSRA_c : DPTvaInst<(outs DR:$dst), (ins DR:$src0, DR:$src1),
                           "$dst = sra $src0, $src1", []>;

  // Integer comparison
  let isCompare = 1 in
    def VOpICmp   : FUInst<(outs DR:$dst), (ins DR:$src0, DR:$src1, i8imm:$CC),
                           "$dst = icmp $src0, $src1 $CC", [], FUICmp,
	                       0 /*writeUntilFinish*/>;
	// Data-path only version.
    def VOpICmp_c  : DPTvaInst<(outs DR:$dst), (ins DR:$src0, DR:$src1, i8imm:$CC),
                                "$dst = icmp $src0, $src1 $CC", []>;
}

// Memory transaction.
let mayLoad      = 1,
    mayStore    = 1 in {
  def VOpMemTrans : FUInst<(outs DR:$dst),
                           (ins ptr_rc:$addr, DR:$src, DR:$isStore, DR:$byteenable),
                            "$dst = memtrans $src, $addr, $isStore, $byteenable",
                            [], FUMemBus, 0 /*writeUntilFinish*/>;
  def VOpBRAMTrans : FUInst<(outs DR:$dst),
                       (ins ptr_rc:$addr, DR:$src, DR:$isStore, DR:$byteenable,
                        i8imm:$bramnum),
                       "$dst = bram $src, $addr, $isStore, $byteenable, $bramnum",
                       [], FUBRam, 0 /*writeUntilFinish*/>;
}

def ImpUse : InstVTM<(outs), (ins variable_ops), "imp-use", [], FUTrivial>;

//===----------------------------------------------------------------------===//

let isBarrier    = 1 in {
  // These instruction use the bitwidth operand as slot operand.
  def CtrlStart  : CtrlInst<(outs), (ins i8imm:$slot),
                            "ctrl start;", []>;
  def CtrlEnd  : CtrlInst<(outs), (ins i8imm:$slot, variable_ops),
                            "control;", []>;

  def Datapath  : CtrlInst<(outs), (ins i8imm:$slot),
                           "control;", []>;
  let  isTerminator = 1 in
    def EndState : DPTvaInst<(outs),(ins), "End state;", []>;
}
