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
def SDT_ReadSymbol : SDTypeProfile<1, 2, []>;
def VTMReadSymbol  :  SDNode<"VTMISD::ReadSymbol", SDT_ReadSymbol, []>;

def SDT_RetVal : SDTypeProfile<0, 2, [SDTCisVT<1, i8>]>;
def VTMRetVal  :  SDNode<"VTMISD::RetVal", SDT_RetVal, [SDNPHasChain]>;

def SDT_FnRet : SDTypeProfile<0, 0, []>;
def VTMFnRet  :  SDNode<"VTMISD::Ret", SDT_FnRet,
              [SDNPVariadic, SDNPHasChain]>;

// 'Calling' internal module, which instantiated inside the current module.
// Return the result token.
def SDT_InternalCall : SDTypeProfile<1, -1, []>;
def VTMInternalCall  :  SDNode<"VTMISD::InternalCall", SDT_InternalCall,
                             [SDNPHasChain]>;

def SDT_ADDC : SDTypeProfile<2, 3, [
  SDTCisSameAs<0, 2>, SDTCisSameAs<0, 3>, SDTCisInt<0>,
  SDTCisVT<1, i1>,  SDTCisVT<4, i1>
]>;
def VTMADD    :  SDNode<"VTMISD::ADD", SDT_ADDC>;


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
  def VOpReadSymbol : CtrlInst<(outs DR:$val), (ins DR:$symbol, DR:$token),
                               "$val = arg[$symbol]", [],
                               0 /*Latency*/, 1 /*ReadAtEmit*/>;
  let isBarrier    = 1 in {
    let isReturn     = 1,
        isTerminator = 1 in
    def VOpRet : CtrlInst<(outs), (ins), "ret;", [(VTMFnRet)]>;
    
    def VOpRetVal : CtrlInst<(outs), (ins DR:$var, i8imm:$ret_no),
                             "ret $var at $ret_no;",
                             [], 0, 1>;
    let isCall = 1 in {
      def VOpInternalCall : CtrlInst<(outs DR:$token), (ins variable_ops),
                                     "call internal;", [],
                                     1 /*Latency*/, 1 /*ReadAtEmit*/>;
    }
  }
  
  let isBranch   = 1,
      isBarrier  = 1,
    isTerminator = 1 in {
    def VOpToStateb : CtrlInst<(outs), (ins State:$target),
                              "to $target;",
                              [], 0, 1>;
  let isBarrier = 0 in
    def VOpToState : CtrlInst<(outs), (ins State:$target),
                              "to $target;",
                              [], 0, 1>;

    def VOpUnreachable : CtrlInst<(outs), (ins), "unreachable;", [], 0, 1>;
  }
}

let isReMaterializable = 1,
    isAsCheapAsAMove   = 1,
    neverHasSideEffects = 1 in {
  multiclass MoveInst {
    def _ri : CtrlInst<(outs DR:$dst), (ins i64imm:$src), "$dst=$src", [], 0, 1>;
    def _rw : CtrlInst<(outs DR:$dst), (ins Wire:$src), "$dst=$src", [], 0, 1>;
    def _ra : CtrlInst<(outs DR:$dst), (ins RADD:$src), "$dst=$src", [], 0, 1>;
    def _rm : CtrlInst<(outs DR:$dst), (ins RMUL:$src), "$dst=$src", [], 0, 1>;
    def _rs : CtrlInst<(outs DR:$dst), (ins RSHT:$src), "$dst=$src", [], 0, 1>;
    def _rp : CtrlInst<(outs DR:$dst), (ins PredR:$src), "$dst=$src", [], 0, 1>;
  }

  defm VOpMove  : MoveInst;

  def VOpDefPhi : CtrlInst<(outs DR:$dst), (ins PHIR:$src),
                           "$dst <= $src", [], 0, 1>;
  def VOpMvPhi : CtrlInst<(outs PHIR:$dst), (ins DR:$src),
                           "$dst <= $src", [], 0, 1>;

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

  def VOpSel   : DPTvaInst<(outs DR:$dst), (ins DR:$cnd, DR:$src0, DR:$src1),
                            "$dst = $cnd?$src0:$src1", []>;

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

//===----------------------------------------------------------------------===//
// Computation operations.
let isReMaterializable = 1,
    neverHasSideEffects = 1 in {
  let isCommutable        = 1 in {
    def VOpAdd : FUInst<(outs RADD:$dst, DR:$co),
                  (ins DR:$src1, DR:$src2, DR:$ci),
                          "{$dst, $co} = $src1 + $src2 + $ci;",
                     [], FUAddSub>;
    def VOpMult : FUInst<(outs RMUL:$dst),
                   (ins DR:$src1, DR:$src2),
                          "$dst = $src1 * $src2;",
                     [], FUMult>;
  }

  def VOpSHL : FUInst<(outs RSHT:$dst), (ins DR:$src0, DR:$src1),
                       "$dst = shl $src0, $src1", [], FUSHIFT>;
  def VOpSRL : FUInst<(outs RSHT:$dst), (ins DR:$src0, DR:$src1),
                       "$dst = lsr $src0, $src1", [], FUSHIFT>;

  def VOpSRA : FUInst<(outs RSHT:$dst), (ins DR:$src0, DR:$src1),
                       "$dst = sra $src0, $src1", [], FUSHIFT>;
}

// Memory transaction.
let mayLoad      = 1,
    mayStore    = 1 in {
  def VOpMemTrans : FUInst<(outs DR:$dst),
                           (ins ptr_rc:$addr, DR:$src, DR:$isStore, DR:$byteenable),
                            "$dst = memtrans $src, $addr, $isStore, $byteenable",
                            [], FUMemBus, 1, 0, 0>;

  def VOpBRam : FUInst<(outs DR:$dst),
                       (ins ptr_rc:$addr, DR:$src, DR:$isStore, DR:$byteenable,
                        i8imm:$bramnum),
                       "$dst = bram $src, $addr, $isStore, $byteenable, $bramnum",
                       [], FUBRam, 1, 1, 0>;
}
//===----------------------------------------------------------------------===//

let isBarrier    = 1 in {
  // These instruction use the bitwidth operand as slot operand.
  def Control  : CtrlInst<(outs variable_ops), (ins variable_ops),
                         "control;", []>;
  def Datapath : DPTvaInst<(outs variable_ops), (ins variable_ops),
                           "datapath;", []>;
  let  isTerminator = 1 in
  def EndState : DPTvaInst<(outs),(ins), "End state;", []>;
}
