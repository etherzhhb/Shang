//===- BitLevelOpt.cpp - Implement bit level optimizations on SelectionDAG -===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This pass try to lower some arithmetic and logic operations to much more
// cheaper bitwise operation which can be express by the combination of bit
// slice selection, bit concation and bit repeation.
// for example, logic operation shift left:
//   a[31:0] = b[31:0] << 2 => a[31:0] = {b[29:0], 2'b00 }
//
// After lowering to bit level operation, more optimization opportunity may be
// exposed.
//
//===----------------------------------------------------------------------===//

#include "vtm/VISelLowering.h"

#include "llvm/Function.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/PseudoSourceValue.h"
#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"
#include "llvm/ADT/VectorExtras.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

template<typename Func>
inline static SDValue commuteAndTryAgain(SDNode *N, const VTargetLowering &TLI,
                                         TargetLowering::DAGCombinerInfo &DCI,
                                         bool ExchangeOperand, Func F) {

  if (ExchangeOperand) return SDValue();

  // If we not try to exchange the operands, exchange and try again.
  return F(N, TLI, DCI, !ExchangeOperand);
}

// Lower shifting a constant amount:
//   a[31:0] = b[31:0] << 2 => a[31:0] = {b[29:0], 2'b00 }
static SDValue PerformShiftImmCombine(SDNode *N, const VTargetLowering &TLI,
                                      TargetLowering::DAGCombinerInfo &DCI) {
  SelectionDAG &DAG = DCI.DAG;
  ConstantSDNode *ShAmt = dyn_cast<ConstantSDNode>(N->getOperand(1));
  // Can only handle shifting a constant amount.
  if (ShAmt == 0) return SDValue();

  DebugLoc dl = N->getDebugLoc();
  EVT VT = N->getValueType(0);
  unsigned PaddingSize = ShAmt->getSExtValue();
  EVT PaddingVT = EVT::getIntegerVT(*DAG.getContext(), PaddingSize);
  // Create this padding bits as target constant.
  SDValue PaddingBits = DAG.getConstant(0, PaddingVT, true);

  SDValue Src = N->getOperand(0);
  unsigned SrcSize = TLI.computeSizeInBits(Src);
  

  switch (N->getOpcode()) {
  case ISD::SHL: {
    // Discard the higher bits of src.
    Src = TLI.getBitSlice(DAG, dl, Src, SrcSize - PaddingSize, 0);
    DCI.AddToWorklist(Src.getNode());
    return DAG.getNode(VTMISD::BitCat, dl, VT, Src, PaddingBits);
  }
  case ISD::SRA:
    PaddingBits = TLI.getBitRepeat(DAG, dl, TLI.getSignBit(DAG, dl, Src),
                                   PaddingSize);
    // Fall though
  case ISD::SRL:
    // Discard the lower bits of src.
    Src = TLI.getBitSlice(DAG, dl, Src, SrcSize, PaddingSize);
    DCI.AddToWorklist(Src.getNode());
    return DAG.getNode(VTMISD::BitCat, dl, VT, PaddingBits, Src);
  default:
    assert(0 && "Bad opcode!");
    return SDValue();
  }
}

static SDValue PerformAddCombine(SDNode *N, const VTargetLowering &TLI,
                                 TargetLowering::DAGCombinerInfo &DCI,
                                 bool ExchangeOperand = false) {
  ConstantSDNode *C = dyn_cast<ConstantSDNode>(N->getOperand(2));
  // Can only combinable if carry is known.
  if (C == 0)  return SDValue();

  SDValue OpA = N->getOperand(0 ^ ExchangeOperand),
          OpB = N->getOperand(1 ^ ExchangeOperand);
  
  SelectionDAG &DAG = DCI.DAG;

  if (ConstantSDNode *COpB =  dyn_cast<ConstantSDNode>(OpB)) {
    // A + ~0 + 1 => A - 0 => {1, A}
    if (COpB->isAllOnesValue() && C->isAllOnesValue()) {
      DCI.CombineTo(N, OpA, DAG.getTargetConstant(1, MVT::i1));
      return SDValue(N, 0);
    }
    // A + 0 + 0 => {0, A}
    if (COpB->isNullValue() && C->isNullValue()) {
      DCI.CombineTo(N, OpA, DAG.getTargetConstant(0, MVT::i1));
      return SDValue(N, 0);
    }
  }

  // TODO: Combine with bit mask information.
  return commuteAndTryAgain(N, TLI, DCI, ExchangeOperand, PerformAddCombine);
}

static SDValue PerformXorCombine(SDNode *N, const VTargetLowering &TLI,
                                 TargetLowering::DAGCombinerInfo &DCI,
                                 bool ExchangeOperand = false) {
  SDValue OpA = N->getOperand(0 ^ ExchangeOperand),
          OpB = N->getOperand(1 ^ ExchangeOperand);
  SelectionDAG &DAG = DCI.DAG;
  
  if (ConstantSDNode *COpB =  dyn_cast<ConstantSDNode>(OpB))
    if (COpB->isAllOnesValue()) return TLI.getNot(DAG, N->getDebugLoc(), OpA);

  return commuteAndTryAgain(N, TLI, DCI, ExchangeOperand, PerformXorCombine);
}

static bool ExtractBitMaskInfo(int64_t Val, unsigned SizeInBits,
                               unsigned &UB, unsigned &LB) {
  if (isShiftedMask_64(Val) || isMask_64(Val)) {
    LB = CountTrailingZeros_64(Val);
    UB = std::min(SizeInBits, 64 - CountLeadingZeros_64(Val));
    return true;
  }

  return false;
}

static SDValue ExtractEnabledBitsAnd(TargetLowering::DAGCombinerInfo &DCI,
                                     SDValue Op, unsigned UB, unsigned LB) {
  // A & 1 = A
  SDValue V = VTargetLowering::getBitSlice(DCI.DAG, Op->getDebugLoc(),
                                           Op, UB, LB);
  DCI.AddToWorklist(V.getNode());
  return V;
}

static SDValue ExtractDisabledBitsAnd(TargetLowering::DAGCombinerInfo &DCI,
                                      SDValue Op, unsigned UB, unsigned LB) {
  EVT HiVT = EVT::getIntegerVT(*DCI.DAG.getContext(), UB - LB);
  // A & 0 = 0
  return DCI.DAG.getTargetConstant(0, HiVT);
}


static SDValue ExtractEnabledBitsOr(TargetLowering::DAGCombinerInfo &DCI,
                                    SDValue Op,  unsigned UB, unsigned LB) {
  EVT VT = EVT::getIntegerVT(*DCI.DAG.getContext(), UB - LB);
  // A | 1 = 1
  return DCI.DAG.getTargetConstant(~uint64_t(0), VT);
}

static SDValue ExtractDisabledBitsOr(TargetLowering::DAGCombinerInfo &DCI,
                                     SDValue Op, unsigned UB, unsigned LB) {
  // A | 0 = A
  SDValue V = VTargetLowering::getBitSlice(DCI.DAG, Op->getDebugLoc(),
                                           Op, UB, LB);
  DCI.AddToWorklist(V.getNode());
  return V;
}

static SDValue ExtractEnabledBitsXOr(TargetLowering::DAGCombinerInfo &DCI,
                                     SDValue Op, unsigned UB, unsigned LB) {
  // A ^ 1 = ~A
  SDValue V = VTargetLowering::getBitSlice(DCI.DAG, Op->getDebugLoc(),
                                           Op, UB, LB);
  DCI.AddToWorklist(V.getNode());
  V = VTargetLowering::getNot(DCI.DAG, Op->getDebugLoc(), V);
  DCI.AddToWorklist(V.getNode());
  return V;
}

static SDValue ExtractDisabledBitsXOr(TargetLowering::DAGCombinerInfo &DCI,
                                      SDValue Op, unsigned UB, unsigned LB) {
  // A ^ 0 = A
  SDValue V = VTargetLowering::getBitSlice(DCI.DAG, Op->getDebugLoc(),
                                           Op, UB, LB);
  DCI.AddToWorklist(V.getNode());
  return V;
}


template<typename ExtractBitsFunc>
static SDValue ExtractBits(SDValue Op, int64_t Mask, const VTargetLowering &TLI,
                           TargetLowering::DAGCombinerInfo &DCI,
                           ExtractBitsFunc ExtractEnabledBits,
                           ExtractBitsFunc ExtractDisabledBits,
                           bool flipped = false) {

  DebugLoc dl = Op->getDebugLoc();
  SelectionDAG &DAG = DCI.DAG;
  EVT VT = Op.getValueType();
  unsigned SizeInBits = Op.getValueSizeInBits();

  unsigned UB, LB;
  if (ExtractBitMaskInfo(Mask, SizeInBits, UB, LB)) {
    // All disabled?
    if (UB - LB == SizeInBits)
      return ExtractEnabledBits(DCI, Op, SizeInBits, 0);

    // Handle and with a bit mask like A & 0xf0, simply extract the
    // corresponding bits.
    // Handle or with a bit mask like A | 0xf0, simply set the corresponding
    // bits to 1.

    SDValue MidBits = ExtractEnabledBits(DCI, Op, UB, LB);
    DCI.AddToWorklist(MidBits.getNode());

    SDValue HiBits;
    if (SizeInBits != UB) {
      HiBits = ExtractDisabledBits(DCI, Op, SizeInBits, UB);
      DCI.AddToWorklist(HiBits.getNode());
    }

    if(LB == 0) {
      assert(SizeInBits != UB && "Unexpected all one value!");
      SDValue Result = DAG.getNode(VTMISD::BitCat, dl, VT, HiBits, MidBits);
      assert(TLI.computeSizeInBits(Result) == SizeInBits
        && "Bit widht not match!");
      return Result;
    }

    // Build the lower part.
    SDValue LoBits = ExtractDisabledBits(DCI, Op, LB, 0);
    DCI.AddToWorklist(LoBits.getNode());

    SDValue Lo = DAG.getNode(VTMISD::BitCat, dl, VT, MidBits, LoBits);
    if (UB == SizeInBits) {
      assert(TLI.computeSizeInBits(Lo) == SizeInBits
             && "Bit widht not match!");
      return Lo;
    }

    DCI.AddToWorklist(Lo.getNode());
    SDValue Result = DAG.getNode(VTMISD::BitCat, Op->getDebugLoc(), VT, HiBits, Lo);
    assert(TLI.computeSizeInBits(Result) == SizeInBits
           && "Bit widht not match!");
    return Result;
  }

  if (flipped) return SDValue();

  // Flip the mask and try again.
  return ExtractBits(Op, ~Mask, TLI, DCI,
                     ExtractDisabledBits, ExtractEnabledBits, true);
}

static SDValue PerformLogicCombine(SDNode *N, const VTargetLowering &TLI,
                                   TargetLowering::DAGCombinerInfo &DCI,
                                   bool ExchangeOperand = false) {
  SDValue OpA = N->getOperand(0 ^ ExchangeOperand),
          OpB = N->getOperand(1 ^ ExchangeOperand);

  if (ConstantSDNode *COpB =  dyn_cast<ConstantSDNode>(OpB)) {
    int64_t Val = COpB->getSExtValue();

    switch(N->getOpcode()) {
    case ISD::AND:
      return ExtractBits(OpA, Val, TLI, DCI,
                         ExtractEnabledBitsAnd,
                         ExtractDisabledBitsAnd);
    case ISD::OR:
      return ExtractBits(OpA, Val, TLI, DCI,
                         ExtractEnabledBitsOr,
                         ExtractDisabledBitsOr);
    case ISD::XOR:
      return ExtractBits(OpA, Val, TLI, DCI,
                         ExtractEnabledBitsXOr,
                         ExtractDisabledBitsXOr);
    default:
      llvm_unreachable("Unexpected Logic Node!");
    }
  }

  return commuteAndTryAgain(N, TLI, DCI, ExchangeOperand, PerformLogicCombine);
}

static SDValue PerformNotCombine(SDNode *N, const VTargetLowering &TLI,
                                 TargetLowering::DAGCombinerInfo &DCI) {
  SDValue Op = N->getOperand(0);

  // ~(~A) = A.
  if (Op->getOpcode() == VTMISD::Not) return Op->getOperand(0);

  return SDValue();
}

static SDValue PerformBitCatCombine(SDNode *N, const VTargetLowering &TLI,
                                    TargetLowering::DAGCombinerInfo &DCI) {

  SDValue OpA = N->getOperand(0),
          OpB = N->getOperand(1);

  // Dose the node looks like {a[UB-1, M], a[M-1, LB]}? If so, combine it to
  // a[UB-1, LB]
  if (OpA->getOpcode() == VTMISD::BitSlice
      && OpB->getOpcode() == VTMISD::BitSlice) {
    SDValue SrcOp = OpA->getOperand(0);

    if (SrcOp.getNode() == OpB->getOperand(0).getNode()
        && OpA->getConstantOperandVal(2) == OpB->getConstantOperandVal(1))
      return VTargetLowering::getBitSlice(DCI.DAG, SrcOp->getDebugLoc(),
                                          SrcOp,
                                          OpA->getConstantOperandVal(1),
                                          OpB->getConstantOperandVal(2),
                                          N->getValueSizeInBits(0));
  }

  return SDValue();
}

static SDValue PerformBitSliceCombine(SDNode *N, const VTargetLowering &TLI,
                                      TargetLowering::DAGCombinerInfo &DCI) {
  SDValue Op = N->getOperand(0);
  unsigned UB = N->getConstantOperandVal(1),
           LB = N->getConstantOperandVal(2);
  DebugLoc dl = N->getDebugLoc();

  // Try to flatten the bitslice tree.
  if (Op->getOpcode() == VTMISD::BitSlice) {
    SDValue SrcOp = Op->getOperand(0);
    unsigned Offset = Op->getConstantOperandVal(2);
    return VTargetLowering::getBitSlice(DCI.DAG, dl,
                                        SrcOp, UB + Offset, LB + Offset,
                                        N->getValueSizeInBits(0));
  }

  // If the big range fall into the bit range of one of the BitCat operand,
  // return bitslice of that operand.
  if (Op->getOpcode() == VTMISD::BitCat) {
    SDValue HiOp = Op->getOperand(0), LoOp = Op->getOperand(1);
    unsigned SplitBit = TLI.computeSizeInBits(LoOp);
    if (UB <= SplitBit)
      return VTargetLowering::getBitSlice(DCI.DAG, dl, LoOp, UB, LB,
                                          N->getValueSizeInBits(0));

    if (LB >= SplitBit)
      return VTargetLowering::getBitSlice(DCI.DAG, dl,
                                          HiOp, UB - SplitBit, LB - SplitBit,
                                          N->getValueSizeInBits(0));

    HiOp = VTargetLowering::getBitSlice(DCI.DAG, dl, HiOp, UB - SplitBit, 0);
    DCI.AddToWorklist(HiOp.getNode());
    LoOp = VTargetLowering::getBitSlice(DCI.DAG, dl, LoOp, SplitBit, LB);
    DCI.AddToWorklist(LoOp.getNode());
    return DCI.DAG.getNode(VTMISD::BitCat, dl, N->getVTList(), HiOp, LoOp);
  }

  return SDValue();
}

static SDValue PerformReduceCombine(SDNode *N, const VTargetLowering &TLI,
                                    TargetLowering::DAGCombinerInfo &DCI) {
  SDValue Op = N->getOperand(0);

  // 1 bit value do not need to reduce at all.
  if (TLI.computeSizeInBits(Op) == 1) return Op;

  return SDValue();
}

SDValue VTargetLowering::PerformDAGCombine(SDNode *N,
                                           TargetLowering::DAGCombinerInfo &DCI)
                                           const {
  switch (N->getOpcode()) {
  case VTMISD::BitCat:
    return PerformBitCatCombine(N, *this, DCI);
  case VTMISD::BitSlice:
    return PerformBitSliceCombine(N, *this, DCI);
  case VTMISD::ADD:
    return PerformAddCombine(N, *this, DCI);
  case ISD::SHL:
  case ISD::SRA:
  case ISD::SRL:
    return PerformShiftImmCombine(N, *this, DCI);
  case ISD::AND:
  case ISD::OR:
  case ISD::XOR:
    return PerformLogicCombine(N, *this, DCI);
  case VTMISD::Not:
    return PerformNotCombine(N, *this, DCI);
  case VTMISD::RAnd:
  case VTMISD::ROr:
  case VTMISD::RXor:
    return PerformReduceCombine(N, *this, DCI);
  }

  return SDValue();
}
