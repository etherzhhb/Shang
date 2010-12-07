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
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

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
    return DAG.getNode(VTMISD::BitCat, dl, VT, Src, PaddingBits);
  }
  case ISD::SRA:
    PaddingBits = TLI.getBitRepeat(DAG, dl, TLI.getSignBit(DAG, dl, Src),
                                   PaddingSize);
    // Fall though
  case ISD::SRL:
    // Discard the lower bits of src.
    Src = TLI.getBitSlice(DAG, dl, Src, SrcSize, PaddingSize);
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

  // If we not try to exchange the operands, exchange and try again.
  if (!ExchangeOperand)
    return PerformAddCombine(N, TLI, DCI, !ExchangeOperand);

  return SDValue();
}

static  SDValue PerformXorCombine(SDNode *N, const VTargetLowering &TLI,
                                  TargetLowering::DAGCombinerInfo &DCI,
                                  bool ExchangeOperand = false) {
  SDValue OpA = N->getOperand(0 ^ ExchangeOperand),
          OpB = N->getOperand(1 ^ ExchangeOperand);
  SelectionDAG &DAG = DCI.DAG;
  
  if (ConstantSDNode *COpB =  dyn_cast<ConstantSDNode>(OpB))
    if (COpB->isAllOnesValue()) return TLI.getNot(DAG, N->getDebugLoc(), OpA);

  // If we not try to exchange the operands, exchange and try again.
  if (!ExchangeOperand)
    return PerformXorCombine(N, TLI, DCI, !ExchangeOperand);

  return SDValue();
}

static  SDValue PerformNotCombine(SDNode *N, const VTargetLowering &TLI,
                                  TargetLowering::DAGCombinerInfo &DCI) {
  SDValue Op = N->getOperand(0);
  SelectionDAG &DAG = DCI.DAG;

  // ~(~A) = A.
  if (Op->getOpcode() == VTMISD::Not) return Op->getOperand(0);

  return SDValue();
}

SDValue VTargetLowering::PerformDAGCombine(SDNode *N,
                                           TargetLowering::DAGCombinerInfo &DCI)
                                           const {
  switch (N->getOpcode()) {
  case ISD::SHL:
  case ISD::SRA:
  case ISD::SRL:
    return PerformShiftImmCombine(N, *this, DCI);
  case ISD::XOR:
    return PerformXorCombine(N, *this, DCI);
  case VTMISD::ADD:
    return PerformAddCombine(N, *this, DCI);
  case VTMISD::Not:
    return PerformNotCombine(N, *this, DCI);

  }

  return SDValue();
}
