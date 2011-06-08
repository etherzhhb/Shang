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
                                         bool Commuted, Func F) {

  if (Commuted) return SDValue();

  // If we not try to exchange the operands, exchange and try again.
  return F(N, TLI, DCI, !Commuted);
}

inline static bool isAllOnesValue(uint64_t Val, unsigned SizeInBits) {
  uint64_t AllOnes = VTargetLowering::getBitSlice(~uint64_t(0), SizeInBits);
  return VTargetLowering::getBitSlice(Val, SizeInBits) == AllOnes;
}

inline static bool isNullValue(uint64_t Val, unsigned SizeInBits) {
  return VTargetLowering::getBitSlice(Val, SizeInBits) == 0;
}

inline static unsigned ExtractConstant(SDValue V, uint64_t &Val) {
  if (ConstantSDNode *CSD =dyn_cast<ConstantSDNode>(V)) {
    unsigned SizeInBits = V.getValueSizeInBits();
    Val = VTargetLowering::getBitSlice(CSD->getZExtValue(), SizeInBits);
    return SizeInBits;
  }

  if (V.getOpcode() == VTMISD::BitSlice)
    if (ConstantSDNode *CSD =dyn_cast<ConstantSDNode>(V.getOperand(0))) {
      unsigned UB = V.getConstantOperandVal(1), LB = V.getConstantOperandVal(2);
      Val = VTargetLowering::getBitSlice(CSD->getZExtValue(), UB, LB);
      return UB - LB;
    }

  return 0;
}

// Lower shifting a constant amount:
//   a[31:0] = b[31:0] << 2 => a[31:0] = {b[29:0], 2'b00 }
static SDValue PerformShiftImmCombine(SDNode *N, const VTargetLowering &TLI,
                                      TargetLowering::DAGCombinerInfo &DCI) {
  SelectionDAG &DAG = DCI.DAG;
  SDValue Op = N->getOperand(0);
  SDValue ShiftAmt = N->getOperand(1);
  uint64_t ShiftVal = 0;
  DebugLoc dl = N->getDebugLoc();

  // Can only handle shifting a constant amount.
  if (!ExtractConstant(ShiftAmt, ShiftVal)) {
    unsigned MaxShiftAmtSize = Log2_32_Ceil(TLI.computeSizeInBits(Op));
    unsigned ShiftAmtSize = TLI.computeSizeInBits(ShiftAmt);
    // Limit the shift amount to the the witdh of operand.
    if (ShiftAmtSize > MaxShiftAmtSize) {
      ShiftAmt = VTargetLowering::getBitSlice(DAG, dl, ShiftAmt,
                                              MaxShiftAmtSize, 0,
                                              ShiftAmt.getValueSizeInBits());
      SDValue NewNode = DAG.getNode(N->getOpcode(), dl, N->getVTList(),
                                    Op, ShiftAmt);
      // Replace N by a new value shifted by the right amount, and do not try
      // to combine the user of this node because there is nothing happen in
      // fact.
      DCI.CombineTo(N, NewNode, false);
      return SDValue(N, 0);
    }

    return SDValue();
  }

  // If we not shift at all, simply return the operand.
  if (ShiftVal == 0) return N->getOperand(0);

  EVT VT = N->getValueType(0);
  unsigned PaddingSize = ShiftVal;
  EVT PaddingVT = EVT::getIntegerVT(*DAG.getContext(), PaddingSize);
  // Create this padding bits as target constant.
  SDValue PaddingBits = DAG.getConstant(0, PaddingVT, true);

  unsigned SrcSize = TLI.computeSizeInBits(Op);

  switch (N->getOpcode()) {
  case ISD::SHL: {
    // Discard the higher bits of src.
    Op = TLI.getBitSlice(DAG, dl, Op, SrcSize - PaddingSize, 0);
    DCI.AddToWorklist(Op.getNode());
    return DAG.getNode(VTMISD::BitCat, dl, VT, Op, PaddingBits);
  }
  case ISD::SRA:
    PaddingBits = TLI.getBitRepeat(DAG, dl, TLI.getSignBit(DAG, dl, Op),
                                   PaddingSize);
    // Fall though
  case ISD::SRL:
    // Discard the lower bits of src.
    Op = TLI.getBitSlice(DAG, dl, Op, SrcSize, PaddingSize);
    DCI.AddToWorklist(Op.getNode());
    return DAG.getNode(VTMISD::BitCat, dl, VT, PaddingBits, Op);
  default:
    assert(0 && "Bad opcode!");
    return SDValue();
  }
}

static SDValue PerformAddCombine(SDNode *N, const VTargetLowering &TLI,
                                 TargetLowering::DAGCombinerInfo &DCI,
                                 bool Commuted = false) {

  uint64_t CVal = 0;
  // Can only combinable if carry is known.
  if (!ExtractConstant(N->getOperand(2), CVal))  return SDValue();

  SDValue OpA = N->getOperand(0 ^ Commuted),
          OpB = N->getOperand(1 ^ Commuted);
  
  uint64_t OpBVal = 0;
  if (unsigned OpBSize = ExtractConstant(OpB, OpBVal)) {
    SelectionDAG &DAG = DCI.DAG;

    // A + ~0 + 1 => A - 0 => {1, A}
    if (isAllOnesValue(CVal, 1) && isAllOnesValue(OpBVal, OpBSize)) {
      DCI.CombineTo(N, OpA, DAG.getTargetConstant(1, MVT::i1));
      return SDValue(N, 0);
    }

    // A + 0 + 0 => {0, A}
    if (isNullValue(CVal, 1) && isNullValue(OpBVal, OpBSize)){
      DCI.CombineTo(N, OpA, DAG.getTargetConstant(0, MVT::i1));
      return SDValue(N, 0);
    }
  }

  // TODO: Combine with bit mask information.
  return commuteAndTryAgain(N, TLI, DCI, Commuted, PerformAddCombine);
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

//===--------------------------------------------------------------------===//
// Bit level manipulate function for the BitSlice/BitCat based bit level
// optimization framework.

// Simply concat higher part and lower part.
inline static SDValue ConcatBits(TargetLowering::DAGCombinerInfo &DCI,
                                 SDNode *N, SDValue Hi, SDValue Lo) {
  return DCI.DAG.getNode(VTMISD::BitCat, N->getDebugLoc(), N->getVTList(),
                         Hi, Lo);
}

inline static SDValue ExtractBitSlice(TargetLowering::DAGCombinerInfo &DCI,
                                      SDValue Op, unsigned UB, unsigned LB) {
  SDValue V = VTargetLowering::getBitSlice(DCI.DAG, Op->getDebugLoc(),
                                           Op, UB, LB);
  DCI.AddToWorklist(V.getNode());
  return V;
}

inline static SDValue GetZerosBitSlice(TargetLowering::DAGCombinerInfo &DCI,
                                       SDValue Op, unsigned UB, unsigned LB) {
  EVT HiVT = EVT::getIntegerVT(*DCI.DAG.getContext(), UB - LB);
  return DCI.DAG.getTargetConstant(0, HiVT);
}

inline static SDValue GetOnesBitSlice(TargetLowering::DAGCombinerInfo &DCI,
                                      SDValue Op,  unsigned UB, unsigned LB) {
  EVT VT = EVT::getIntegerVT(*DCI.DAG.getContext(), UB - LB);
  return DCI.DAG.getTargetConstant(~uint64_t(0), VT);
}

inline static SDValue FlipBitSlice(TargetLowering::DAGCombinerInfo &DCI,
                                   SDValue Op, unsigned UB, unsigned LB) {
  SDValue V = VTargetLowering::getBitSlice(DCI.DAG, Op->getDebugLoc(),
                                           Op, UB, LB);
  DCI.AddToWorklist(V.getNode());
  V = VTargetLowering::getNot(DCI.DAG, Op->getDebugLoc(), V);
  DCI.AddToWorklist(V.getNode());
  return V;
}

// FIXME: Allow custom bit concation.
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

  // DirtyHack: ExtractBitMaskInfo cannot handle 0.
  if (Mask == 0)
    return ExtractDisabledBits(DCI, Op, SizeInBits, 0);

  unsigned UB, LB;
  if (ExtractBitMaskInfo(Mask, SizeInBits, UB, LB)) {
    // All enable?
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
    SDValue Result = DAG.getNode(VTMISD::BitCat, dl, VT, HiBits, Lo);
    assert(TLI.computeSizeInBits(Result) == SizeInBits
           && "Bit widht not match!");
    return Result;
  }

  if (flipped) return SDValue();

  // Flip the mask and try again.
  return ExtractBits(Op, ~Mask, TLI, DCI,
                     ExtractDisabledBits, ExtractEnabledBits, !flipped);
}

// Promote bitcat in dag, like {a , b} & {c, d} => {a & c, b &d } or something.
template<typename ConcatBitsFunc>
static SDValue PromoteBinOpBitCat(SDNode *N, const VTargetLowering &TLI,
                                  TargetLowering::DAGCombinerInfo &DCI,
                                  ConcatBitsFunc ConcatBits) {
  SDValue LHS = N->getOperand(0), RHS = N->getOperand(1);

  if (LHS.getOpcode() != VTMISD::BitCat || RHS.getOpcode() != VTMISD::BitCat)
    return SDValue();

  // Split point must agree.
  // FIXME: We can also find the most common part if there is constant.
  SDValue LHSLo = LHS.getOperand(1), RHSLo = RHS.getOperand(1);
  unsigned LHSSplit = VTargetLowering::computeSizeInBits(LHSLo);
  unsigned RHSSplit = VTargetLowering::computeSizeInBits(RHSLo);
  if (LHSSplit != RHSSplit) return SDValue();

  SDValue LHSHi = LHS.getOperand(0), RHSHi = RHS.getOperand(0);
  assert(VTargetLowering::computeSizeInBits(LHSHi) ==
         VTargetLowering::computeSizeInBits(LHSHi) && "Hi size do not match!");

  SelectionDAG &DAG = DCI.DAG;
  DebugLoc dl = N->getDebugLoc();
  SDValue Hi = DAG.getNode(N->getOpcode(), dl, LHSHi.getValueType(),
                           LHSHi, RHSHi);
  DCI.AddToWorklist(Hi.getNode());
  SDValue Lo = DAG.getNode(N->getOpcode(), dl, LHSLo.getValueType(),
                           LHSLo, RHSLo);
  DCI.AddToWorklist(Lo.getNode());

  // FIXME: Allow custom concation.
  return ConcatBits(DCI, N, Hi, Lo);
}

static SDValue PerformLogicCombine(SDNode *N, const VTargetLowering &TLI,
                                   TargetLowering::DAGCombinerInfo &DCI,
                                   bool Commuted = false) {
  SDValue LHS = N->getOperand(0 ^ Commuted),
          RHS = N->getOperand(1 ^ Commuted);

  uint64_t Mask = 0;
  if (ExtractConstant(RHS, Mask)) {
    switch(N->getOpcode()) {
    case ISD::AND:
      return ExtractBits(LHS, Mask, TLI, DCI, ExtractBitSlice, GetZerosBitSlice);
    case ISD::OR:
      return ExtractBits(LHS, Mask, TLI, DCI, GetOnesBitSlice, ExtractBitSlice);
    case ISD::XOR:
      return ExtractBits(LHS, Mask, TLI, DCI, FlipBitSlice, ExtractBitSlice);
    default:
      llvm_unreachable("Unexpected Logic Node!");
    }
  }

  // Try to promote the bitcat after operand commuted.
  if (Commuted) {
    SDValue RV = PromoteBinOpBitCat(N, TLI, DCI, ConcatBits);
    if (RV.getNode()) return RV;
  }

  return commuteAndTryAgain(N, TLI, DCI, Commuted, PerformLogicCombine);
}

static SDValue PerformNotCombine(SDNode *N, const VTargetLowering &TLI,
                                 TargetLowering::DAGCombinerInfo &DCI) {
  SDValue Op = N->getOperand(0);

  // ~(~A) = A.
  if (Op->getOpcode() == VTMISD::Not) return Op->getOperand(0);

  if (Op->getOpcode() == VTMISD::BitCat) {
    SDValue Hi = Op->getOperand(0),
            Lo = Op->getOperand(1);

    Hi = VTargetLowering::getNot(DCI.DAG, N->getDebugLoc(), Hi);
    DCI.AddToWorklist(Hi.getNode());
    Lo = VTargetLowering::getNot(DCI.DAG, N->getDebugLoc(), Lo);
    DCI.AddToWorklist(Lo.getNode());
    return DCI.DAG.getNode(VTMISD::BitCat, N->getDebugLoc(), N->getVTList(),
                           Hi, Lo);
  }

  return SDValue();
}

static SDValue CombineConstants(SelectionDAG &DAG, SDValue &Hi, SDValue &Lo,
                                unsigned ResultWidth) {
  uint64_t LoVal = 0;
  unsigned LoSizeInBits = ExtractConstant(Lo, LoVal);
  if (!LoSizeInBits) return SDValue();
  assert(LoSizeInBits <= 64 && "Lower part of constant too large!");

  uint64_t HiVal = 0;
  unsigned HiSizeInBits = ExtractConstant(Hi, HiVal);
  if (!HiSizeInBits) return SDValue();

  unsigned SizeInBits = LoSizeInBits + HiSizeInBits;
  assert(SizeInBits <= 64 && "Constant too large!");
  uint64_t Val = (LoVal) | (HiVal << LoSizeInBits);

  EVT VT =  EVT::getIntegerVT(*DAG.getContext(), SizeInBits);
  // Use BitSlice to match the type if necessary.
  return VTargetLowering::getBitSlice(DAG, Hi.getDebugLoc(),
                                      DAG.getTargetConstant(Val, VT),
                                      SizeInBits, 0, ResultWidth);
}


static SDValue PerformBitCatCombine(SDNode *N, const VTargetLowering &TLI,
                                    TargetLowering::DAGCombinerInfo &DCI) {
  SDValue Hi = N->getOperand(0),
          Lo = N->getOperand(1);
  SelectionDAG &DAG = DCI.DAG;

  // Dose the node looks like {a[UB-1, M], a[M-1, LB]}? If so, combine it to
  // a[UB-1, LB]
  if (Hi->getOpcode() == VTMISD::BitSlice
      && Lo->getOpcode() == VTMISD::BitSlice) {
    SDValue HiSrc = Hi->getOperand(0);

    if (HiSrc.getNode() == Lo->getOperand(0).getNode()
        && Hi->getConstantOperandVal(2) == Lo->getConstantOperandVal(1))
      return VTargetLowering::getBitSlice(DAG, HiSrc->getDebugLoc(),
                                          HiSrc,
                                          Hi->getConstantOperandVal(1),
                                          Lo->getConstantOperandVal(2),
                                          N->getValueSizeInBits(0));

  }

  // Try to merge the constants.
  return CombineConstants(DAG, Hi, Lo, N->getValueSizeInBits(0));
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

  SelectionDAG &DAG = DCI.DAG;

  // Try to fold the reduction
  uint64_t Val = 0;
  if (unsigned SizeInBits = ExtractConstant(Op, Val)) {
    switch (N->getOpcode()) {
    case VTMISD::ROr:
      // Only reduce to 0 if all bits are 0.
      if (isNullValue(Val, SizeInBits))
        return DAG.getTargetConstant(0, MVT::i1);
      else
        return DAG.getTargetConstant(1, MVT::i1);
    case VTMISD::RAnd:
      // Only reduce to 1 if all bits are 1.
      if (isAllOnesValue(Val, SizeInBits))
        return DAG.getTargetConstant(1, MVT::i1);
      else
        return DAG.getTargetConstant(0, MVT::i1);
    case VTMISD::RXor:
      // Only reduce to 1 if there are odd 1s.
      if (CountPopulation_64(Val) & 0x1)
        return DAG.getTargetConstant(1, MVT::i1);
      else
        return DAG.getTargetConstant(0, MVT::i1);
      break; // FIXME: Who knows how to evaluate this?
    default:  llvm_unreachable("Unexpected Reduction Node!");
    }
  }

  DebugLoc dl = N->getDebugLoc();
  // Reduce high part and low part respectively.
  if (Op->getOpcode() == VTMISD::BitCat) {
    SDValue Hi = Op->getOperand(0), Lo = Op->getOperand(1);
    Hi = VTargetLowering::getReductionOp(DAG, N->getOpcode(), dl, Hi);
    DCI.AddToWorklist(Hi.getNode());
    Lo = VTargetLowering::getReductionOp(DAG, N->getOpcode(), dl, Lo);
    DCI.AddToWorklist(Lo.getNode());
    unsigned Opc = 0;
    switch (N->getOpcode()) {
    case VTMISD::ROr:   Opc = ISD::OR;  break;
    case VTMISD::RAnd:  Opc = ISD::AND; break;
    case VTMISD::RXor:  Opc = ISD::XOR; break;
    default:  llvm_unreachable("Unexpected Reduction Node!");
    }

    return DAG.getNode(Opc, dl, MVT::i1, Hi, Lo);
  }

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
  case ISD::ADDE:
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
