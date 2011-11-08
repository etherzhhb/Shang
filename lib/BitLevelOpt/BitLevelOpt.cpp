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

inline static bool IsConstant(SDValue V) {
  uint64_t Val;
  return ExtractConstant(V, Val) != 0;
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

  // Limit the shift amount.
  if (!ExtractConstant(ShiftAmt, ShiftVal)) {
    unsigned MaxShiftAmtSize = Log2_32_Ceil(TLI.computeSizeInBits(Op));
    unsigned ShiftAmtSize = TLI.computeSizeInBits(ShiftAmt);
    // Limit the shift amount to the the width of operand.
    if (ShiftAmtSize > MaxShiftAmtSize) {
      ShiftAmt = VTargetLowering::getBitSlice(DAG, dl, ShiftAmt,
                                              MaxShiftAmtSize, 0,
                                              ShiftAmt.getValueSizeInBits());
      DCI.AddToWorklist(ShiftAmt.getNode());
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

  unsigned SrcSize = TLI.computeSizeInBits(Op);
  EVT VT = N->getValueType(0);
  unsigned PaddingSize = ShiftVal;
  EVT PaddingVT = EVT::getIntegerVT(*DAG.getContext(), PaddingSize);
  // Create this padding bits as target constant.
  SDValue PaddingBits;

  switch (N->getOpcode()) {
  default:
    PaddingBits = DAG.getConstant(0, PaddingVT, true);
    break;
  case ISD::SRA:
    PaddingBits = TLI.getBitRepeat(DAG, dl, TLI.getSignBit(DAG, dl, Op),
      PaddingSize);
    break;
  case ISD::ROTL:
    PaddingBits = TLI.getBitSlice(DAG, dl, Op, SrcSize, SrcSize - PaddingSize);
    DCI.AddToWorklist(PaddingBits.getNode());
    break;
  case ISD::ROTR:
    PaddingBits = TLI.getBitSlice(DAG, dl, Op, PaddingSize, 0);
    DCI.AddToWorklist(PaddingBits.getNode());
    break;
  }

  switch (N->getOpcode()) {
  case ISD::ROTL:
  case ISD::SHL: {
    // Discard the higher bits of src.
    Op = TLI.getBitSlice(DAG, dl, Op, SrcSize - PaddingSize, 0);
    DCI.AddToWorklist(Op.getNode());
    return DAG.getNode(VTMISD::BitCat, dl, VT, Op, PaddingBits);
  }
  case ISD::ROTR:
  case ISD::SRA:
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

inline static SDValue LogicOpBuildLowPart(TargetLowering::DAGCombinerInfo &DCI,
                                          SDNode *N, SDValue LHS, SDValue RHS,
                                          bool Commuted) {
  SDValue Lo = DCI.DAG.getNode(N->getOpcode(), N->getDebugLoc(),
                               LHS.getValueType(), LHS, RHS);
  DCI.AddToWorklist(Lo.getNode());
  return Lo;
}

inline static SDValue LogicOpBuildHighPart(TargetLowering::DAGCombinerInfo &DCI,
                                           SDNode *N, SDValue LHS, SDValue RHS,
                                           SDValue Lo, bool Commuted) {
  SDValue Hi = DCI.DAG.getNode(N->getOpcode(), N->getDebugLoc(),
                               LHS.getValueType(), LHS, RHS);
  DCI.AddToWorklist(Hi.getNode());
  return Hi;
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
  unsigned SizeInBits = VTargetLowering::computeSizeInBits(Op);

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

static unsigned GetDefaultSplitBit(SDNode *N,
                                   TargetLowering::DAGCombinerInfo &DCI,
                                   SDValue LHS, SDValue &LHSLo, SDValue &LHSHi,
                                   SDValue RHS, SDValue &RHSLo, SDValue &RHSHi){
  if (LHS->getOpcode() != VTMISD::BitCat || RHS->getOpcode() != VTMISD::BitCat)
    return 0;

  unsigned LHSLoBits = VTargetLowering::computeSizeInBits(LHS->getOperand(1));
  unsigned RHSLoBits = VTargetLowering::computeSizeInBits(RHS->getOperand(1));
  if (LHSLoBits != RHSLoBits) return 0;

  unsigned LHSHiBits = VTargetLowering::computeSizeInBits(LHS->getOperand(0));
  unsigned RHSHiBits = VTargetLowering::computeSizeInBits(RHS->getOperand(0));
  assert(LHSHiBits == RHSHiBits && "Hi size do not match!");

  LHSHi = LHS.getOperand(0);
  RHSHi = RHS.getOperand(0);
  // TODO: Force they have the same type!
  if (LHSHi.getValueType() != RHSHi.getValueType())
    return 0;

  LHSLo = LHS.getOperand(1);
  RHSLo = RHS.getOperand(1);
  // TODO: Force they have the same type!
  if (LHSLo.getValueType() != RHSLo.getValueType())
    return 0;

  return LHSLoBits;
}


// Promote bitcat in dag, like {a , b} & {c, d} => {a & c, b &d } or something.
template<typename ConcatBitsFunc,
         typename BuildLowPartFunc,
         typename BuildHighPartFunc,
         typename GetSplitBitFunc>
static SDValue PromoteBinOpBitCat(SDNode *N, const VTargetLowering &TLI,
                                  TargetLowering::DAGCombinerInfo &DCI,
                                  // For some SDNode, we cannot call getbitslice
                                  // on hi and lo.
                                  bool GetBitSliceForHiLo,
                                  ConcatBitsFunc &ConcatBits,
                                  BuildLowPartFunc &BuildLowPart,
                                  BuildHighPartFunc &BuildHighPart,
                                  GetSplitBitFunc &GetSplitBit,
                                  bool Commuted = false) {
  SDValue LHS = N->getOperand(0 ^ Commuted), RHS = N->getOperand(1 ^ Commuted);

  unsigned SizeInBit = VTargetLowering::computeSizeInBits(SDValue(N, 0));
  SDValue LHSLo, RHSLo, LHSHi, RHSHi;
  unsigned LoBitWidth = GetSplitBit(N, DCI,
                                    LHS, LHSLo, LHSHi,
                                    RHS, RHSLo, RHSHi);
  if (LoBitWidth == 0) return SDValue();

  unsigned HiBitWidth = SizeInBit - LoBitWidth;

  SelectionDAG &DAG = DCI.DAG;
  DebugLoc dl = N->getDebugLoc();

  SDValue Lo = BuildLowPart(DCI, N, LHSLo, RHSLo, Commuted);
  if (GetBitSliceForHiLo) {
    Lo = VTargetLowering::getBitSlice(DAG, dl, Lo, LoBitWidth, 0);
    DCI.AddToWorklist(Lo.getNode());
  }

  SDValue Hi = BuildHighPart(DCI, N, LHSHi, RHSHi, Lo, Commuted);
  if (GetBitSliceForHiLo) {
    Hi = VTargetLowering::getBitSlice(DAG, dl, Hi, HiBitWidth, 0);
    DCI.AddToWorklist(Hi.getNode());
  }
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
    SDValue RV = PromoteBinOpBitCat(N, TLI, DCI,
                                    // Get bitslice from hi part and lo before
                                    // concact them.
                                    true,
                                    ConcatBits,
                                    LogicOpBuildLowPart,
                                    LogicOpBuildHighPart,
                                    GetDefaultSplitBit);
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
    SDValue Hi = Op->getOperand(0), Lo = Op->getOperand(1);

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
  SDValue Hi = N->getOperand(0), Lo = N->getOperand(1);
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
    assert(UB <= Op->getConstantOperandVal(1) - Offset && "Broken bitslice!");
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

//===--------------------------------------------------------------------===//
// Arithmetic operations.
// Add
static bool isAddEOpBOneBit(SDValue Op, bool Commuted) {
  SDValue OpB = Op->getOperand(1 ^ Commuted);
  uint64_t OpBVal = 0;
  if (!ExtractConstant(OpB, OpBVal)) return false;

  // FIXME: Use Bit mask information.
  return OpBVal == 0 || OpBVal == 1;
}

inline static SDValue ConcatADDEs(TargetLowering::DAGCombinerInfo &DCI,
                                  SDNode *N, SDValue Hi, SDValue Lo) {
  SDValue NewOp = DCI.DAG.getNode(VTMISD::BitCat, N->getDebugLoc(),
                                  N->getValueType(0), Hi, Lo);
  if (Hi->getOpcode() == VTMISD::BitSlice)
    Hi = Hi->getOperand(0);
  DCI.CombineTo(N, NewOp, Hi.getValue(1));
  return SDValue(N, 0);
}

// Pad the operand of ADDE so we can always get the right carry value.
// for example we have:
// i16 sum, i1 c = i12 a + i12 b + i1 0
// in this case the carry value should be sum[12], to put the carry value to c,
// we can pad the higher bits of a with 1 and the higher bits of a with 0:
// i16 sum, i1 c = {i4 f, i12 a} + {i4 0, i12 b} + i1 0
inline void PadADDEOperand(TargetLowering::DAGCombinerInfo &DCI, DebugLoc dl,
                           SDValue &LHS, SDValue &RHS) {
  SelectionDAG &DAG = DCI.DAG;
  unsigned ActualBits = VTargetLowering::computeSizeInBits(LHS);
  assert(VTargetLowering::computeSizeInBits(LHS)
          == VTargetLowering::computeSizeInBits(RHS)
         && "Bitwidth do not match!");
  if (unsigned DiffBits = LHS.getValueSizeInBits() - ActualBits) {
    // Try to keep RHS as constant.
    if (IsConstant(LHS)) std::swap(LHS, RHS);
    EVT PaddingVT = EVT::getIntegerVT(*DAG.getContext(), DiffBits);
    SDValue LHSPadding = DAG.getTargetConstant(~uint64_t(0), PaddingVT);
    LHS = DAG.getNode(VTMISD::BitCat, dl, LHS.getValueType(), LHSPadding, LHS);
    DCI.AddToWorklist(LHS.getNode());
    uint64_t RHSVal;
    if (ExtractConstant(RHS, RHSVal)) {
      RHS = DAG.getTargetConstant(RHSVal, RHS.getValueType());
    } else {
      SDValue RHSPadding = DAG.getTargetConstant(0, PaddingVT);
      RHS = DAG.getNode(VTMISD::BitCat, dl, RHS.getValueType(), RHSPadding, RHS);
      DCI.AddToWorklist(RHS.getNode());
    }
  }
}

inline static SDValue ADDEBuildLowPart(TargetLowering::DAGCombinerInfo &DCI,
                                       SDNode *N, SDValue LHS, SDValue RHS,
                                       bool Commuted) {
  SelectionDAG &DAG = DCI.DAG;
  DebugLoc dl = N->getDebugLoc();

  PadADDEOperand(DCI, dl, LHS, RHS);

  SDVTList VTs = DAG.getVTList(LHS.getValueType(),  MVT::i1);
  SDValue Lo = DAG.getNode(ISD::ADDE, dl, VTs, LHS, RHS, N->getOperand(2));
  DCI.AddToWorklist(Lo.getNode());
  return Lo;
}

inline static SDValue ADDEBuildHighPart(TargetLowering::DAGCombinerInfo &DCI,
                                        SDNode *N, SDValue LHS, SDValue RHS,
                                        SDValue Lo, bool Commuted) {
  SelectionDAG &DAG = DCI.DAG;
  DebugLoc dl = N->getDebugLoc();
  PadADDEOperand(DCI, dl, LHS, RHS);

  SDVTList VTs = DAG.getVTList(LHS.getValueType(),  MVT::i1);

  if (Lo->getOpcode() == VTMISD::BitSlice)
    Lo = Lo->getOperand(0);

  SDValue LoC = Lo.getValue(1);
  SDValue Hi = DAG.getNode(ISD::ADDE, N->getDebugLoc(), VTs, LHS, RHS, LoC);
  DCI.AddToWorklist(Hi.getNode());
  return Hi;
}

static
unsigned GetADDEBitCatSplitBit(SDNode *N, TargetLowering::DAGCombinerInfo &DCI,
                               SDValue LHS, SDValue &LHSLo, SDValue &LHSHi,
                               SDValue RHS, SDValue &RHSLo, SDValue &RHSHi) {
  unsigned RHSLoBits = VTargetLowering::computeSizeInBits(RHS->getOperand(1));
  unsigned RHSHiBits = VTargetLowering::computeSizeInBits(RHS->getOperand(0));

  // FIXME: Can we also optimize this?
  if (RHSLoBits < RHSHiBits) return 0;

  // C and RHSLo must be constant.
  uint64_t CVal = 0;
  SDValue C = N->getOperand(2);
  if (!ExtractConstant(C, CVal)) return 0;

  RHSHi = RHS.getOperand(0);
  RHSLo = RHS.getOperand(1);

  // Only promote the ADDE when we can drop the lower part.
  uint64_t RHSLoVal = 0;
  if (!ExtractConstant(RHSLo, RHSLoVal)) return 0;

  if (!isNullValue(CVal + RHSLoVal, RHSLoBits)) return 0;

  SelectionDAG &DAG = DCI.DAG;
  DebugLoc dl = N->getDebugLoc();

  LHSLo = VTargetLowering::getBitSlice(DAG, dl, LHS, RHSLoBits, 0);
  DCI.AddToWorklist(LHSLo.getNode());
  // Adjust the bitwidth of constant to match LHS's width.
  if (LHSLo.getValueSizeInBits() != RHSLo.getValueSizeInBits()) {
    RHSLo = VTargetLowering::getBitSlice(DAG, dl, RHSLo, RHSLoBits, 0,
                                         LHSLo.getValueSizeInBits());
    DCI.AddToWorklist(RHSLo.getNode());
  }

  LHSHi = VTargetLowering::getBitSlice(DAG, dl, LHS,
                                       RHSLoBits + RHSHiBits, RHSLoBits);
  DCI.AddToWorklist(LHSHi.getNode());

  return RHSLoBits;
}

static unsigned GetADDESplitBit(SDNode *N, TargetLowering::DAGCombinerInfo &DCI,
                                SDValue LHS, SDValue &LHSLo, SDValue &LHSHi,
                                SDValue RHS, SDValue &RHSLo, SDValue &RHSHi) {
  if (RHS->getOpcode() == VTMISD::BitCat)
    return GetADDEBitCatSplitBit(N, DCI, LHS, LHSLo, LHSHi, RHS, RHSLo, RHSHi);

  // Try to perform: a + 0x8000 => { a[15:14] + 0x1, a[13:0] }
  uint64_t RHSVal = 0;
  if (unsigned SizeInBit = ExtractConstant(RHS, RHSVal)) {
    unsigned SplitBit = CountTrailingZeros_64(RHSVal);
    // It is not profitable to split if the lower zero part is too small.
    if (SplitBit < SizeInBit / 2) return 0;

    DebugLoc dl = N->getDebugLoc();
    SelectionDAG &DAG = DCI.DAG;
    // Build the lower part.
    LHSLo = VTargetLowering::getBitSlice(DAG, dl, LHS, SplitBit, 0);
    DCI.AddToWorklist(LHSLo.getNode());
    RHSLo = VTargetLowering::getBitSlice(DAG, dl, RHS, SplitBit, 0,
                                         LHSLo.getValueSizeInBits());
    DCI.AddToWorklist(RHSLo.getNode());
    // And the higher part.
    LHSHi = VTargetLowering::getBitSlice(DAG, dl, LHS, SizeInBit, SplitBit);
    DCI.AddToWorklist(LHSHi.getNode());
    RHSHi = VTargetLowering::getBitSlice(DAG, dl, RHS, SizeInBit, SplitBit,
                                         LHSHi.getValueSizeInBits());
    DCI.AddToWorklist(RHSHi.getNode());

    return SplitBit;
  }

  return 0;
}

static SDValue PerformAddCombine(SDNode *N, const VTargetLowering &TLI,
                                 TargetLowering::DAGCombinerInfo &DCI,
                                 bool Commuted = false) {
  DebugLoc dl = N->getDebugLoc();
  SelectionDAG &DAG = DCI.DAG;

  uint64_t CVal = 0;

  SDValue OpA = N->getOperand(0 ^ Commuted), OpB = N->getOperand(1 ^ Commuted);
  SDValue C = N->getOperand(2);

  // Expand 1 bit adder to full adder.
  if (VTargetLowering::computeSizeInBits(SDValue(N, 0)) == 1) {
    SDValue AXOrB = DAG.getNode(ISD::XOR, dl, MVT::i1, OpA, OpB);
    SDValue CAndAXOrB = DAG.getNode(ISD::AND, dl, MVT::i1, C, AXOrB);
    SDValue AAndB = DAG.getNode(ISD::AND, dl, MVT::i1, OpA, OpB);
    SDValue NewC = DAG.getNode(ISD::XOR, dl, MVT::i1, AAndB, CAndAXOrB);
    SDValue Sum = DAG.getNode(ISD::XOR, dl, MVT::i1, AXOrB, C);
    DCI.CombineTo(N, Sum, NewC);
    return SDValue(N, 0);
  }

  // Can only combinable if carry is known.
  if (!ExtractConstant(C, CVal)) {
    uint64_t OpAVal = 0, OpBVal = 0;
    // Fold the constant.
    if (unsigned OpASize = ExtractConstant(OpA, OpAVal)) {
      if (unsigned OpBSize = ExtractConstant(OpB, OpBVal)) {
        // 0 + ~0 + carry = {carry, 0}
        if (isNullValue(OpAVal, OpASize) && isAllOnesValue(OpBVal, OpBSize)) {
          DCI.CombineTo(N, DAG.getTargetConstant(0, N->getValueType(0)), C);
          return SDValue(N, 0);
        }
        // TODO: Fold the constant addition.
      }
    }
    return SDValue();
  }


  // A + (B + 1 bit value + 0) + 0 -> A + B + 1'bit value
  if (CVal == 0 && OpB->getOpcode() == ISD::ADDE) {
    uint64_t OpBCVal = 0;
    if (ExtractConstant(OpB->getOperand(2), OpBCVal) && OpBCVal == 0) {
      bool CommuteOpB = false;
      bool isOpBOneBitOnly = false;
      if (!(isOpBOneBitOnly = /*ASSIGNMENT*/ isAddEOpBOneBit(OpB, CommuteOpB))){
        CommuteOpB = true; // Commute and try again.
        isOpBOneBitOnly = isAddEOpBOneBit(OpB, CommuteOpB);
      }

      if (isOpBOneBitOnly) {
        SDValue OpBOpA = OpB->getOperand(0 ^ CommuteOpB),
                OpBOpB = OpB->getOperand(1 ^ CommuteOpB);
        OpBOpB = VTargetLowering::getBitSlice(DAG, dl, OpBOpB, 1, 0);
        DCI.AddToWorklist(OpBOpB.getNode());
        return DAG.getNode(ISD::ADDE, dl, N->getVTList(), OpA, OpBOpA, OpBOpB);
      }
    }
  }

  uint64_t OpBVal = 0;
  if (unsigned OpBSize = ExtractConstant(OpB, OpBVal)) {
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

    if (CVal) {
      // Fold the constant carry to RHS.
      assert((VTargetLowering::getBitSlice(OpBVal, OpBSize) + 1 ==
          VTargetLowering::getBitSlice(OpBVal + 1, std::min(64u, OpBSize + 1)))
             && "Unexpected overflow!");
      OpBVal += 1;
      return DAG.getNode(ISD::ADDE, dl, N->getVTList(),
                         OpA,
                         DAG.getTargetConstant(OpBVal, OpB.getValueType()),
                         DAG.getTargetConstant(0, MVT::i1));
    }
  }

  SDValue RV = PromoteBinOpBitCat(N, TLI, DCI,
                                  // Get bitslice from hi part and lo before
                                  // concact them.
                                  true,
                                  ConcatADDEs,
                                  ADDEBuildLowPart,
                                  ADDEBuildHighPart,
                                  GetADDESplitBit,
                                  Commuted);
  if (RV.getNode()) return RV;

  // TODO: Combine with bit mask information.
  return commuteAndTryAgain(N, TLI, DCI, Commuted, PerformAddCombine);
}

//static void ExpandOperand(TargetLowering::DAGCombinerInfo &DCI, SDValue Op,
//                          SDValue &HiOp, SDValue &LoOp) {
//  unsigned BitWidth = Op.getValueSizeInBits();
//  assert(isPowerOf2_32(BitWidth) && "Cannot handle irregular bitwidth!");
//  unsigned SplitBit = BitWidth / 2;
//  assert(VTargetLowering::computeSizeInBits(Op) > SplitBit
//         && "Cannot expand operand!");
//  HiOp = VTargetLowering::getBitSlice(DCI.DAG, Op.getDebugLoc(), Op,
//                                      BitWidth, SplitBit);
//  DCI.AddToWorklist(HiOp.getNode());
//  LoOp = VTargetLowering::getBitSlice(DCI.DAG, Op.getDebugLoc(), Op,
//                                      SplitBit, 0);
//  DCI.AddToWorklist(LoOp.getNode());
//}
//
//template<typename ConcatBitsFunc,
//         typename BuildLowPartFunc,
//         typename BuildHighPartFunc>
//static SDValue ExpandArithmeticOp(TargetLowering::DAGCombinerInfo &DCI,
//                                  const VTargetLowering &TLI, SDNode *N,
//                                  ConcatBitsFunc ConcatBits,
//                                  BuildLowPartFunc BuildLowPart,
//                                  BuildHighPartFunc BuildHighPart) {
//  SDValue LHS = N->getOperand(0), RHS = N->getOperand(1);
//  SDValue LHSLo, LHSHi;
//  ExpandOperand(DCI, LHS, LHSHi, LHSLo);
//  SDValue RHSLo, RHSHi;
//  ExpandOperand(DCI, RHS, RHSHi, RHSLo);
//  SDValue ADDELo = BuildLowPart(DCI, N, LHSLo, RHSLo);
//  SDValue ADDEHi = BuildHighPart(DCI, N, LHSHi, RHSHi, ADDELo);
//  return ConcatBits(DCI, N, ADDEHi, ADDELo);
//}

//----------------------------------------------------------------------------//
// Function for ICmp combine.
// Split the size of comparison to get smaller latency.
static
unsigned GetICmpBitCatSplitBit(SDNode *N, TargetLowering::DAGCombinerInfo &DCI,
                               SDValue LHS, SDValue &LHSLo, SDValue &LHSHi,
                               SDValue RHS, SDValue &RHSLo, SDValue &RHSHi) {
  unsigned RHSLoBits = VTargetLowering::computeSizeInBits(RHS->getOperand(1));
  unsigned RHSHiBits = VTargetLowering::computeSizeInBits(RHS->getOperand(0));
  RHSHi = RHS.getOperand(0);
  RHSLo = RHS.getOperand(1);

  // Only promote the ICmp when one of them are constant, and we can split the
  // icmp at the middle.
  if (!((IsConstant(RHSLo)|| IsConstant(RHSHi)) && RHSLoBits == RHSHiBits))
    return 0;

  SelectionDAG &DAG = DCI.DAG;
  DebugLoc dl = N->getDebugLoc();

  LHSLo = VTargetLowering::getBitSlice(DAG, dl, LHS, RHSLoBits, 0);
  DCI.AddToWorklist(LHSLo.getNode());
  // Adjust the bitwidth of constant to match LHS's width.
  if (LHSLo.getValueSizeInBits() != RHSLo.getValueSizeInBits()) {
    RHSLo = VTargetLowering::getBitSlice(DAG, dl, RHSLo, RHSLoBits, 0,
                                         LHSLo.getValueSizeInBits());
    DCI.AddToWorklist(RHSLo.getNode());
  }

  LHSHi = VTargetLowering::getBitSlice(DAG, dl, LHS,
                                       RHSLoBits + RHSHiBits, RHSLoBits);
  DCI.AddToWorklist(LHSHi.getNode());

  return RHSLoBits;
}

// KnownBit appears at the lower part, round down the split bit, so lower part
// are all known bit.
static unsigned RoundDownToPowerOf2(unsigned KnownBits, unsigned UB) {
  assert(KnownBits < UB && "Unexpected all known bits!");
  unsigned UnknownBits = UB - KnownBits;
  unsigned RoundUnknownBits = std::max(1u << Log2_32_Ceil(UnknownBits), 8u);
  // Become all unknown after round?
  if (RoundUnknownBits >= UB)
    return 0;

  return UB - RoundUnknownBits;
}

// KnownBit appears at the higher part, round up the split bit, so higher part
// are all known bit.
//static unsigned RoundUpToPowerOf2(unsigned KnownBits, unsigned UB) {
//  assert(KnownBits < UB && "Unexpected all known bits!");
//  unsigned UnknownBits = UB - KnownBits;
//  unsigned RoundUnknownBits = std::max(1u << Log2_32_Ceil(UnknownBits), 8u);
//  // Become all unknown after round?
//  if (RoundUnknownBits >= UB)
//    return 0;
//
//  return RoundUnknownBits;
//}

static unsigned GetICmpRHSConstSplitBit(uint64_t RHSVal, unsigned RHSSize) {
  unsigned SplitBit = CountTrailingZeros_64(RHSVal);
  SplitBit = RoundDownToPowerOf2(SplitBit, RHSSize);
  if (SplitBit >= RHSSize / 2) return SplitBit;

  SplitBit = CountTrailingOnes_64(RHSVal);
  SplitBit = RoundDownToPowerOf2(SplitBit, RHSSize);
  if (SplitBit >= RHSSize / 2) return SplitBit;

  // FIXME: This casue infinte loop in dag combiner.
  //int64_t SignedRHSVal = int64_t(RHSVal << (64 - RHSSize)) >> (64 - RHSSize);
  //unsigned LeadingOnes = CountLeadingOnes_64(SignedRHSVal);
  //if (LeadingOnes) {
  //  LeadingOnes = LeadingOnes - (64 - RHSSize);
  //  SplitBit = RoundUpToPowerOf2(LeadingOnes, RHSSize);
  //  if (SplitBit <= RHSSize / 2) return SplitBit;
  //}

  //unsigned LeadingZeros = (CountLeadingZeros_64(RHSVal) - (64 - RHSSize));
  //SplitBit = RoundUpToPowerOf2(LeadingZeros, RHSSize);
  //if (SplitBit <= RHSSize / 2) return SplitBit;

  return 0;
}

static unsigned GetICmpSplitBit(SDNode *N, TargetLowering::DAGCombinerInfo &DCI,
                                SDValue LHS, SDValue &LHSLo, SDValue &LHSHi,
                                SDValue RHS, SDValue &RHSLo, SDValue &RHSHi) {
  if (RHS->getOpcode() == VTMISD::BitCat)
    return GetICmpBitCatSplitBit(N, DCI, LHS, LHSLo, LHSHi, RHS, RHSLo, RHSHi);

  uint64_t RHSVal = 0;
  unsigned SizeInBit = ExtractConstant(RHS, RHSVal);
  // Only split if RHS is constant.
  if (SizeInBit == 0) return 0;

  // Try lower some comparison like: a > 0x8000 or a > 0x00ff
  unsigned SplitBit = GetICmpRHSConstSplitBit(RHSVal, SizeInBit);
  if (SplitBit  == 0) return 0;

  DebugLoc dl = N->getDebugLoc();
  SelectionDAG &DAG = DCI.DAG;
  // Build the lower part.
  LHSLo = VTargetLowering::getBitSlice(DAG, dl, LHS, SplitBit, 0);
  DCI.AddToWorklist(LHSLo.getNode());
  RHSLo = VTargetLowering::getBitSlice(DAG, dl, RHS, SplitBit, 0,
                                        LHSLo.getValueSizeInBits());
  DCI.AddToWorklist(RHSLo.getNode());
  // And the higher part.
  LHSHi = VTargetLowering::getBitSlice(DAG, dl, LHS, SizeInBit, SplitBit);
  DCI.AddToWorklist(LHSHi.getNode());
  RHSHi = VTargetLowering::getBitSlice(DAG, dl, RHS, SizeInBit, SplitBit,
                                        LHSHi.getValueSizeInBits());
  DCI.AddToWorklist(RHSHi.getNode());

  return SplitBit;
}

// Pad the operand of ICmp so we can always get the right result on signed
// comparision for example we have:
// i1 c = icmp i12 a, i12 b signed_cc
// we can pad the higher bits of a with 1 and the higher bits of a with 0:
// i16 sum, i1 c = signed_ext_to_i16(a) + signed_ext_to_i16(b)
static void PadICmpOperand(TargetLowering::DAGCombinerInfo &DCI, DebugLoc dl,
                           SDValue &LHS, SDValue &RHS, ISD::CondCode CC) {
  SelectionDAG &DAG = DCI.DAG;
  unsigned ActualBits = VTargetLowering::computeSizeInBits(LHS);
  assert(VTargetLowering::computeSizeInBits(LHS)
          == VTargetLowering::computeSizeInBits(RHS)
         && "Bitwidth do not match!");

  EVT VT = VTargetLowering::getRoundIntegerOrBitType(ActualBits,
                                                     *DAG.getContext());
  if (VT.getSizeInBits() > ActualBits) {
    LHS = VTargetLowering::getExtend(DAG, dl, LHS, VT.getSizeInBits(),
      ISD::isSignedIntSetCC(CC));
    DCI.AddToWorklist(LHS.getNode());
    RHS = VTargetLowering::getExtend(DAG, dl, RHS, VT.getSizeInBits(),
      ISD::isSignedIntSetCC(CC));
    DCI.AddToWorklist(RHS.getNode());
  }
}

inline static SDValue BuildICmpLowPart(TargetLowering::DAGCombinerInfo &DCI,
                                       SDNode *N, SDValue LHS, SDValue RHS,
                                       bool Commuted) {
  SelectionDAG &DAG = DCI.DAG;
  DebugLoc dl = N->getDebugLoc();
  CondCodeSDNode *CCNode = cast<CondCodeSDNode>(N->getOperand(2));
  ISD::CondCode CC = CCNode->get();
  if (Commuted) CC = ISD::getSetCCSwappedOperands(CC);

  PadICmpOperand(DCI, dl, LHS, RHS, CC);

  SDValue Lo = DAG.getNode(VTMISD::ICmp, dl, MVT::i1, LHS, RHS,
                           DAG.getCondCode(CC));
  DCI.AddToWorklist(Lo.getNode());
  return Lo;
}

inline static SDValue BuildICmpHighPart(TargetLowering::DAGCombinerInfo &DCI,
                                        SDNode *N, SDValue LHS, SDValue RHS,
                                        SDValue Lo, bool Commuted) {
  SelectionDAG &DAG = DCI.DAG;
  DebugLoc dl = N->getDebugLoc();
  CondCodeSDNode *CCNode = cast<CondCodeSDNode>(N->getOperand(2));
  ISD::CondCode CC = CCNode->get();
  if (Commuted) CC = ISD::getSetCCSwappedOperands(CC);

  PadICmpOperand(DCI, dl, LHS, RHS, CC);

  SDValue HiICmp = DAG.getNode(VTMISD::ICmp, dl, MVT::i1, LHS, RHS,
                               DAG.getCondCode(CC));
  DCI.AddToWorklist(HiICmp.getNode());
  SDValue HiEq = DAG.getNode(VTMISD::ICmp, dl, MVT::i1, LHS, RHS,
                             DAG.getCondCode(ISD::SETEQ));
  DCI.AddToWorklist(HiEq.getNode());
  // Combine the comparison result of lower part by:
  // HiICmp | (HiEq & LoICmp)
  SDValue LoActive = DAG.getNode(ISD::AND, dl, MVT::i1, HiEq, Lo);
  DCI.AddToWorklist(LoActive.getNode());
  return DAG.getNode(ISD::OR, dl, MVT::i1, HiICmp, LoActive);
}

static SDValue ConcatICmps(TargetLowering::DAGCombinerInfo &DCI,
                           SDNode *N, SDValue Hi, SDValue Lo) {
  // The ICmp result return by BuildICmpHighPart.
  return Hi;
}

#define GETLHSNOT(WHAT) GetLHSNot##WHAT

#define DEF_GETLHSNOT(WHAT) \
static SDValue GETLHSNOT(WHAT)(SDValue LHS, EVT RHSVT, SelectionDAG &DAG,\
                               const VTargetLowering &TLI,\
                               TargetLowering::DAGCombinerInfo &DCI) {\
  SDValue LHSWHAT = GetLHS##WHAT(LHS, RHSVT, DAG, TLI, DCI);\
  DCI.AddToWorklist(LHSWHAT.getNode());\
  return TLI.getNot(DAG, LHS->getDebugLoc(), LHSWHAT);\
}

#define GETLHSBINOP(OP, LHS, RHS) GetLHS##LHS##OP##RHS
#define DEF_GETLHSBINOP(OP, LHS, RHS) \
static SDValue GETLHSBINOP(OP, LHS, RHS)(SDValue LHS, EVT RHSVT,\
                                         SelectionDAG &DAG,\
                                         const VTargetLowering &TLI,\
                                         TargetLowering::DAGCombinerInfo &DCI) {\
  /*Work around for GCC does not accept something like "ISD::##OP"*/\
  unsigned OpCOR = ISD::OR;\
  (void) OpCOR;\
  unsigned OpCAND = ISD::AND;\
  (void) OpCAND;\
  SDValue OPLHS = GetLHS##LHS(LHS, RHSVT, DAG, TLI, DCI);\
  DCI.AddToWorklist(OPLHS.getNode());\
  SDValue OPRHS = GetLHS##RHS(LHS, RHSVT, DAG, TLI, DCI);\
  DCI.AddToWorklist(OPRHS.getNode());\
  return DAG.getNode(OpC##OP, LHS->getDebugLoc(), MVT::i1, OPLHS, OPRHS);\
}

static SDValue GetLHSNotZero(SDValue LHS, EVT RHSVT, SelectionDAG &DAG,
                             const VTargetLowering &TLI,
                             TargetLowering::DAGCombinerInfo &DCI) {
  // A value is not zero, if it has some bit set.
  return TLI.getReductionOp(DAG, VTMISD::ROr, LHS->getDebugLoc(), LHS);
}

static SDValue GetLHSAllOnes(SDValue LHS, EVT RHSVT, SelectionDAG &DAG,
                             const VTargetLowering &TLI,
                             TargetLowering::DAGCombinerInfo &DCI) {
  // A value is all ones, if it has all bit set.
  return TLI.getReductionOp(DAG, VTMISD::RAnd, LHS->getDebugLoc(), LHS);
}

static SDValue GetLHSNegative(SDValue LHS, EVT RHSVT, SelectionDAG &DAG,
                              const VTargetLowering &TLI,
                              TargetLowering::DAGCombinerInfo &DCI) {
  return TLI.getSignBit(DAG, LHS->getDebugLoc(), LHS);
}

DEF_GETLHSNOT(AllOnes)
DEF_GETLHSNOT(NotZero)
DEF_GETLHSNOT(Negative)
DEF_GETLHSBINOP(OR, AllOnes, NotNegative)
DEF_GETLHSBINOP(OR, NotNotZero, Negative)
DEF_GETLHSBINOP(AND, NotZero, NotNegative)
DEF_GETLHSBINOP(AND, NotAllOnes, Negative)


static SDValue GetAlwaysFalse(SDValue LHS, EVT RHSVT, SelectionDAG &DAG,
                              const VTargetLowering &TLI,
                              TargetLowering::DAGCombinerInfo &DCI) {
  return DAG.getTargetConstant(0, MVT::i1);
}

static SDValue GetAlwaysTrue(SDValue LHS, EVT RHSVT, SelectionDAG &DAG,
                              const VTargetLowering &TLI,
                              TargetLowering::DAGCombinerInfo &DCI) {
  return DAG.getTargetConstant(1, MVT::i1);
}

template<typename OnRHSZeroFunc, typename OnRHSAllOnesFunc>
static
SDValue PerfromRHSConstantGenericCombine(SDValue LHS, uint64_t RHSVal,
                                         EVT RHSVT, SelectionDAG &DAG,
                                         DebugLoc dl, const VTargetLowering &TLI,
                                         TargetLowering::DAGCombinerInfo &DCI,
                                         OnRHSZeroFunc &OnRHSZero,
                                         OnRHSAllOnesFunc &OnRHSAllOnes) {
  if (isNullValue(RHSVal, RHSVT.getSizeInBits()))
    return OnRHSZero(LHS, RHSVT, DAG, TLI, DCI);
  if (isAllOnesValue(RHSVal, RHSVT.getSizeInBits()))
    return OnRHSAllOnes(LHS, RHSVT, DAG, TLI, DCI);

  return SDValue();
}

static
SDValue PerfromICmpRHSConstantCombine(SDNode *N, const VTargetLowering &TLI,
                                      TargetLowering::DAGCombinerInfo &DCI,
                                      bool Commuted = false) {
  DebugLoc dl = N->getDebugLoc();
  SelectionDAG &DAG = DCI.DAG;
  SDValue LHS = N->getOperand(0 ^ Commuted), RHS = N->getOperand(1 ^ Commuted);
  CondCodeSDNode *CCNode = cast<CondCodeSDNode>(N->getOperand(2));
  ISD::CondCode CC = CCNode->get();
  if (Commuted) CC = ISD::getSetCCSwappedOperands(CC);

  uint64_t RHSVal;

  unsigned RHSSize = ExtractConstant(RHS, RHSVal);
  if (RHSSize) {
    LLVMContext &Cntx = *DAG.getContext();
    EVT RHSVT = EVT::getIntegerVT(Cntx, RHSSize);

    switch (CC) {
    case ISD::SETUGT:
      return PerfromRHSConstantGenericCombine(LHS, RHSVal, RHSVT, DAG, dl, TLI, DCI,
          // Lower a > 0 to a != 0 for unsigned greater than.
                                   GetLHSNotZero,
          // We never have an unsigned value greater than ~0
                                   GetAlwaysFalse);
    case ISD::SETUGE:
      return PerfromRHSConstantGenericCombine(LHS, RHSVal, RHSVT, DAG, dl, TLI, DCI,
          // All unsigned value is greater than or equal to 0.
                                   GetAlwaysTrue,
          // An unsigned value is greater than or equal to ~0 only it is ~0
                                   GetLHSNotAllOnes);
    case ISD::SETULT:
      return PerfromRHSConstantGenericCombine(LHS, RHSVal, RHSVT, DAG, dl, TLI, DCI,
          // We never have an unsigned value less than 0
                                   GetAlwaysFalse,
          // An unsigned value is less than ~0 only it is not ~0
                                   GetLHSNotAllOnes);
    case ISD::SETULE:
      return PerfromRHSConstantGenericCombine(LHS, RHSVal, RHSVT, DAG, dl, TLI, DCI,
          // An unsigned value is less than or equal to 0 only is 0.
                                   GetLHSNotNotZero,
          // All unsigned value is less than or equal to ~0
                                   GetAlwaysTrue);
    case ISD::SETGT:
      return PerfromRHSConstantGenericCombine(LHS, RHSVal, RHSVT, DAG, dl, TLI, DCI,
          // An signed value is greater than 0
          // if its signed bit not set and Not zero.
                                   GetLHSNotZeroANDNotNegative,
          // An signed value is greater than -1
          // if its signed bit not set.
                                   GetLHSNotNegative);
    case ISD::SETGE:
      return PerfromRHSConstantGenericCombine(LHS, RHSVal, RHSVT, DAG, dl, TLI, DCI,
          // An signed value is greater than or equal to 0
          // if its signed bit not set.
                                   GetLHSNotNegative,
          // An signed value is greater than or equal to -1
          // if its -1 or signed bit not set.
                                   GetLHSAllOnesORNotNegative);
    case ISD::SETLT:
      return PerfromRHSConstantGenericCombine(LHS, RHSVal, RHSVT, DAG, dl, TLI, DCI,
          // An signed value is less than  0 if its signed bit set.
                                   GetLHSNegative,
          // An signed value is less than -1 its signed bit set and not equal to -1
                                   GetLHSNotAllOnesANDNegative);
    case ISD::SETLE:
      return PerfromRHSConstantGenericCombine(LHS, RHSVal, RHSVT, DAG, dl, TLI, DCI,
          // An signed value is less than or equal to 0
          // if its signed bit set or it is 0.
                                   GetLHSNotNotZeroORNegative,
          // An signed value is less than  or equal to -1 its signed bit set
                                   GetLHSNegative);
    default: break;
    }
  }

  return commuteAndTryAgain(N, TLI, DCI, Commuted, PerfromICmpRHSConstantCombine);
}

static SDValue PerformICmpSplit(SDNode *N, const VTargetLowering &TLI,
                                TargetLowering::DAGCombinerInfo &DCI,
                                bool Commuted = false) {
  // Can we find optimization opportunity by spliting the comparison?
  SDValue RV = PromoteBinOpBitCat(N, TLI, DCI,
                                  // No need to get bitslice from hi part and lo before
                                  // concact them.
                                  false,
                                  ConcatICmps,
                                  BuildICmpLowPart,
                                  BuildICmpHighPart,
                                  GetICmpSplitBit,
                                  Commuted);
  if (RV.getNode()) return RV;

  return commuteAndTryAgain(N, TLI, DCI, Commuted, PerformICmpSplit);
}

static SDValue PerfromICmpCombine(SDNode *N, const VTargetLowering &TLI,
                                  TargetLowering::DAGCombinerInfo &DCI) {

  DebugLoc dl = N->getDebugLoc();
  SelectionDAG &DAG = DCI.DAG;
  LLVMContext &Cntx = *DAG.getContext();
  SDValue LHS = N->getOperand(0), RHS = N->getOperand(1);

  CondCodeSDNode *CCNode = cast<CondCodeSDNode>(N->getOperand(2));
  ISD::CondCode CC = CCNode->get();

  uint64_t LHSVal, RHSVal;
  unsigned LHSSize = ExtractConstant(LHS, LHSVal);

  unsigned RHSSize = ExtractConstant(RHS, RHSVal);
  // Do we got a constant comparison?
  if (RHSSize && LHSSize) {
    EVT LHSVT = EVT::getIntegerVT(Cntx, LHSSize);
    EVT RHSVT = EVT::getIntegerVT(Cntx, RHSSize);
    return DAG.FoldSetCC(MVT::i1, DAG.getTargetConstant(LHSVal, LHSVT),
                                  DAG.getTargetConstant(LHSVal, RHSVT),
                                  CC, dl);
  }

  // Lower SETNE and SETEQ
  if (CC == ISD::SETNE || CC == ISD::SETEQ) {

    unsigned CmpWidth = VTargetLowering::computeSizeInBits(LHS);
    assert(CmpWidth == VTargetLowering::computeSizeInBits(RHS)
      && "Compare operand with difference width!");

    EVT OperandVT = VTargetLowering::getRoundIntegerOrBitType(CmpWidth, Cntx);

    SDValue NE = DAG.getNode(ISD::XOR, dl, OperandVT, LHS, RHS);
    DCI.AddToWorklist(NE.getNode());
    NE = TLI.getReductionOp(DAG, VTMISD::ROr, dl, NE);
    if (CC == ISD::SETNE) return NE;

    // Else it is a SETEQ, just get it from not(SETNE);
    DCI.AddToWorklist(NE.getNode());
    return TLI.getNot(DAG, dl, NE);
  }

  SDValue RV = PerfromICmpRHSConstantCombine(N, TLI, DCI);
  if (RV.getNode()) return RV;

  return PerformICmpSplit(N, TLI, DCI);
}

SDValue VTargetLowering::PerformDAGCombine(SDNode *N,
                                           TargetLowering::DAGCombinerInfo &DCI)
                                           const {
  switch (N->getOpcode()) {
  case VTMISD::BitCat:
    return PerformBitCatCombine(N, *this, DCI);
  case VTMISD::BitSlice:
    return PerformBitSliceCombine(N, *this, DCI);
  case ISD::ADDE: {
    SDValue RV = PerformAddCombine(N, *this, DCI);
    if (RV.getNode()) return RV;

    // Expansion is disable at the moment.
    //Expand the operation if the ADDE cannot fit into the FU.
    //if (N->getValueSizeInBits(0) > MaxAddSubBits)
    //  return ExpandArithmeticOp(DCI, *this, N, ConcatADDEs,
    //                            ADDEBuildLowPart, ADDEBuildHighPart);
    break;
  }
  case VTMISD::ICmp:
    return PerfromICmpCombine(N, *this, DCI);
  case ISD::ROTL:
  case ISD::ROTR:
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
