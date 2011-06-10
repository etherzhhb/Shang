//===------------ VISelLowering.h - VTM DAG Lowering Interface ----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines the interfaces that Blackfin uses to lower LLVM code into a
// selection DAG.
//
//===----------------------------------------------------------------------===//

#ifndef VISELLOWERING_H
#define VISELLOWERING_H

#include "llvm/Target/TargetLowering.h"
#include "vtm/VTM.h"

namespace llvm {

namespace VTMISD {
  enum {
    FIRST_NUMBER = ISD::BUILTIN_OP_END,
    ReadReturn, // Extract value from a chain.
    Ret,
    RetVal,
    InternalCall,
    // Arithmetic operation.
    //
    Not,
    // Bit level operation.  
    BitSlice,
    BitCat,
    BitRepeat,
    // Reduction logic operation.
    RAnd,
    ROr,
    RXor,
    // Memory operations.
    MemAccess = ISD::FIRST_TARGET_MEMORY_OPCODE
  };
}

class VTargetLowering : public TargetLowering {
  unsigned MaxAddSubBits, MaxMultBits, MaxShiftBits;
public:
  VTargetLowering(TargetMachine &TM);
  
  // TODO:
  virtual bool allowsUnalignedMemoryAccesses(EVT VT) const {
    return false;
  }

  virtual MVT::SimpleValueType getSetCCResultType(EVT VT) const;

  virtual SDValue LowerOperation(SDValue Op, SelectionDAG &DAG) const;

  SDValue PerformDAGCombine(SDNode *N, DAGCombinerInfo &DCI) const;

  const char *getTargetNodeName(unsigned Opcode) const;

  unsigned getFunctionAlignment(const Function *F) const;

  virtual MVT getShiftAmountTy(EVT LHSTy) const { return MVT::i8; }

  virtual void computeMaskedBitsForTargetNode(const SDValue Op, const APInt &Mask,
                                              APInt &KnownZero, APInt &KnownOne,
                                              const SelectionDAG &DAG,
                                              unsigned Depth = 0) const;
  // Narrowing is always profitable.
  virtual bool isNarrowingProfitable(EVT VT1, EVT VT2) const { return true; }
  virtual bool isTruncateFree(EVT VT1, EVT VT2) const { return true; }
  virtual bool isTruncateFree(const Type *Ty1, const Type *Ty2) const {
    return true;
  }
  virtual bool isZExtFree(EVT VT1, EVT VT2) const { return true; }
  virtual bool isZExtFree(const Type *Ty1, const Type *Ty2) const {
    return true;
  }

  //===--------------------------------------------------------------------===//
  // heterogeneous accelerator architecture bit level SDNodes.
  static unsigned computeSizeInBits(SDValue Op);
  static void ComputeSignificantBitMask(SDValue Op, const APInt &Mask,
                                        APInt &KnownZero, APInt &KnownOne,
                                        const SelectionDAG &DAG, unsigned Depth);

  // Get the bit slice in range (UB, LB].
  /// GetBits - Retrieve bits between [LB, UB).
  static inline uint64_t getBitSlice(uint64_t x, unsigned UB, unsigned LB = 0) {
    assert(UB - LB <= 64 && UB <= 64 && "Cannot extract bit slice!");
    // If the bit slice contains the whole 64-bit variable, simply return it.
    if (UB == 64 && LB == 0) return x;

    return (x >> LB) & ((uint64_t(1) << (UB - LB)) - 1);
  }

  static SDValue getBitSlice(SelectionDAG &DAG, DebugLoc dl, SDValue Op,
                             unsigned UB, unsigned LB,
                             unsigned ResultWidth = 0);
  
  static SDValue getBitRepeat(SelectionDAG &DAG, DebugLoc dl, SDValue Op,
                              unsigned Times);

  static SDValue getSignBit(SelectionDAG &DAG, DebugLoc dl, SDValue Op) {
    unsigned SizeInBit = computeSizeInBits(Op);
    return getBitSlice(DAG, dl, Op, SizeInBit, SizeInBit - 1);
  }

  static SDValue getTruncate(SelectionDAG &DAG, DebugLoc dl, SDValue SrcOp,
                      unsigned DstSize) {
    return getBitSlice(DAG, dl, SrcOp, DstSize, 0);
  }

  static SDValue getExtend(SelectionDAG &DAG, DebugLoc dl, SDValue SrcOp,
                           unsigned DstSize, bool Signed);

  static SDValue getReductionOp(SelectionDAG &DAG, unsigned Opc, DebugLoc dl,
                                SDValue Src);

  static SDValue getNot(SelectionDAG &DAG, DebugLoc dl, SDValue Operand);

  //===--------------------------------------------------------------------===//
  // Helper function for comparison lowering.
  static SDValue getCmpResult(SelectionDAG &DAG, SDValue SetCC, bool dontSub);

  // Not Zero.
  static SDValue getNZFlag(SelectionDAG &DAG, SDValue SetCC,
                           bool dontSub = false){
    DebugLoc dl = SetCC.getDebugLoc();               
    return getReductionOp(DAG, VTMISD::ROr, dl,
                          getCmpResult(DAG, SetCC, dontSub));
  }

  // The zero flag.
  static SDValue getZFlag(SelectionDAG &DAG, SDValue SetCC,
                          bool dontSub = false) {
    DebugLoc dl = SetCC.getDebugLoc();
    return getNot(DAG, dl, getNZFlag(DAG, SetCC, dontSub));
  }

  template<class Func>
  static SDValue getNotFlag(SelectionDAG &DAG, SDValue SetCC, Func F) {  
    DebugLoc dl = SetCC.getDebugLoc();               
    return getNot(DAG, dl, F(DAG, SetCC));
  }

  // Carry (or Unsigned Overflow).
  static SDValue getCFlag(SelectionDAG &DAG, SDValue SetCC) {
    SDValue Result = getCmpResult(DAG, SetCC, false);
    return Result.getValue(1);
  }

  // The negative flag.
  static SDValue getNFlag(SelectionDAG &DAG, SDValue SetCC) {
    DebugLoc dl = SetCC.getDebugLoc();
    SDValue Result = getCmpResult(DAG, SetCC, false);
    return getSignBit(DAG, dl, Result);
  }

  // The signed overflow flag.
  static SDValue getVFlag(SelectionDAG &DAG, SDValue SetCC);

  // Negative not equal signed overflow.
  static SDValue getNNotEQVFlag(SelectionDAG &DAG, SDValue SetCC);

private:
  SDValue LowerAdd(SDValue Op, SelectionDAG &DAG) const;
  SDValue LowerAddC(SDValue Op, SelectionDAG &DAG) const;
  SDValue LowerSubC(SDValue Op, SelectionDAG &DAG) const;
  SDValue LowerExtend(SDValue Op, SelectionDAG &DAG, bool Signed) const;
  SDValue LowerTruncate(SDValue Op, SelectionDAG &DAG) const;

  SDValue LowerSetCC(SDValue Op, SelectionDAG &DAG) const;

  SDValue LowerMemAccess(SDValue Op, SelectionDAG &DAG, bool isStore) const;

  SDValue LowerINTRINSIC_W_CHAIN(SDValue Op, SelectionDAG &DAG) const;
  SDValue LowerINTRINSIC_WO_CHAIN(SDValue Op, SelectionDAG &DAG) const;
  /// ReplaceNodeResults - Replace the results of node with an illegal result
  /// type with new values built out of custom code.
  ///
  virtual void ReplaceNodeResults(SDNode *N, SmallVectorImpl<SDValue>&Results,
                                  SelectionDAG &DAG) const;

  virtual SDValue LowerFormalArguments(SDValue Chain, CallingConv::ID CallConv,
                                       bool isVarArg,
                                       const SmallVectorImpl<ISD::InputArg> &Ins,
                                       DebugLoc dl, SelectionDAG &DAG,
                                       SmallVectorImpl<SDValue> &InVals) const;
  virtual SDValue LowerCall(SDValue Chain, SDValue Callee,
                            CallingConv::ID CallConv, bool isVarArg,
                            bool &isTailCall,
                            const SmallVectorImpl<ISD::OutputArg> &Outs,
                            const SmallVectorImpl<SDValue> &OutVals,
                            const SmallVectorImpl<ISD::InputArg> &Ins,
                            DebugLoc dl, SelectionDAG &DAG,
                            SmallVectorImpl<SDValue> &InVals) const;

  virtual SDValue LowerReturn(SDValue Chain, CallingConv::ID CallConv,
                              bool isVarArg,
                              const SmallVectorImpl<ISD::OutputArg> &Outs,
                              const SmallVectorImpl<SDValue> &OutVals,
                              DebugLoc dl, SelectionDAG &DAG) const;

  virtual bool getTgtMemIntrinsic(IntrinsicInfo &Info, const CallInst &I,
                                  unsigned Intrinsic) const;
};
} // end namespace llvm

#endif    // BLACKFIN_ISELLOWERING_H
