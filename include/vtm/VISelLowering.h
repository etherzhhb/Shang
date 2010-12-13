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
    InArg,
    Ret,
    RetVal,
    // Arithmetic operation.
    ADD,
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


  //===--------------------------------------------------------------------===//
  // heterogeneous accelerator architecture bit level SDNodes.
  static unsigned computeSizeInBits(SDValue Op);

  static SDValue getBitSlice(SelectionDAG &DAG, DebugLoc dl, SDValue Op,
                             unsigned UB, unsigned LB);
  
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
  // heterogeneous accelerator architecture arithmetic SDNodes.
  static SDValue getAdd(SelectionDAG &DAG, DebugLoc dl, EVT VT,
                        SDValue OpA, SDValue OpB, SDValue CarryIn,
                        bool dontCreate = false);
  static SDValue getAdd(SelectionDAG &DAG, DebugLoc dl, EVT VT,
                        SDValue OpA, SDValue OpB,
                        bool dontCreate = false);

  static SDValue getSub(SelectionDAG &DAG, DebugLoc dl, EVT VT,
                        SDValue OpA, SDValue OpB, SDValue CarryIn,
                        bool dontCreate = false);
  static SDValue getSub(SelectionDAG &DAG, DebugLoc dl, EVT VT,
                        SDValue OpA, SDValue OpB,
                        bool dontCreate = false);

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
    return SDValue(Result.getNode(), 1);
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

  SDValue LowerExtend(SDValue Op, SelectionDAG &DAG, bool Signed) const;
  SDValue LowerTruncate(SDValue Op, SelectionDAG &DAG) const;
  SDValue LowerBR(SDValue Op, SelectionDAG &DAG) const;

  SDValue LowerSetCC(SDValue Op, SelectionDAG &DAG) const;

  SDValue LowerMemAccess(SDValue Op, SelectionDAG &DAG, bool isLoad) const;

  /// ReplaceNodeResults - Replace the results of node with an illegal result
  /// type with new values built out of custom code.
  ///
  virtual void ReplaceNodeResults(SDNode *N, SmallVectorImpl<SDValue>&Results,
                                  SelectionDAG &DAG) const;

  virtual SDValue
    LowerFormalArguments(SDValue Chain,
    CallingConv::ID CallConv, bool isVarArg,
    const SmallVectorImpl<ISD::InputArg> &Ins,
    DebugLoc dl, SelectionDAG &DAG,
    SmallVectorImpl<SDValue> &InVals) const;
  virtual SDValue
    LowerCall(SDValue Chain, SDValue Callee,
    CallingConv::ID CallConv, bool isVarArg, bool &isTailCall,
    const SmallVectorImpl<ISD::OutputArg> &Outs,
    const SmallVectorImpl<SDValue> &OutVals,
    const SmallVectorImpl<ISD::InputArg> &Ins,
    DebugLoc dl, SelectionDAG &DAG,
    SmallVectorImpl<SDValue> &InVals) const;

  virtual SDValue
    LowerReturn(SDValue Chain,
    CallingConv::ID CallConv, bool isVarArg,
    const SmallVectorImpl<ISD::OutputArg> &Outs,
    const SmallVectorImpl<SDValue> &OutVals,
    DebugLoc dl, SelectionDAG &DAG) const;
};
} // end namespace llvm

#endif    // BLACKFIN_ISELLOWERING_H
