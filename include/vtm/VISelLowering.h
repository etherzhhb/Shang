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
    ADD,
    BitSlice,
    BitCat,
    BitRepeat,
    // Memory operations.
    MemAccess = ISD::FIRST_TARGET_MEMORY_OPCODE
  };
}

class VTargetLowering : public TargetLowering {
public:
  VTargetLowering(TargetMachine &TM);
  virtual MVT::SimpleValueType getSetCCResultType(EVT VT) const;
  virtual SDValue LowerOperation(SDValue Op, SelectionDAG &DAG) const;
  SDValue PerformDAGCombine(SDNode *N, DAGCombinerInfo &DCI) const;

  const char *getTargetNodeName(unsigned Opcode) const;
  unsigned getFunctionAlignment(const Function *F) const;

  unsigned computeSizeInBits(SDValue Op) const;

  SDValue getBitSlice(SDValue Op, unsigned UB, unsigned LB,
                      SelectionDAG &DAG, DebugLoc dl) const;
  SDValue getBitRepeat(SDValue Op, unsigned Times,
                       SelectionDAG &DAG, DebugLoc dl) const;

  SDValue getSignBit(SDValue Op, SelectionDAG &DAG, DebugLoc dl) const {
    unsigned SizeInBit = computeSizeInBits(Op);
    return getBitSlice(Op, SizeInBit, SizeInBit - 1, DAG, dl);
  }
private:

  SDValue LowerExtend(SDValue Op, SelectionDAG &DAG, bool Signed) const;
  SDValue LowerTruncate(SDValue Op, SelectionDAG &DAG) const;
  SDValue LowerBR(SDValue Op, SelectionDAG &DAG) const;
  SDValue LowerADDSUB(SDValue Op, SelectionDAG &DAG, SDValue CarrayIn,
                      bool isSub = false) const;

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
