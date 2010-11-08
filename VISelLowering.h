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
#include "VTM.h"

namespace llvm {

namespace VTMISD {
  enum {
    FIRST_NUMBER = ISD::BUILTIN_OP_END,
    InArg,
    FnRet,
    RetVal
  };
}

class VTargetLowering : public TargetLowering {
public:
  VTargetLowering(TargetMachine &TM);
  virtual MVT::SimpleValueType getSetCCResultType(EVT VT) const;
  virtual SDValue LowerOperation(SDValue Op, SelectionDAG &DAG) const;

  const char *getTargetNodeName(unsigned Opcode) const;
  unsigned getFunctionAlignment(const Function *F) const;

private:
  SDValue LowerADDE(SDValue Op, SelectionDAG &DAG) const;

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
