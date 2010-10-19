//===---------- VISelLowering.cpp - VTM DAG Lowering Implementation --------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the interfaces that VTM uses to lower LLVM code
// into a selection DAG.
//
//===----------------------------------------------------------------------===//

#include "VISelLowering.h"
#include "VTargetMachine.h"

#include "llvm/Function.h"
#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
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

//===----------------------------------------------------------------------===//
// Calling Convention Implementation
//===----------------------------------------------------------------------===//

// #include "VGenCallingConv.inc"

//===----------------------------------------------------------------------===//
// TargetLowering Implementation
//===----------------------------------------------------------------------===//

VTargetLowering::VTargetLowering(TargetMachine &TM)
  : TargetLowering(TM, new TargetLoweringObjectFileELF()) {
  setShiftAmountType(MVT::i8);
  setBooleanContents(UndefinedBooleanContent);
  setIntDivIsCheap(false);

  // Set up the legal register classes.
  addRegisterClass(MVT::i1,   VTM::DR1RegisterClass);
  addRegisterClass(MVT::i8,   VTM::DR8RegisterClass);
  addRegisterClass(MVT::i16,  VTM::DR16RegisterClass);
  addRegisterClass(MVT::i32,  VTM::DR32RegisterClass);
  addRegisterClass(MVT::i64,  VTM::DR64RegisterClass);

  computeRegisterProperties();


  setOperationAction(ISD::GlobalAddress, MVT::i32, Custom);
  setOperationAction(ISD::JumpTable,     MVT::i32, Custom);

  // No carry-in operations.
  setOperationAction(ISD::ADDE, MVT::i32, Custom);
  setOperationAction(ISD::SUBE, MVT::i32, Custom);
}

const char *VTargetLowering::getTargetNodeName(unsigned Opcode) const {
  switch (Opcode) {
  default: return 0;
  case VTMISD::InArg: return "VTMISD::InArg";
  case VTMISD::FnRet: return "VTMISD::FnRet";
  }
}

MVT::SimpleValueType VTargetLowering::getSetCCResultType(EVT VT) const {
  return MVT::i1;
}

SDValue
VTargetLowering::LowerFormalArguments(SDValue Chain, CallingConv::ID CallConv,
                                      bool isVarArg,
                                      const SmallVectorImpl<ISD::InputArg> &Ins,
                                      DebugLoc dl, SelectionDAG &DAG,
                                      SmallVectorImpl<SDValue> &InVals) const {
  assert(!isVarArg && "VarArg not support yet!");

  typedef const SmallVectorImpl<ISD::InputArg> IAVec;
  MachineFunction &MF = DAG.getMachineFunction();

  unsigned Idx = 0;
  for (IAVec::const_iterator I = Ins.begin(), E = Ins.end(); I != E; ++I) {
    // Get the argument form InArg Node.

    const ISD::InputArg &IA = *I;
    EVT ArgVT = IA.VT;

    // FIXME: Remember the Argument number.
    SDValue SDInArg = DAG.getNode(VTMISD::InArg, dl, DAG.getVTList(ArgVT, MVT::Other),
                                  Chain, DAG.getConstant(Idx++,MVT::i8, false));

    InVals.push_back(SDInArg);
    // Get the chain from InArg Node.
    Chain = SDInArg.getValue(1);
  }

  return Chain;
}

SDValue
VTargetLowering::LowerReturn(SDValue Chain, CallingConv::ID CallConv,
                             bool isVarArg,
                             const SmallVectorImpl<ISD::OutputArg> &Outs,
                             const SmallVectorImpl<SDValue> &OutVals,
                             DebugLoc dl, SelectionDAG &DAG) const {
  SmallVector<SDValue, 4> RetOps;
  
  RetOps.push_back(Chain);
  RetOps.append(OutVals.begin(), OutVals.end());

  return DAG.getNode(VTMISD::FnRet, dl, MVT::Other, RetOps.data(), RetOps.size());
}

SDValue VTargetLowering::LowerCall(SDValue Chain, SDValue Callee,
                                   CallingConv::ID CallConv, bool isVarArg,
                                   bool &isTailCall,
                                   const SmallVectorImpl<ISD::OutputArg> &Outs,
                                   const SmallVectorImpl<SDValue> &OutVals,
                                   const SmallVectorImpl<ISD::InputArg> &Ins,
                                   DebugLoc dl, SelectionDAG &DAG,
                                   SmallVectorImpl<SDValue> &InVals) const {
  return Chain;
}

// Expansion of ADDE / SUBE. This is a bit involved since blackfin doesn't have
// add-with-carry instructions.
SDValue VTargetLowering::LowerADDE(SDValue Op, SelectionDAG &DAG) const {
  // Operands: lhs, rhs, carry-in
  // Results: sum, carry-out
  return Op;
}

SDValue VTargetLowering::LowerOperation(SDValue Op,
                                               SelectionDAG &DAG) const {
  switch (Op.getOpcode()) {
  default:
    Op.getNode()->dump();
    llvm_unreachable("Should not custom lower this!");
  case ISD::ADDE:
  case ISD::SUBE:               return LowerADDE(Op, DAG);
  }
}

/// getFunctionAlignment - Return the Log2 alignment of this function.
unsigned VTargetLowering::getFunctionAlignment(const Function *F) const {
  return 2;
}
