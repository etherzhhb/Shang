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

  for (unsigned VT = (unsigned)MVT::FIRST_INTEGER_VALUETYPE;
      VT <= (unsigned)MVT::LAST_INTEGER_VALUETYPE; ++VT) {
    // Lower the add/sub operation to full adder operation.
    setOperationAction(ISD::ADD, (MVT::SimpleValueType)VT, Custom);
    setOperationAction(ISD::SUB, (MVT::SimpleValueType)VT, Custom);
    setOperationAction(ISD::ADDE, (MVT::SimpleValueType)VT, Custom);
    setOperationAction(ISD::SUBE, (MVT::SimpleValueType)VT, Custom);
    setOperationAction(ISD::ADDC, (MVT::SimpleValueType)VT, Custom);
    setOperationAction(ISD::SUBC, (MVT::SimpleValueType)VT, Custom);
    setOperationAction(ISD::SADDO, (MVT::SimpleValueType)VT, Custom);
    setOperationAction(ISD::SSUBO, (MVT::SimpleValueType)VT, Custom);
    setOperationAction(ISD::UADDO, (MVT::SimpleValueType)VT, Custom);
    setOperationAction(ISD::UADDO, (MVT::SimpleValueType)VT, Custom);

    setOperationAction(ISD::LOAD, (MVT::SimpleValueType)VT, Custom);
    setOperationAction(ISD::STORE, (MVT::SimpleValueType)VT, Custom);
  }
}

const char *VTargetLowering::getTargetNodeName(unsigned Opcode) const {
  switch (Opcode) {
  default:
    assert(0 && "Unknown SDNode!");
    return "<???>";
  case VTMISD::InArg:      return "VTMISD::InArg";
  case VTMISD::Ret:        return "VTMISD::Ret";
  case VTMISD::RetVal:     return "VTMISD::RetVal";
  case VTMISD::ADD:        return "VTMISD::ADD";
  case VTMISD::MemAccess:  return "VTMISD::MemAccess";
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
    SDValue SDInArg = DAG.getNode(VTMISD::InArg, dl,
                                  DAG.getVTList(ArgVT, MVT::Other),
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
  typedef const SmallVectorImpl<ISD::OutputArg> OAVec;
  MachineFunction &MF = DAG.getMachineFunction();

  unsigned Idx = 0;
  for (OAVec::const_iterator I = Outs.begin(), E = Outs.end(); I != E; ++I) {
    // Get the argument form InArg Node.

    const ISD::OutputArg &OA = *I;
    EVT ArgVT = OA.VT;

    // FIXME: Remember the Argument number.
    SDValue SDOutArg = DAG.getNode(VTMISD::RetVal, dl, MVT::Other, Chain,
                                  OutVals[Idx],
                                  DAG.getConstant(Idx,MVT::i8, false));
    ++Idx;
    // Get the chain from InArg Node.
    Chain = SDOutArg.getValue(0);
  }

  return DAG.getNode(VTMISD::Ret, dl, MVT::Other, Chain);
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

// Lower add to full adder operation.
// Operands: lhs, rhs, carry-in
// Results: sum, carry-out
// Overflow = [16]^[15];
SDValue VTargetLowering::LowerADDSUB(SDValue Op, SelectionDAG &DAG,
                                     SDValue CarrayIn, bool isSub) const {
  SDValue OpB = Op->getOperand(1);
  
  // A + B = A + (-B) = A + (~B) + 1
  if (isSub) {
    OpB = DAG.getNOT(OpB.getDebugLoc(), OpB, OpB.getValueType());
    // FIXME: Is this true for ADDE?
    // CarrayIn = DAG.getNOT(CarrayIn.getDebugLoc(), CarrayIn, MVT::i1);
  }

  SDValue Result = DAG.getNode(VTMISD::ADD, Op->getDebugLoc(),
                               DAG.getVTList(Op.getValueType(), MVT::i1),
                               Op->getOperand(0), OpB, CarrayIn);
  return Result;
}

SDValue VTargetLowering::LowerMemAccess(SDValue Op, SelectionDAG &DAG,
                                        bool isLoad) const {
  LSBaseSDNode *LSNode = cast<LSBaseSDNode>(Op);
  // FIXME: Handle the index.
  assert(LSNode->isUnindexed() && "Indexed load/store is not supported!");

  EVT VT = isLoad ? Op.getValueType()
                  : cast<StoreSDNode>(Op)->getValue().getValueType();
  
  SDValue StoreVal = isLoad ? DAG.getConstant(0, VT)
                            : cast<StoreSDNode>(Op)->getValue();
  
  SDValue SDOps[] = {// The chain.
                     LSNode->getChain(), 
                     // The Value to store (if any), and the address.
                     StoreVal, LSNode->getBasePtr(),
                     // Is load?
                     DAG.getConstant(isLoad, MVT::i1) };

  SDValue Result  =
    DAG.getMemIntrinsicNode(VTMISD::MemAccess, Op.getDebugLoc(),
                            // Result and the chain.
                            DAG.getVTList(VT, MVT::Other),
                            // SDValue operands
                            SDOps, array_lengthof(SDOps), 
                            // Memory operands.
                            LSNode->getMemoryVT(), LSNode->getMemOperand());

  return isLoad ? Result : SDValue(Result.getNode(), 1);
}

SDValue VTargetLowering::LowerOperation(SDValue Op, SelectionDAG &DAG) const {
  switch (Op.getOpcode()) {
  default:
    llvm_unreachable("Should not custom lower this!");
    return SDValue();
  case ISD::LOAD:
    return LowerMemAccess(Op, DAG, true);
  case ISD::STORE:
    return LowerMemAccess(Op, DAG, false);
  case ISD::SUB:
    return LowerADDSUB(Op, DAG, DAG.getConstant(1, MVT::i1), true);
  case ISD::ADD:
    return LowerADDSUB(Op, DAG, DAG.getConstant(0, MVT::i1));
  case ISD::ADDE:   case ISD::SUBE:   case ISD::ADDC:   case ISD::SUBC:
  case ISD::SADDO:  case ISD::SSUBO:  case ISD::UADDO:  case ISD::USUBO:
    return SDValue();
  }
}

/// getFunctionAlignment - Return the Log2 alignment of this function.
unsigned VTargetLowering::getFunctionAlignment(const Function *F) const {
  return 2;
}

void VTargetLowering::ReplaceNodeResults(SDNode *N,
                                         SmallVectorImpl<SDValue>&Results,
                                         SelectionDAG &DAG ) const {
  assert(0 && "ReplaceNodeResults not implemented for this target!");

}
