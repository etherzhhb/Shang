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

#include "vtm/VISelLowering.h"
#include "vtm/VTargetMachine.h"

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
  setSchedulingPreference(Sched::ILP);

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
    // Lower load and store to memory access node.
    setOperationAction(ISD::LOAD, (MVT::SimpleValueType)VT, Custom);
    setOperationAction(ISD::STORE, (MVT::SimpleValueType)VT, Custom);
    // Lower cast node to bit level operation.
    setOperationAction(ISD::SIGN_EXTEND, (MVT::SimpleValueType)VT, Custom);
    setOperationAction(ISD::ZERO_EXTEND, (MVT::SimpleValueType)VT, Custom);
    setOperationAction(ISD::ANY_EXTEND, (MVT::SimpleValueType)VT, Custom);
    setOperationAction(ISD::TRUNCATE, (MVT::SimpleValueType)VT, Custom);
  }


  // Operations not directly supported by VTM.
  setOperationAction(ISD::BR_JT,  MVT::Other, Expand);
  setOperationAction(ISD::BR_CC,  MVT::Other, Expand);
  // Just Lower ISD::BR to BR_CC
  setOperationAction(ISD::BR,     MVT::Other, Custom);

  // Try to perform bit level optimization on these nodes:
  setTargetDAGCombine(ISD::SHL);
  setTargetDAGCombine(ISD::SRA);
  setTargetDAGCombine(ISD::SRL);
}

MVT::SimpleValueType VTargetLowering::getSetCCResultType(EVT VT) const {
  return MVT::i1;
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
  case VTMISD::BitSlice:   return "VTMISD::BitSlice";
  case VTMISD::BitCat:     return "VTMISD::BitCat";
  case VTMISD::BitRepeat:  return "VTMISD::BitRepeat";
  }
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
unsigned VTargetLowering::computeSizeInBits(SDValue Op) const {
  assert(Op.getValueType().isInteger() && "Can not compute size in bit!");
  switch (Op->getOpcode()) {
  default: return Op.getValueSizeInBits();
  case VTMISD::BitSlice:
    return cast<ConstantSDNode>(Op->getOperand(1))->getZExtValue()
           - cast<ConstantSDNode>(Op->getOperand(2))->getZExtValue();
  case VTMISD::BitRepeat:
    return cast<ConstantSDNode>(Op->getOperand(1))->getZExtValue()
           * computeSizeInBits(Op->getOperand(0));
  case VTMISD::BitCat: {
    unsigned SizeInBit = 0;
    for (SDNode::op_iterator I = Op->op_begin(), E = Op->op_end(); I != E; ++I)
      SizeInBit += computeSizeInBits(*I);
    return SizeInBit;
  }
  }
}
SDValue VTargetLowering::getBitSlice(SDValue Op, unsigned UB, unsigned LB,
                                     SelectionDAG &DAG, DebugLoc dl) const {
  LLVMContext &Context = *DAG.getContext();
  unsigned SizeInBits = UB - LB;
  
  assert(SizeInBits < computeSizeInBits(Op) && "Bad bit slice bit width!");
  assert(UB <= computeSizeInBits(Op) && "Bad upper bound of bit slice!");

  EVT VT =
    EVT::getIntegerVT(Context, SizeInBits).getRoundIntegerOrBitType(Context);

  return DAG.getNode(VTMISD::BitSlice, dl, VT, Op,
                     DAG.getConstant(UB, MVT::i8),
                     DAG.getConstant(LB, MVT::i8));
}

SDValue VTargetLowering::getBitRepeat(SDValue Op, unsigned Times,
                                      SelectionDAG &DAG, DebugLoc dl) const {
  LLVMContext &Context = *DAG.getContext();
  unsigned SizeInBits = computeSizeInBits(Op) * Times;
  EVT VT =
    EVT::getIntegerVT(Context, SizeInBits).getRoundIntegerOrBitType(Context);

  return DAG.getNode(VTMISD::BitRepeat, dl, VT, Op,
                     DAG.getConstant(Times, MVT::i8));
}

// Lower br <target> to brcond 1, <target>
SDValue VTargetLowering::LowerBR(SDValue Op, SelectionDAG &DAG) const {
  SDValue Chain = Op.getOperand(0);

  SDValue Cond = DAG.getConstant(1, MVT::i1);

  if (Chain->getOpcode() == ISD::BRCOND)
    Cond = DAG.getNOT(Op.getDebugLoc(), Chain->getOperand(1), MVT::i1);

  return DAG.getNode(ISD::BRCOND, Op.getDebugLoc(), MVT::Other, Chain, Cond,
                     Op.getOperand(1));
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

SDValue VTargetLowering::LowerExtend(SDValue Op, SelectionDAG &DAG,
                                     bool Signed) const {
  SDValue Operand = Op.getOperand(0);
  DebugLoc dl = Operand.getDebugLoc();

  unsigned SrcSize = computeSizeInBits(Operand),
           DstSize = Op.getValueSizeInBits();

  unsigned DiffSize = DstSize - SrcSize;

  SDValue HighBits;
  if (Signed)
    // Fill the high bits with sign bit for signed extend.
    HighBits = getBitRepeat(getSignBit(Operand, DAG, dl), DiffSize, DAG, dl);
  else {
    EVT ConstVT = EVT::getIntegerVT(*DAG.getContext(), DiffSize);
    // Fill the high bits with zeros for signed extend.
    HighBits = DAG.getConstant(0, ConstVT, true);
  }

  return DAG.getNode(VTMISD::BitCat, Op.getDebugLoc(), Op.getValueType(),
                     HighBits, Operand);
}

SDValue VTargetLowering::LowerTruncate(SDValue Op, SelectionDAG &DAG) const {
  SDValue Operand = Op.getOperand(0);
  unsigned SrcSize = computeSizeInBits(Operand),
           DstSize = Op.getValueSizeInBits();

  // Select the lower bit slice to truncate values.
  return getBitSlice(Operand, DstSize, 0, DAG, Op.getDebugLoc());
}

SDValue VTargetLowering::LowerOperation(SDValue Op, SelectionDAG &DAG) const {
  switch (Op.getOpcode()) {
  default:
    llvm_unreachable("Should not custom lower this!");
    return SDValue();
  case ISD::BR:
    return LowerBR(Op, DAG);
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
  case ISD::SIGN_EXTEND:
    return LowerExtend(Op, DAG, true);
  case ISD::ANY_EXTEND:
  case ISD::ZERO_EXTEND:
    return LowerExtend(Op, DAG, false);
  case ISD::TRUNCATE:
    return LowerTruncate(Op, DAG);
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
