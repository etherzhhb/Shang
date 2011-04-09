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

#include "VTargetMachine.h"
#include "vtm/VISelLowering.h"
#include "vtm/Utilities.h"

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
#include "llvm/Support/MathExtras.h"

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
  setBooleanContents(UndefinedBooleanContent);
  setIntDivIsCheap(false);
  setSchedulingPreference(Sched::ILP);

  // Set up the legal register classes.
  addRegisterClass(MVT::i1,   VTM::DRRegisterClass);
  addRegisterClass(MVT::i8,   VTM::DRRegisterClass);
  addRegisterClass(MVT::i16,  VTM::DRRegisterClass);
  addRegisterClass(MVT::i32,  VTM::DRRegisterClass);
  addRegisterClass(MVT::i64,  VTM::DRRegisterClass);

  computeRegisterProperties();


  setOperationAction(ISD::GlobalAddress, MVT::i32, Custom);
  setOperationAction(ISD::JumpTable,     MVT::i32, Custom);

  for (unsigned VT = (unsigned)MVT::FIRST_INTEGER_VALUETYPE;
      VT <= (unsigned)MVT::LAST_INTEGER_VALUETYPE; ++VT) {
    // Lower the add/sub operation to full adder operation.
    setOperationAction(ISD::ADD, (MVT::SimpleValueType)VT, Custom);
    // Expend a - b to a + ~b + 1;
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
    // Just break down the extend load
    setLoadExtAction(ISD::EXTLOAD, (MVT::SimpleValueType)VT, Custom);
    setLoadExtAction(ISD::SEXTLOAD, (MVT::SimpleValueType)VT, Custom);
    setLoadExtAction(ISD::ZEXTLOAD, (MVT::SimpleValueType)VT, Custom);
    
    // Lower cast node to bit level operation.
    setOperationAction(ISD::SIGN_EXTEND, (MVT::SimpleValueType)VT, Custom);
    setOperationAction(ISD::ZERO_EXTEND, (MVT::SimpleValueType)VT, Custom);
    setOperationAction(ISD::ANY_EXTEND, (MVT::SimpleValueType)VT, Custom);
    setOperationAction(ISD::TRUNCATE, (MVT::SimpleValueType)VT, Custom);
    // Condition code will not work.
    setOperationAction(ISD::SELECT_CC, (MVT::SimpleValueType)VT, Expand);
    // Lower SetCC to more fundamental operation.
    setOperationAction(ISD::SETCC, (MVT::SimpleValueType)VT, Custom);

    for (unsigned CC = 0; CC < ISD::SETCC_INVALID; ++CC) {
      setCondCodeAction((ISD::CondCode)CC, (MVT::SimpleValueType)VT, Custom);
    }
    
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

  setTargetDAGCombine(ISD::XOR);
}

MVT::SimpleValueType VTargetLowering::getSetCCResultType(EVT VT) const {
  return MVT::i1;
}

const char *VTargetLowering::getTargetNodeName(unsigned Opcode) const {
  switch (Opcode) {
  default:
    assert(0 && "Unknown SDNode!");
    return "???";
  case VTMISD::InArg:      return "VTMISD::InArg";
  case VTMISD::Ret:        return "VTMISD::Ret";
  case VTMISD::RetVal:     return "VTMISD::RetVal";
  case VTMISD::ADD:        return "VTMISD::ADD";
  case VTMISD::MemAccess:  return "VTMISD::MemAccess";
  case VTMISD::BitSlice:   return "VTMISD::BitSlice";
  case VTMISD::BitCat:     return "VTMISD::BitCat";
  case VTMISD::BitRepeat:  return "VTMISD::BitRepeat";
  case VTMISD::RAnd:       return "VTMISD::RAnd";
  case VTMISD::ROr:        return "VTMISD::ROr";
  case VTMISD::RXor:       return "VTMISD::RXor";
  case VTMISD::Not:        return "VTMISD::Not";
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

  unsigned Idx = 0;
  for (IAVec::const_iterator I = Ins.begin(), E = Ins.end(); I != E; ++I) {
    // Get the argument form InArg Node.

    const ISD::InputArg &IA = *I;
    EVT ArgVT = IA.VT;

    // FIXME: Remember the Argument number.
    SDValue SDInArg = DAG.getNode(VTMISD::InArg, dl,
                                  DAG.getVTList(ArgVT, MVT::Other),
                                  Chain, DAG.getConstant(Idx++,MVT::i8, true));

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
  unsigned Idx = 0;

  for (OAVec::const_iterator I = Outs.begin(), E = Outs.end(); I != E; ++I) {
    // Get the argument form InArg Node.

    const ISD::OutputArg &OA = *I;
    EVT ArgVT = OA.VT;

    // FIXME: Remember the Argument number.
    SDValue SDOutArg = DAG.getNode(VTMISD::RetVal, dl, MVT::Other, Chain,
                                  OutVals[Idx],
                                  DAG.getConstant(Idx,MVT::i8, true));
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
unsigned VTargetLowering::computeSizeInBits(SDValue Op) {
  assert(Op.getValueType().isInteger() && "Bad SDValue type!");

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

SDValue VTargetLowering::getNot(SelectionDAG &DAG, DebugLoc dl,
                                SDValue Operand) {
  EVT VT = Operand.getValueType();

  if (ConstantSDNode *C = dyn_cast<ConstantSDNode>(Operand))
    return DAG.getConstant(~C->getAPIntValue(), VT, 
                           C->getOpcode() == ISD::TargetConstant);

  return DAG.getNode(VTMISD::Not, Operand.getDebugLoc(), VT, Operand);
}

SDValue VTargetLowering::getBitSlice(SelectionDAG &DAG, DebugLoc dl, SDValue Op,
                                     unsigned UB, unsigned LB) {
  LLVMContext &Context = *DAG.getContext();
  unsigned SizeInBits = UB - LB;
  
  assert(SizeInBits < computeSizeInBits(Op) && "Bad bit slice bit width!");
  assert(UB <= computeSizeInBits(Op) && "Bad upper bound of bit slice!");

  EVT VT = EVT::getIntegerVT(Context, SizeInBits);

  if (ConstantSDNode *C = dyn_cast<ConstantSDNode>(Op))
    return DAG.getTargetConstant(GetBitSlice(C->getZExtValue(), UB, LB), VT);

  VT = VT.getRoundIntegerOrBitType(Context);

  return DAG.getNode(VTMISD::BitSlice, dl, VT, Op,
                     DAG.getConstant(UB, MVT::i8, true),
                     DAG.getConstant(LB, MVT::i8, true));
}


SDValue VTargetLowering::getExtend(SelectionDAG &DAG, DebugLoc dl, SDValue SrcOp,
                                   unsigned DstSize, bool Signed) {
  unsigned SrcSize = computeSizeInBits(SrcOp);

  assert(DstSize > SrcSize && "Bad extend operation!");
  unsigned DiffSize = DstSize - SrcSize;

  SDValue HighBits;
  if (Signed)
    // Fill the high bits with sign bit for signed extend.
    HighBits = getBitRepeat(DAG, dl, getSignBit(DAG, dl, SrcOp), DiffSize);
  else {
    EVT ConstVT = EVT::getIntegerVT(*DAG.getContext(), DiffSize);
    // Fill the high bits with zeros for zero extend.
    HighBits = DAG.getConstant(0, ConstVT, true);
  }

  EVT DstVT = EVT::getIntegerVT(*DAG.getContext(), DstSize);
  return DAG.getNode(VTMISD::BitCat, dl, DstVT, HighBits, SrcOp);
}

SDValue VTargetLowering::getBitRepeat(SelectionDAG &DAG, DebugLoc dl, SDValue Op,
                                      unsigned Times) {
  assert(Times > 1 && "Nothing to repeat!");

  LLVMContext &Context = *DAG.getContext();
  unsigned EltBits = computeSizeInBits(Op);
  unsigned SizeInBits = EltBits * Times;
  EVT VT = EVT::getIntegerVT(Context, SizeInBits);

  if (ConstantSDNode *C = dyn_cast<ConstantSDNode>(Op)) {
    uint64_t c = GetBitSlice(C->getZExtValue(), EltBits);
    uint64_t ret = 0;
    for (unsigned i = 0; i < Times; ++i)
      ret |= (c << (i * EltBits));

    return DAG.getTargetConstant(ret, VT);
  }

  VT = VT.getRoundIntegerOrBitType(Context);
  return DAG.getNode(VTMISD::BitRepeat, dl, VT, Op,
                     DAG.getConstant(Times, MVT::i8, true));
}

// Lower br <target> to brcond 1, <target>
SDValue VTargetLowering::LowerBR(SDValue Op, SelectionDAG &DAG) const {
  SDValue Chain = Op.getOperand(0);

  SDValue Cond = DAG.getConstant(1, MVT::i1, true);

  if (Chain->getOpcode() == ISD::BRCOND)
    Cond = getNot(DAG, Op.getDebugLoc(), Chain->getOperand(1));

  return DAG.getNode(ISD::BRCOND, Op.getDebugLoc(), MVT::Other, Chain, Cond,
                     Op.getOperand(1));
}

SDValue VTargetLowering::getCmpResult(SelectionDAG &DAG, SDValue SetCC,
                                      bool dontSub) {
  DebugLoc dl = SetCC.getDebugLoc();
  SDValue LHS = SetCC->getOperand(0), RHS = SetCC->getOperand(1);
  EVT VT = LHS.getValueType();

  SDValue Result = getSub(DAG, dl, VT, LHS, RHS, dontSub);
  // If the user do not want to compute the result from a subtraction,
  // just give them an xor.
  if (!Result.getNode())
    Result = DAG.getNode(ISD::XOR, dl, VT, LHS, RHS);

  return Result;
}

SDValue VTargetLowering::getNNotEQVFlag(SelectionDAG &DAG, SDValue SetCC) {
  DebugLoc dl = SetCC.getDebugLoc();
  SDValue N = getNFlag(DAG, SetCC);
  SDValue V = getVFlag(DAG, SetCC);
  return DAG.getNode(ISD::XOR, dl, MVT::i1, N, V);
}

SDValue VTargetLowering::getVFlag(SelectionDAG &DAG, SDValue SetCC) {
  DebugLoc dl = SetCC.getDebugLoc();
  SDValue LHS = SetCC->getOperand(0), RHS = SetCC->getOperand(1);
  EVT VT = LHS.getValueType();

  SDValue Result = getCmpResult(DAG, SetCC, false);
  //   Sub:
  //   Overflow -> (LHSSign != RHSSign) && (LHSSign != SumSign)
  SDValue LHSSign = getSignBit(DAG,dl, LHS);
  SDValue RHSSign = getSignBit(DAG,dl, RHS);
  SDValue N = getSignBit(DAG, dl, Result);

  return DAG.getNode(ISD::AND, dl, MVT::i1,
                     DAG.getNode(ISD::XOR, dl, MVT::i1,
                                 LHSSign, RHSSign),
                     DAG.getNode(ISD::XOR, dl, MVT::i1,
                                 LHSSign, N));
}

SDValue VTargetLowering::LowerSetCC(SDValue Op, SelectionDAG &DAG) const {
  CondCodeSDNode *Cnd = cast<CondCodeSDNode>(Op->getOperand(2));
  DebugLoc dl = Op.getDebugLoc();

  switch (Cnd->get()) {
  // Z==0
  case ISD::SETNE:  return getNZFlag(DAG, Op, true);
  // Z==1
  case ISD::SETEQ:  return getZFlag(DAG, Op, true);
  // (Z==0) && (N==V)
  case ISD::SETGT:  return DAG.getNode(ISD::AND, dl, MVT::i1,
                                       getNZFlag(DAG, Op),
                                       getNotFlag(DAG, Op, getNNotEQVFlag));
  // N==V
  case ISD::SETGE:  return getNotFlag(DAG, Op, getNNotEQVFlag);
  // N!=V
  case ISD::SETLT:  return getNNotEQVFlag(DAG, Op);
  // (Z==1) || (N!=V)
  case ISD::SETLE:  return DAG.getNode(ISD::OR, dl, MVT::i1,
                                       getZFlag(DAG, Op),
                                       getNNotEQVFlag(DAG, Op));

  // (C==1) && (Z==0)
  case ISD::SETUGT: return DAG.getNode(ISD::AND, dl, MVT::i1,
                                       getCFlag(DAG, Op),
                                       getNZFlag(DAG, Op));
  // 	C==1
  case ISD::SETUGE: return getCFlag(DAG, Op);
  // 	C==0
  case ISD::SETULT: return getNotFlag(DAG, Op, getCFlag);
  // (C==0) || (Z==1)
  case ISD::SETULE: return DAG.getNode(ISD::OR, dl, MVT::i1,
                                       getNotFlag(DAG, Op, getCFlag),
                                       getZFlag(DAG, Op));
  default:
    assert(0 && "Bad condition code!");
    return SDValue();
  }
}


// Lower add to full adder operation.
// Operands: lhs, rhs, carry-in
// Results: sum, carry-out
// Overflow = [16]^[15];
SDValue VTargetLowering::getAdd(SelectionDAG &DAG, DebugLoc dl, EVT VT,
                                SDValue OpA, SDValue OpB,
                                SDValue CarryIn, bool dontCreate) {
  SDVTList VTs = DAG.getVTList(VT, MVT::i1);
  // TODO: Try to fold constant arithmetic.
  // NOTE: We need to perform the arithmetic at the bit width of VT + 1.
  //ConstantSDNode *N1C = dyn_cast<ConstantSDNode>(OpA.getNode());
  //ConstantSDNode *N2C = dyn_cast<ConstantSDNode>(OpB.getNode());
  //ConstantSDNode *CC = dyn_cast<ConstantSDNode>(CarryIn.getNode());

  // Should us create the node?
  SDValue Ops[] = { OpA, OpB, CarryIn };
  if (dontCreate) {
    if (SDNode *N = DAG.getNodeIfExists(VTMISD::ADD, VTs,
                                        Ops, array_lengthof(Ops)))
      return SDValue(N, 0);

    return SDValue();
  }

  return DAG.getNode(VTMISD::ADD, dl, VTs, Ops, array_lengthof(Ops));
}

SDValue VTargetLowering::getAdd(SelectionDAG &DAG, DebugLoc dl, EVT VT,
                                SDValue OpA, SDValue OpB,
                                bool dontCreate) {
  return getAdd(DAG, dl, VT, OpA, OpB, DAG.getConstant(0, MVT::i1, true),
                dontCreate);
}

SDValue VTargetLowering::getSub(SelectionDAG &DAG, DebugLoc dl, EVT VT,
                                SDValue OpA, SDValue OpB,
                                bool dontCreate) {
  return getSub(DAG, dl, VT, OpA, OpB, DAG.getConstant(0, MVT::i1, true),
                dontCreate);
}

SDValue VTargetLowering::getSub(SelectionDAG &DAG, DebugLoc dl, EVT VT,
                                SDValue OpA, SDValue OpB,
                                SDValue CarryIn, bool dontCreate) {
  // A + B = A + (-B) = A + (~B) + 1
  OpB = getNot(DAG, OpB.getDebugLoc(), OpB);
  // FIXME: Is this correct?
  CarryIn = getNot(DAG, CarryIn.getDebugLoc(), CarryIn);

  return getAdd(DAG, dl, VT, OpA, OpB, CarryIn, dontCreate);
}

SDValue VTargetLowering::getReductionOp(SelectionDAG &DAG, unsigned Opc,
                                        DebugLoc dl, SDValue Src) {
  assert((Opc == VTMISD::RAnd || Opc == VTMISD::ROr || Opc == VTMISD::RXor)
         && "Bad opcode!");
  return DAG.getNode(Opc, dl, MVT::i1, Src);
}

SDValue VTargetLowering::LowerMemAccess(SDValue Op, SelectionDAG &DAG,
                                        bool isLoad) const {
  LSBaseSDNode *LSNode = cast<LSBaseSDNode>(Op);
  // FIXME: Handle the index.
  assert(LSNode->isUnindexed() && "Indexed load/store is not supported!");

  EVT VT = LSNode->getMemoryVT();

  unsigned VTSize = VT.getSizeInBits();

  SDValue StoreVal = isLoad ? DAG.getConstant(0, VT, true)
                            : cast<StoreSDNode>(Op)->getValue();

  SDValue SDOps[] = {// The chain.
                     LSNode->getChain(), 
                     // The Value to store (if any), and the address.
                     StoreVal, LSNode->getBasePtr(),
                     // Is load?
                     DAG.getConstant(isLoad, MVT::i1, true),
                     // Byte enable.
                     DAG.getConstant(getByteEnable(VT.getStoreSize()), MVT::i8,
                                     true)
                    };

  unsigned DataBusWidth = vtmfus().getFUDesc<VFUMemBus>()->getDataWidth();
  assert(DataBusWidth >= VTSize && "Unexpected large data!");

  MVT DataBusVT =
    EVT::getIntegerVT(*DAG.getContext(), DataBusWidth).getSimpleVT();  

  DebugLoc dl =  Op.getDebugLoc();
  SDValue Result  =
    DAG.getMemIntrinsicNode(VTMISD::MemAccess, dl,
                            // Result and the chain.
                            DAG.getVTList(DataBusVT, MVT::Other),
                            // SDValue operands
                            SDOps, array_lengthof(SDOps), 
                            // Memory operands.
                            LSNode->getMemoryVT(), LSNode->getMemOperand());
  if (!isLoad)
    return SDValue(Result.getNode(), 1);

  SDValue Val = Result;
  // Truncate the data bus, the system bus should place the valid data start
  // from LSM.
  if (DataBusWidth > VTSize)
    Val = getTruncate(DAG, dl, Result, VTSize);

  // Check if this an extend load.
  LoadSDNode *LD = cast<LoadSDNode>(Op);
  ISD::LoadExtType ExtType = LD->getExtensionType();
  if (ExtType != ISD::NON_EXTLOAD) {
    unsigned DstSize = Op.getValueSizeInBits();
    Val = getExtend(DAG, dl, Val, DstSize, ExtType == ISD::SEXTLOAD);
  }

  // Do we need to replace the result of the load operation?
  if (Result.getNode() != Val.getNode())
    DAG.ReplaceAllUsesOfValueWith(Op, Val);

  return Result;

}

SDValue VTargetLowering::LowerExtend(SDValue Op, SelectionDAG &DAG,
                                     bool Signed) const {
  SDValue Operand = Op.getOperand(0);
  DebugLoc dl = Operand.getDebugLoc();

  unsigned DstSize = Op.getValueSizeInBits();

  return getExtend(DAG, dl, Operand, DstSize, Signed);
}

SDValue VTargetLowering::LowerTruncate(SDValue Op, SelectionDAG &DAG) const {
  SDValue Operand = Op.getOperand(0);
  unsigned DstSize = Op.getValueSizeInBits();

  // Select the lower bit slice to truncate values.
  return getTruncate(DAG, Op.getDebugLoc(), Operand, DstSize);
}

SDValue VTargetLowering::LowerOperation(SDValue Op, SelectionDAG &DAG) const {
  switch (Op.getOpcode()) {
  default:
    llvm_unreachable("Should not custom lower this!");
    return SDValue();
  case ISD::BR:
    return LowerBR(Op, DAG);
  case ISD::SETCC:
    return LowerSetCC(Op, DAG);
  case ISD::LOAD:
    return LowerMemAccess(Op, DAG, true);
  case ISD::STORE:
    return LowerMemAccess(Op, DAG, false);
  case ISD::ADD:
    return getAdd(DAG, Op.getDebugLoc(), Op.getValueType(),
                  Op->getOperand(0), Op->getOperand(1));
  case ISD::SUB:
    return getSub(DAG, Op.getDebugLoc(), Op.getValueType(),
                  Op->getOperand(0), Op->getOperand(1));
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
