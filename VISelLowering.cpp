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
#include "vtm/VFInfo.h"
#include "vtm/VISelLowering.h"
#include "vtm/Utilities.h"

#include "llvm/Function.h"
#include "llvm/Instructions.h"
#include "llvm/Intrinsics.h"
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
// Helper functions
//===----------------------------------------------------------------------===//
/// GetBits - Retrieve bits between [LB, UB).
static inline uint64_t GetBitSlice(uint64_t x, unsigned UB, unsigned LB = 0) {
  return (x >> LB) & ((1 << (UB - LB)) - 1);
}

/// getRoundIntegerOrBitType - Rounds the bit-width of the given integer EVT
/// up to the nearest power of two (one bit otherwise at least eight bits),
/// and returns the integer EVT with that number of bits.
static EVT getRoundIntegerOrBitType(EVT &VT, LLVMContext &Context) {
  assert(VT.isInteger() && !VT.isVector() && "Invalid integer type!");
  unsigned BitWidth = VT.getSizeInBits();
  if (BitWidth == 1)
    return MVT::i1;
  if (BitWidth <= 8 )
    return EVT(MVT::i8);
  return EVT::getIntegerVT(Context, 1 << Log2_32_Ceil(BitWidth));
}
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

  // Set the intrinsic_wo_chain to be illegal in MVT::Other means it is
  // illegal in all types.
  setOperationAction(ISD::INTRINSIC_WO_CHAIN, MVT::Other, Custom);
  setOperationAction(ISD::INTRINSIC_W_CHAIN, MVT::Other, Custom);
  setOperationAction(ISD::INTRINSIC_VOID, MVT::Other, Custom);

  for (unsigned VT = (unsigned)MVT::FIRST_INTEGER_VALUETYPE;
       VT <= (unsigned)MVT::LAST_INTEGER_VALUETYPE; ++VT) {
    MVT::SimpleValueType SimpleVT = (MVT::SimpleValueType)VT;

    // Lower the add/sub operation to full adder operation.
    setOperationAction(ISD::ADD, SimpleVT, Custom);
    // Expend a - b to a + ~b + 1;
    setOperationAction(ISD::SUB, SimpleVT, Custom);
    setOperationAction(ISD::ADDE, SimpleVT, Custom);
    setOperationAction(ISD::SUBE, SimpleVT, Custom);
    setOperationAction(ISD::ADDC, SimpleVT, Custom);
    setOperationAction(ISD::SUBC, SimpleVT, Custom);
    setOperationAction(ISD::SADDO, SimpleVT, Custom);
    setOperationAction(ISD::SSUBO, SimpleVT, Custom);
    setOperationAction(ISD::UADDO, SimpleVT, Custom);
    setOperationAction(ISD::UADDO, SimpleVT, Custom);

    // Lower load and store to memory access node.
    setOperationAction(ISD::LOAD, SimpleVT, Custom);
    setOperationAction(ISD::STORE, SimpleVT, Custom);
    // Just break down the extend load
    setLoadExtAction(ISD::EXTLOAD, SimpleVT, Custom);
    setLoadExtAction(ISD::SEXTLOAD, SimpleVT, Custom);
    setLoadExtAction(ISD::ZEXTLOAD, SimpleVT, Custom);
    
    // Lower cast node to bit level operation.
    setOperationAction(ISD::SIGN_EXTEND, SimpleVT, Custom);
    setOperationAction(ISD::ZERO_EXTEND, SimpleVT, Custom);
    setOperationAction(ISD::ANY_EXTEND, SimpleVT, Custom);
    setOperationAction(ISD::TRUNCATE, SimpleVT, Custom);
    // Condition code will not work.
    setOperationAction(ISD::SELECT_CC, SimpleVT, Expand);
    // Lower SetCC to more fundamental operation.
    setOperationAction(ISD::SETCC, SimpleVT, Custom);

    for (unsigned CC = 0; CC < ISD::SETCC_INVALID; ++CC) {
      setCondCodeAction((ISD::CondCode)CC, SimpleVT, Custom);
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

  VT = getRoundIntegerOrBitType(VT, Context);

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

  VT = getRoundIntegerOrBitType(VT, Context);
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
  
  // Split the operands if the cannot fit into the function unit.
  unsigned MaxSize = getFUDesc<VFUAddSub>()->getMaxBitWidth(),
           VTSize = VT.getSizeInBits();

  if (VTSize > MaxSize) {
    unsigned HaflVTSize = VTSize / 2;
    EVT HalfVT = MVT::getIntegerVT(HaflVTSize);
    assert(HalfVT.isSimple() && "Expected HalfVT is also a simple VT!");

    SDValue SumL = getAdd(DAG, dl, HalfVT,
                          getBitSlice(DAG, dl, OpA, HaflVTSize, 0),
                          getBitSlice(DAG, dl, OpB, HaflVTSize, 0),
                          CarryIn, dontCreate);
    SDValue HalfCarry = getCarry(SumL);
    SDValue SumH = getAdd(DAG, dl, HalfVT,
                          getBitSlice(DAG, dl, OpA, VTSize, HaflVTSize),
                          getBitSlice(DAG, dl, OpB, VTSize, HaflVTSize),
                          HalfCarry, dontCreate);
    // TODO: Handle the carry.
    return DAG.getNode(VTMISD::BitCat, dl, VT, SumH, SumL);
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
                                        bool isStore) const {
  LSBaseSDNode *LSNode = cast<LSBaseSDNode>(Op);
  // FIXME: Handle the index.
  assert(LSNode->isUnindexed() && "Indexed load/store is not supported!");

  EVT VT = LSNode->getMemoryVT();

  unsigned VTSize = VT.getSizeInBits();

  SDValue StoreVal = isStore ? cast<StoreSDNode>(Op)->getValue()
                             : DAG.getTargetConstant(0, VT);

  SDValue SDOps[] = {// The chain.
                     LSNode->getChain(), 
                     // The Value to store (if any), and the address.
                     LSNode->getBasePtr(), StoreVal,
                     // Is load?
                     DAG.getTargetConstant(isStore, MVT::i1),
                     // Byte enable.
                     DAG.getTargetConstant(getByteEnable(VT.getStoreSize()),
                                           MVT::i8)
                    };

  unsigned DataBusWidth = getFUDesc<VFUMemBus>()->getDataWidth();
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
  if (isStore)
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

SDValue VTargetLowering::LowerINTRINSIC_W_CHAIN(SDValue Op,
                                                SelectionDAG &DAG) const {
  SDValue Chain = Op.getOperand(0);
  unsigned IntNo = Op.getConstantOperandVal(1);
  DebugLoc dl = Op.getDebugLoc();
  switch (IntNo) {
  default: return SDValue();    // Don't custom lower most intrinsics.
  case vtmIntrinsic::vtm_alloca_bram: {
    // FIXME: Provide some uniform method to access the operand of the intrinsics!
    unsigned BRamNum = Op->getConstantOperandVal(2),
             NumElem = Op.getConstantOperandVal(3),
             ElemSizeInBytes = Op.getConstantOperandVal(4);
    
    VFInfo *Info = DAG.getMachineFunction().getInfo<VFInfo>();

    Info->allocateBRam(BRamNum, NumElem, ElemSizeInBytes);

    // Replace the results.
    // The base address inside a block ram is always 0.
    DAG.ReplaceAllUsesOfValueWith(Op, DAG.getTargetConstant(0, getPointerTy()));
    DAG.ReplaceAllUsesOfValueWith(Op.getValue(1), Chain);

    return SDValue();
  }
  }
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
    return LowerMemAccess(Op, DAG, false);
  case ISD::STORE:
    return LowerMemAccess(Op, DAG, true);
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
  case ISD::INTRINSIC_W_CHAIN:
    return LowerINTRINSIC_W_CHAIN(Op, DAG);
  }
}

/// getFunctionAlignment - Return the Log2 alignment of this function.
unsigned VTargetLowering::getFunctionAlignment(const Function *F) const {
  return 2;
}

bool VTargetLowering::getTgtMemIntrinsic(IntrinsicInfo &Info, const CallInst &I,
                                         unsigned Intrinsic) const {
  switch (Intrinsic) {
  default: break;
  case vtmIntrinsic::vtm_alloca_bram: {
    Info.opc = ISD::INTRINSIC_W_CHAIN;
    Info.memVT = getPointerTy();
    Info.ptrVal = &I; // Pass the allocated base address as pointer value.
    Info.offset = 0;
    // Align by block ram cell size.
    Info.align = cast<VAllocaBRamInst>(I).getElementSizeInBytes();
    Info.vol = false;
    Info.readMem = true;
    Info.writeMem = false;
    return true;
  }
  case vtmIntrinsic::vtm_access_bram: {
    VAccessBRamInst &Inst = cast<VAccessBRamInst>(I);

    Info.opc = ISD::INTRINSIC_W_CHAIN;
    Info.memVT = getPointerTy();
    Info.ptrVal = Inst.getPointerOperand();
    Info.offset = 0;
    Info.align = Inst.getAlignment();
    Info.vol = Inst.isVolatile();
    Info.readMem = !Inst.isStore();
    Info.writeMem = Inst.isStore();
    return true;
  }
  }
  
  return false;
}

void VTargetLowering::ReplaceNodeResults(SDNode *N,
                                         SmallVectorImpl<SDValue>&Results,
                                         SelectionDAG &DAG ) const {
  assert(0 && "ReplaceNodeResults not implemented for this target!");

}
