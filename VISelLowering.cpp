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
#include "vtm/MicroState.h"
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
#include "llvm/CodeGen/RuntimeLibcalls.h"
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
  // Bitwidth information of function units.
  MaxMultBits = getFUDesc<VFUMult>()->getMaxBitWidth();
  MaxAddSubBits = getFUDesc<VFUAddSub>()->getMaxBitWidth();
  MaxShiftBits = getFUDesc<VFUShift>()->getMaxBitWidth();

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
    MVT CurVT = MVT((MVT::SimpleValueType)VT);


    // Lower the add/sub operation to full adder operation.
    setOperationAction(ISD::ADD, CurVT, Custom);
    setOperationAction(ISD::ADDC, CurVT, Custom);
    // Expend a - b to a + ~b + 1;
    setOperationAction(ISD::SUB, CurVT, Custom);
    setOperationAction(ISD::SUBE, CurVT, Expand);
    //setOperationAction(ISD::ADDC, CurVT, Expand);
    setOperationAction(ISD::SUBC, CurVT, Custom);
    // Expand overflow aware operations
    setOperationAction(ISD::SADDO, CurVT, Expand);
    setOperationAction(ISD::SSUBO, CurVT, Expand);
    setOperationAction(ISD::UADDO, CurVT, Expand);
    setOperationAction(ISD::USUBO, CurVT, Expand);

    // We don't have MUL_LOHI
    setOperationAction(ISD::MULHS, CurVT, Expand);
    setOperationAction(ISD::MULHU, CurVT, Expand);
    //setOperationAction(ISD::SMUL_LOHI, CurVT, Expand);
    //setOperationAction(ISD::UMUL_LOHI, CurVT, Expand);

    // We don't have DIV
    setOperationAction(ISD::SDIV, CurVT, Expand);
    setOperationAction(ISD::SDIVREM, CurVT, Expand);
    setOperationAction(ISD::UDIV, CurVT, Expand);
    setOperationAction(ISD::UDIVREM, CurVT, Expand);
    //if (MVT(CurVT).getSizeInBits() > MaxMultBits) {
    //  // Expand the  multiply;
    //  setOperationAction(ISD::MUL, CurVT, Expand);
    //}

    // Lower load and store to memory access node.
    setOperationAction(ISD::LOAD, CurVT, Custom);
    setOperationAction(ISD::STORE, CurVT, Custom);
    // Just break down the extend load
    setLoadExtAction(ISD::EXTLOAD, CurVT, Custom);
    setLoadExtAction(ISD::SEXTLOAD, CurVT, Custom);
    setLoadExtAction(ISD::ZEXTLOAD, CurVT, Custom);
    for (unsigned DVT = (unsigned)MVT::FIRST_INTEGER_VALUETYPE;
         DVT <= VT; ++DVT) {
      MVT DstVT = MVT((MVT::SimpleValueType)DVT);
      setTruncStoreAction(CurVT, DstVT, Custom);
    }

    // Lower cast node to bit level operation.
    setOperationAction(ISD::SIGN_EXTEND, CurVT, Custom);
    setOperationAction(ISD::SIGN_EXTEND_INREG, CurVT, Expand);
    setOperationAction(ISD::ZERO_EXTEND, CurVT, Custom);
    setOperationAction(ISD::ANY_EXTEND, CurVT, Custom);
    setOperationAction(ISD::TRUNCATE, CurVT, Custom);
    // Condition code will not work.
    setOperationAction(ISD::SELECT_CC, CurVT, Expand);
    // Lower SetCC to more fundamental operation.
    setOperationAction(ISD::SETCC, CurVT, Custom);

    for (unsigned CC = 0; CC < ISD::SETCC_INVALID; ++CC)
      setCondCodeAction((ISD::CondCode)CC, CurVT, Custom);
  }

  setLibcallName(RTLIB::UDIV_I8, "__ip_udiv_i8");
  setLibcallName(RTLIB::UDIV_I16, "__ip_udiv_i16");
  setLibcallName(RTLIB::UDIV_I32, "__ip_udiv_i32");
  setLibcallName(RTLIB::UDIV_I64, "__ip_udiv_i64");
  //setLibcallName(RTLIB::UDIV_I128, "__ip_udiv_i128");
  // TODO: SDIV

  // Operations not directly supported by VTM.
  setOperationAction(ISD::BR_JT,  MVT::Other, Expand);
  setOperationAction(ISD::BR_CC,  MVT::Other, Expand);

  // Try to perform bit level optimization on these nodes:
  setTargetDAGCombine(ISD::SHL);
  setTargetDAGCombine(ISD::SRA);
  setTargetDAGCombine(ISD::SRL);

  setTargetDAGCombine(ISD::XOR);
  setTargetDAGCombine(ISD::OR);;
  setTargetDAGCombine(ISD::AND);
  setTargetDAGCombine(ISD::ADDE);
}

MVT::SimpleValueType VTargetLowering::getSetCCResultType(EVT VT) const {
  return MVT::i1;
}

const TargetRegisterClass *VTargetLowering::getRepRegClassFor(EVT VT) const {
  return VTM::DRRegisterClass;
}

uint8_t VTargetLowering::getRepRegClassCostFor(EVT VT) const {
  return 0;
}

const char *VTargetLowering::getTargetNodeName(unsigned Opcode) const {
  switch (Opcode) {
  case VTMISD::InternalCall:    return "VTMISD::InternalCall";
  case VTMISD::ReadReturn:      return "VTMISD::ReadReturn";
  case VTMISD::Ret:             return "VTMISD::Ret";
  case VTMISD::RetVal:          return "VTMISD::RetVal";
  case VTMISD::MemAccess:       return "VTMISD::MemAccess";
  case VTMISD::BitSlice:        return "VTMISD::BitSlice";
  case VTMISD::BitCat:          return "VTMISD::BitCat";
  case VTMISD::BitRepeat:       return "VTMISD::BitRepeat";
  case VTMISD::RAnd:            return "VTMISD::RAnd";
  case VTMISD::ROr:             return "VTMISD::ROr";
  case VTMISD::RXor:            return "VTMISD::RXor";
  case VTMISD::Not:             return "VTMISD::Not";
  default:
    assert(0 && "Unknown SDNode!");
    return "???";
  }
}

SDValue
VTargetLowering::LowerFormalArguments(SDValue Chain, CallingConv::ID CallConv,
                                      bool isVarArg,
                                      const SmallVectorImpl<ISD::InputArg> &Ins,
                                      DebugLoc dl, SelectionDAG &DAG,
                                      SmallVectorImpl<SDValue> &InVals) const {
  assert(!isVarArg && "VarArg not support yet!");
  const Function *F = DAG.getMachineFunction().getFunction();
  Function::const_arg_iterator AI = F->arg_begin();
  assert(Ins.size() == F->arg_size() && "Argument size do not match!");

  for (unsigned I = 0, E = Ins.size(); I != E; ++I) {
    const ISD::InputArg &IA = Ins[I];
    EVT ArgVT = IA.VT;

    const Argument *Arg = AI++;
    const char *ArgName = Arg->getValueName()->getKeyData();
    SDValue SDInArg = DAG.getTargetExternalSymbol(ArgName, ArgVT);
    InVals.push_back(SDInArg);
  }

  return Chain;
}

SDValue
VTargetLowering::LowerReturn(SDValue Chain, CallingConv::ID CallConv,
                             bool isVarArg,
                             const SmallVectorImpl<ISD::OutputArg> &Outs,
                             const SmallVectorImpl<SDValue> &OutVals,
                             DebugLoc dl, SelectionDAG &DAG) const {
  for (unsigned I = 0, E = OutVals.size(); I != E; ++I) {
    const ISD::OutputArg &OA = Outs[I];
    EVT ArgVT = OA.VT;

    // FIXME: Remember the Argument number.
    SDValue SDOutArg = DAG.getNode(VTMISD::RetVal, dl, MVT::Other, Chain,
                                  OutVals[I],
                                  DAG.getTargetConstant(I,MVT::i8));
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
  // Do not mess with tail call.
  isTailCall = false;
  assert(!isVarArg && "VarArg not support yet!");

  // TODO: Handle calling conversions.
  // There are internal call, which means the hardware module corresponding to
  // the callee function should be instantiated inside the current module.
  // And also external call, which means the hardware module corresponding to
  // the callee function is attached to the system bus, and we should send data
  // to it with memory bus.

  SmallVector<SDValue, 8> Ops;
  // Reserve space for the chain and the function name.
  Ops.push_back(Chain);

  VFInfo *VFI = DAG.getMachineFunction().getInfo<VFInfo>();

  unsigned Id;
  if (GlobalAddressSDNode *CalleeNode = dyn_cast<GlobalAddressSDNode>(Callee)) {
    const Function *CalleeFN = cast<Function>(CalleeNode->getGlobal());
    assert(OutVals.size() == CalleeFN->arg_size()
           && "Argument size do not match!");
    // Get the Id for the internal module.
    Id = VFI->getOrCreateCalleeFN(CalleeFN->getName());
    SDValue CalleeFNName
      = DAG.getTargetExternalSymbol(CalleeFN->getValueName()->getKeyData(),
                                    MVT::i64, Id);
    Ops.push_back(CalleeFNName);
  } else {
    ExternalSymbolSDNode *CalleeName = cast<ExternalSymbolSDNode>(Callee);
    Id = VFI->getOrCreateCalleeFN(CalleeName->getSymbol());
    Ops.push_back(DAG.getTargetExternalSymbol(CalleeName->getSymbol(),
                                              MVT::i64, Id));
  }

  // Push others arguments.
  for (unsigned I = 0, E = OutVals.size(); I != E; ++I)
    Ops.push_back(OutVals[I]);

  // The call node return a i1 value as token to keep the dependence between
  // the call and the follow up extract value.
  SDValue CallNode = DAG.getNode(VTMISD::InternalCall, dl,
                                 DAG.getVTList(MVT::i1, MVT::Other),
                                 Ops.data(), Ops.size());

  // Read the return value from return port.
  assert(Ins.size() == 1 && "Can only handle 1 return value at the moment!");
  EVT RetVT = Ins[0].VT;
  SDValue RetPortName = DAG.getTargetExternalSymbol("return_value", RetVT, Id);
  SDValue RetValue = DAG.getNode(VTMISD::ReadReturn, dl, RetVT,
                                 RetPortName, CallNode);
  InVals.push_back(RetValue);

  return SDValue(CallNode.getNode(), 1);
}

unsigned VTargetLowering::computeSizeInBits(SDValue Op) {
  assert(Op.getValueType().isInteger() && "Bad SDValue type!");

  switch (Op->getOpcode()) {
  default: return Op.getValueSizeInBits();
  case VTMISD::Not:
    return computeSizeInBits(Op->getOperand(0));
  case VTMISD::BitSlice:
    return Op->getConstantOperandVal(1) - Op->getConstantOperandVal(2);
  case VTMISD::BitRepeat:
    return Op->getConstantOperandVal(1) * computeSizeInBits(Op->getOperand(0));
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
                                     unsigned UB, unsigned LB,
                                     unsigned ResultWidth){
  LLVMContext &Context = *DAG.getContext();
  unsigned SizeInBits = UB - LB, OpSize = computeSizeInBits(Op);

  assert(SizeInBits <= OpSize && "Bad bit slice bit width!");
  assert(UB <= OpSize && LB < OpSize && "Bad bounds of bit slice!");
  assert(SizeInBits && "BitSlice not contains anything!");
  // If the range contains all bits of the source operand, simple return the
  // source operand, also try to match the result width if necessary.
  if (SizeInBits == OpSize && (ResultWidth == 0 || SizeInBits == ResultWidth))
    return Op;

  EVT VT = EVT::getIntegerVT(Context, SizeInBits);

  if (ConstantSDNode *C = dyn_cast<ConstantSDNode>(Op)) {
    // This may change the value type, which is undesirable during DAG combine.
    if (ResultWidth == SizeInBits)
      return DAG.getTargetConstant(getBitSlice(C->getZExtValue(), UB, LB), VT);
  }

  if (ResultWidth) VT =  EVT::getIntegerVT(Context, ResultWidth);
  else             VT = getRoundIntegerOrBitType(VT, Context);

  return DAG.getNode(VTMISD::BitSlice, dl, VT , Op,
                     DAG.getTargetConstant(UB, MVT::i8),
                     DAG.getTargetConstant(LB, MVT::i8));
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
    uint64_t c = getBitSlice(C->getZExtValue(), EltBits);
    uint64_t ret = 0;
    for (unsigned i = 0; i < Times; ++i)
      ret |= (c << (i * EltBits));

    return DAG.getTargetConstant(ret, VT);
  }

  VT = getRoundIntegerOrBitType(VT, Context);
  return DAG.getNode(VTMISD::BitRepeat, dl, VT, Op,
                     DAG.getConstant(Times, MVT::i8, true));
}

SDValue VTargetLowering::getCmpResult(SelectionDAG &DAG, SDValue SetCC,
                                      bool dontSub) {
  DebugLoc dl = SetCC.getDebugLoc();
  SDValue LHS = SetCC->getOperand(0), RHS = SetCC->getOperand(1);
  SDValue Ops[] = { LHS, RHS };
  EVT VT = LHS.getValueType();

  SDNode *ExistsNode = DAG.getNodeIfExists(ISD::SUBC,
                                           DAG.getVTList(VT, MVT::i1),
                                           Ops, array_lengthof(Ops));

  if (ExistsNode) return SDValue(ExistsNode, 0);

  // If the user do not want to compute the result from a subtraction,
  // just give them an xor.
  if (dontSub) return DAG.getNode(ISD::XOR, dl, VT, LHS, RHS);

  return DAG.getNode(ISD::SUBC, dl, DAG.getVTList(VT, MVT::i1), LHS, RHS);
}

SDValue VTargetLowering::getNNotEQVFlag(SelectionDAG &DAG, SDValue SetCC) {
  DebugLoc dl = SetCC.getDebugLoc();
  SDValue N = getNFlag(DAG, SetCC);
  SDValue V = getVFlag(DAG, SetCC);
  return DAG.getNode(ISD::XOR, dl, MVT::i1, N, V);
}

// Lower add to full adder operation.
// Operands: lhs, rhs, carry-in
// Results: sum, carry-out
// Overflow = [16]^[15];
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
  case ISD::SETGT:  {
    // Create N==V flag, this may create a subtraction.
    SDValue NEQVFlag = getNotFlag(DAG, Op, getNNotEQVFlag);
    // Create Z == 0, we can reuse the previously created subtraction.
    SDValue NZFlag = getNZFlag(DAG, Op);
    return DAG.getNode(ISD::AND, dl, MVT::i1, NZFlag, NEQVFlag);
  }
  // N==V
  case ISD::SETGE:  return getNotFlag(DAG, Op, getNNotEQVFlag);
  // N!=V
  case ISD::SETLT:  return getNNotEQVFlag(DAG, Op);
  // (Z==1) || (N!=V)
  case ISD::SETLE:  return DAG.getNode(ISD::OR, dl, MVT::i1,
                                       getZFlag(DAG, Op),
                                       getNNotEQVFlag(DAG, Op));

  // (C==1) && (Z==0)
  case ISD::SETUGT: {
    // Create C==1 flag, this may create a subtraction.
    SDValue CFlag = getCFlag(DAG, Op);
    // Create Z == 0, we can reuse the previously created subtraction.
    SDValue NZFlag = getNZFlag(DAG, Op);
    return DAG.getNode(ISD::AND, dl, MVT::i1, CFlag, NZFlag);
  }
  // 	C==1
  case ISD::SETUGE: return getCFlag(DAG, Op);
  // 	C==0
  case ISD::SETULT: return getNotFlag(DAG, Op, getCFlag);
  // (C==0) || (Z==1)
  case ISD::SETULE: {
    // Create C==0 flag, this may create a subtraction.
    SDValue NCFlag = getNotFlag(DAG, Op, getCFlag);
    // Create Z == 1, we can reuse the previously created subtraction.
    SDValue ZFlag = getZFlag(DAG, Op);
    return DAG.getNode(ISD::OR, dl, MVT::i1, NCFlag, ZFlag);
  }
  default:
    assert(0 && "Bad condition code!");
    return SDValue();
  }
}

// A - B = A + (-B) = A + (~B) + 1
SDValue VTargetLowering::LowerSub(SDValue Op, SelectionDAG &DAG) const {
  SDValue LHS = Op.getOperand(0), RHS = Op.getOperand(1);
  RHS = getNot(DAG, Op.getDebugLoc(), RHS);
  return DAG.getNode(ISD::ADDE, Op.getDebugLoc(),
                     DAG.getVTList(Op.getValueType(), MVT::i1),
                     LHS, RHS, DAG.getTargetConstant(1, MVT::i1));
}

SDValue VTargetLowering::LowerSubC(SDValue Op, SelectionDAG &DAG) const {
  SDValue LHS = Op.getOperand(0), RHS = Op.getOperand(1);
  RHS = getNot(DAG, Op.getDebugLoc(), RHS);
  return DAG.getNode(ISD::ADDE, Op.getDebugLoc(),
                     DAG.getVTList(Op.getValueType(), MVT::i1),
                     LHS, RHS, DAG.getTargetConstant(1, MVT::i1));
}

SDValue VTargetLowering::LowerAdd(SDValue Op, SelectionDAG &DAG) const {
  return DAG.getNode(ISD::ADDE, Op.getDebugLoc(),
                     DAG.getVTList(Op.getValueType(), MVT::i1),
                     Op.getOperand(0), Op.getOperand(1),
                     DAG.getTargetConstant(0, MVT::i1));
}

SDValue VTargetLowering::LowerAddC(SDValue Op, SelectionDAG &DAG) const {
  SDValue LHS = Op.getOperand(0), RHS = Op.getOperand(1);
  return DAG.getNode(ISD::ADDE, Op.getDebugLoc(),
                     DAG.getVTList(Op.getValueType(), MVT::i1),
                     Op.getOperand(0), Op.getOperand(1), Op->getOperand(2));
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
  case ISD::ADD:
    return LowerAdd(Op, DAG);
  case ISD::ADDC:
    return LowerAddC(Op, DAG);
  case ISD::SUB:
    return LowerSub(Op, DAG);
  case ISD::SUBC:
    return LowerSubC(Op, DAG);
  case ISD::SETCC:
    return LowerSetCC(Op, DAG);
  case ISD::LOAD:
    return LowerMemAccess(Op, DAG, false);
  case ISD::STORE:
    return LowerMemAccess(Op, DAG, true);
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

void VTargetLowering::computeMaskedBitsForTargetNode(const SDValue Op,
                                                     const APInt &Mask,
                                                     APInt &KnownZero,
                                                     APInt &KnownOne,
                                                     const SelectionDAG &DAG,
                                                     unsigned Depth /* = 0 */)
                                                     const {
  unsigned Opc = Op.getOpcode();
  assert((Opc >= ISD::BUILTIN_OP_END ||
          Opc == ISD::INTRINSIC_WO_CHAIN ||
          Opc == ISD::INTRINSIC_W_CHAIN ||
          Opc == ISD::INTRINSIC_VOID) &&
         "Should use MaskedValueIsZero if you don't know whether Op"
         " is a target node!");

  KnownZero = KnownOne = APInt(Mask.getBitWidth(), 0);   // Don't know anything.
  switch (Opc) {
  default: break;
  case VTMISD::Not: {
    SDValue Src = Op->getOperand(0);
    // Swap knownOne and KnownZero and compute compute the mask for the "Not".
    DAG.ComputeMaskedBits(Src, Mask, KnownOne, KnownZero, Depth + 1);
    return;
  }
  case VTMISD::RAnd:
  case VTMISD::ROr: {
    SDValue Src = Op->getOperand(0);
    APInt SrcMask = APInt::getAllOnesValue(Src.getValueSizeInBits());
    APInt SrcZero, SrcOne;
    ComputeSignificantBitMask(Src, SrcMask, SrcZero, SrcOne, DAG, Depth + 1);
    if (Opc == VTMISD::ROr) {
      // Reduce Or return 1 if any bit of its operand is 1, otherwise if all
      // bits is 0, it returns 0.
      if (!SrcOne.isMinValue())      KnownOne.setAllBits();
      else if (SrcZero.isMaxValue()) KnownZero.setAllBits();
    } else {
      // Reduce And return 0 if any bit of its operand is 0, otherwise if all
      // bits is 1, it returns 1.
      if (!SrcZero.isMinValue())     KnownZero.setAllBits();
      else if (SrcOne.isMaxValue())  KnownOne.setAllBits();
    }
    return;
  }
  case VTMISD::BitSlice: {
    SDValue Src = Op->getOperand(0);
    unsigned UB = Op->getConstantOperandVal(1),
             LB = Op->getConstantOperandVal(2);
    unsigned SrcBitWidth = Src.getValueSizeInBits();
    APInt SrcMask = APInt::getBitsSet(SrcBitWidth, LB, UB)
                    & Mask.zextOrTrunc(SrcBitWidth).shl(LB);
    APInt SrcKnownZero, SrcKnownOne;
    DAG.ComputeMaskedBits(Src, SrcMask, SrcKnownZero, SrcKnownOne, Depth + 1);
    unsigned BitWidth = Op.getValueSizeInBits();
    // Return the bitslice of the source masks.
    KnownOne = SrcKnownOne.lshr(LB).zextOrTrunc(BitWidth);
    // The unused bits of BitSlice is 0.
    KnownZero = SrcKnownZero.lshr(LB).zextOrTrunc(BitWidth);
    return;
  }
  case VTMISD::BitCat:{
    unsigned BitWidth = Op.getValueSizeInBits();
    SDValue HiOp = Op->getOperand(0);
    SDValue LoOp = Op->getOperand(1);
    unsigned SplitBit = computeSizeInBits(LoOp);

    APInt HiMask = Mask.lshr(SplitBit).zextOrTrunc(HiOp.getValueSizeInBits());
    APInt HiZero, HiOne;
    ComputeSignificantBitMask(HiOp, HiMask, HiZero, HiOne, DAG, Depth + 1);
    HiZero = HiZero.zext(BitWidth);
    HiOne = HiOne.zext(BitWidth);

    APInt LoMask = Mask.getLoBits(SplitBit).zextOrTrunc(LoOp.getValueSizeInBits());
    APInt LoZero, LoOne;
    ComputeSignificantBitMask(LoOp, LoMask, LoZero, LoOne, DAG, Depth + 1);
    LoZero = LoZero.zext(BitWidth);
    LoOne = LoOne.zext(BitWidth);

    KnownZero = LoZero | HiZero.shl(SplitBit);
    KnownOne = LoOne | HiOne.shl(SplitBit);
    return;
  }
  case VTMISD::BitRepeat:{
    SDValue Src = Op->getOperand(0);
    unsigned SrcBitWidth = Src.getValueSizeInBits();
    assert(SrcBitWidth == 1 && "Cannot handle complex bit pattern!");
    APInt SrcMaks = APInt::getAllOnesValue(SrcBitWidth);
    APInt SrcZero, SrcOne;
    DAG.ComputeMaskedBits(Src, SrcMaks, SrcZero, SrcOne, Depth + 1);

    unsigned Repeat = Op->getConstantOperandVal(1);
    if (SrcOne.isAllOnesValue())
      KnownOne = APInt::getLowBitsSet(KnownOne.getBitWidth(), Repeat);
    if (SrcZero.isAllOnesValue())
      KnownZero = APInt::getLowBitsSet(KnownZero.getBitWidth(), Repeat);
    return;
  }
  }
}

void VTargetLowering::ComputeSignificantBitMask(SDValue Op, const APInt &Mask,
                                                APInt &KnownZero, APInt &KnownOne,
                                                const SelectionDAG &DAG,
                                                unsigned Depth) {
  unsigned SignificantBitWidth = computeSizeInBits(Op);
  unsigned BitWidth = Op.getValueSizeInBits();
  APInt SrcMask = APInt::getLowBitsSet(BitWidth, SignificantBitWidth) & Mask;
  DAG.ComputeMaskedBits(Op, SrcMask, KnownZero, KnownOne, Depth);
  KnownZero = KnownZero.zextOrTrunc(SignificantBitWidth);
  KnownOne = KnownOne.zextOrTrunc(SignificantBitWidth);
}
