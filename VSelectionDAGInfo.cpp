//===-- VSelectionDAGInfo.cpp - VTM SelectionDAG Info ---------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the VSelectionDAGInfo class.
//
//===----------------------------------------------------------------------===//

#include "VTargetMachine.h"
#include "llvm/LLVMContext.h"
#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/Support/CommandLine.h"
#define DEBUG_TYPE "vtm-selectiondag-info"
#include "llvm/Support/Debug.h"

using namespace llvm;
cl::opt<bool> EnableMemSCM("vtm-enable-memscm",
                           cl::init(true), cl::Hidden);

VSelectionDAGInfo::VSelectionDAGInfo(const VTargetMachine &TM)
  : TargetSelectionDAGInfo(TM) {
}

VSelectionDAGInfo::~VSelectionDAGInfo() {
}

static SDValue EmitMemSCM(unsigned Cmd, SelectionDAG &DAG, DebugLoc dl,
                          SDValue Chain, SDValue Op1, SDValue Op2, SDValue Op3,
                          unsigned Align, bool isVolatile,
                          MachinePointerInfo DstPtrInfo,
                          MachinePointerInfo SrcPtrInfo) {
  if (!EnableMemSCM) return SDValue();

  // Emit the memset command on the membus.
  LLVMContext *Cntx = DAG.getContext();
  EVT CmdVT = EVT::getIntegerVT(*Cntx, VFUMemBus::CMDWidth);
  SDValue SDOps[] = {// The chain.
                     Chain,
                     // Dst pointer and num.
                     Op1, Op3,
                     // CMD
                     DAG.getTargetConstant(Cmd, CmdVT),
                     // Byte enable.
                     DAG.getTargetConstant(VFUMemBus::SeqBegin, MVT::i8)};
  
  unsigned DataWidth = getFUDesc<VFUMemBus>()->getDataWidth();
  
  MVT DataVT = EVT::getIntegerVT(*DAG.getContext(), DataWidth).getSimpleVT();
    
  SDValue MemsetCmd0  =
    DAG.getMemIntrinsicNode(VTMISD::MemAccess, dl,
                            // Result and the chain.
                            DAG.getVTList(DataVT, MVT::Other),
                            // SDValue operands
                            SDOps, array_lengthof(SDOps),
                            // Memory operands.
                            /*FIXME*/MVT::i8, DstPtrInfo, Align, isVolatile,
                            false, true);

  unsigned AddrWidth = getFUDesc<VFUMemBus>()->getAddrWidth();

  SDOps[0] = MemsetCmd0.getValue(1);
  if (Cmd == VFUMemBus::CmdMemSet) {
    SDOps[1] = DAG.getTargetConstant(0, EVT::getIntegerVT(*Cntx, AddrWidth));
    // Value, according to memset fills the block of memory using the
    // unsigned char conversion of this value.
    SDOps[2] = VTargetLowering::getTruncate(DAG, dl, Op2, 8);
  } else {
    // Source pointer.
    SDOps[1] = Op2;
    SDOps[2] = DAG.getTargetConstant(0, EVT::getIntegerVT(*Cntx, DataWidth));
  }

  SDOps[4] = DAG.getTargetConstant(VFUMemBus::SeqEnd, MVT::i8);

  SDValue MemsetCmd1  =
    DAG.getMemIntrinsicNode(VTMISD::MemAccess, dl,
                            // Result and the chain.
                            DAG.getVTList(DataVT, MVT::Other),
                            // SDValue operands
                            SDOps, array_lengthof(SDOps),
                            // Memory operands.
                            /*FIXME*/MVT::i8, SrcPtrInfo, Align, isVolatile,
                            false, true);
  // Return the chain.
  return MemsetCmd1.getValue(1);
}

SDValue
VSelectionDAGInfo::EmitTargetCodeForMemset(SelectionDAG &DAG, DebugLoc dl,
                                           SDValue Chain,
                                           SDValue Op1, SDValue Op2,
                                           SDValue Op3, unsigned Align,
                                           bool isVolatile,
                                           MachinePointerInfo DstPtrInfo) const{
  return EmitMemSCM(VFUMemBus::CmdMemSet, DAG, dl,
                    Chain, Op1, Op2, Op3,
                    Align, isVolatile, DstPtrInfo, MachinePointerInfo());
}

SDValue
VSelectionDAGInfo::EmitTargetCodeForMemcpy(SelectionDAG &DAG, DebugLoc dl,
                                           SDValue Chain,
                                           SDValue Op1, SDValue Op2,
                                           SDValue Op3, unsigned Align,
                                           bool isVolatile, bool AlwaysInline,
                                           MachinePointerInfo DstPtrInfo,
                                           MachinePointerInfo SrcPtrInfo) const{
  return EmitMemSCM(VFUMemBus::CmdMemCpy, DAG, dl,
                    Chain, Op1, Op2, Op3,
                    Align, isVolatile, DstPtrInfo, SrcPtrInfo);
}

SDValue
VSelectionDAGInfo::EmitTargetCodeForMemmove(SelectionDAG &DAG, DebugLoc dl,
                                            SDValue Chain,
                                            SDValue Op1, SDValue Op2,
                                            SDValue Op3, unsigned Align, bool isVolatile,
                                            MachinePointerInfo DstPtrInfo,
                                            MachinePointerInfo SrcPtrInfo)const{
  return EmitMemSCM(VFUMemBus::CmdMemMove, DAG, dl,
                    Chain, Op1, Op2, Op3,
                    Align, isVolatile, DstPtrInfo, SrcPtrInfo);
}
