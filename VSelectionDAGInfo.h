//===--------- VSelectionDAGInfo.h - VTM SelectionDAG Info ------*- C++ -*-===//
//
//                      The Shang HLS frameowrk                               //
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines the Blackfin subclass for TargetSelectionDAGInfo.
//
//===----------------------------------------------------------------------===//

#ifndef VINSELECTIONDAGINFO_H
#define VINSELECTIONDAGINFO_H

#include "llvm/Target/TargetSelectionDAGInfo.h"

namespace llvm {

class VTargetMachine;

class VSelectionDAGInfo : public TargetSelectionDAGInfo {
public:
  explicit VSelectionDAGInfo(const VTargetMachine &TM);
  ~VSelectionDAGInfo();

  /// EmitTargetCodeForMemset - Emit target-specific code that performs a
  /// memset. This can be used by targets to provide code sequences for cases
  /// that don't fit the target's parameters for simple stores and can be more
  /// efficient than using a library call. This function can return a null
  /// SDValue if the target declines to use custom code and a different
  /// lowering strategy should be used.
  SDValue EmitTargetCodeForMemset(SelectionDAG &DAG, DebugLoc dl,
                                  SDValue Chain,
                                  SDValue Op1, SDValue Op2,
                                  SDValue Op3, unsigned Align, bool isVolatile,
                                  MachinePointerInfo DstPtrInfo) const;

  /// EmitTargetCodeForMemmove - Emit target-specific code that performs a
  /// memmove. This can be used by targets to provide code sequences for cases
  /// that don't fit the target's parameters for simple loads/stores and can be
  /// more efficient than using a library call. This function can return a null
  /// SDValue if the target declines to use custom code and a different
  /// lowering strategy should be used.
  SDValue EmitTargetCodeForMemmove(SelectionDAG &DAG, DebugLoc dl,
                                   SDValue Chain,
                                   SDValue Op1, SDValue Op2,
                                   SDValue Op3, unsigned Align, bool isVolatile,
                                   MachinePointerInfo DstPtrInfo,
                                   MachinePointerInfo SrcPtrInfo) const;

  /// EmitTargetCodeForMemcpy - Emit target-specific code that performs a
  /// memcpy. This can be used by targets to provide code sequences for cases
  /// that don't fit the target's parameters for simple loads/stores and can be
  /// more efficient than using a library call. This function can return a null
  /// SDValue if the target declines to use custom code and a different
  /// lowering strategy should be used.
  ///
  /// If AlwaysInline is true, the size is constant and the target should not
  /// emit any calls and is strongly encouraged to attempt to emit inline code
  /// even if it is beyond the usual threshold because this intrinsic is being
  /// expanded in a place where calls are not feasible (e.g. within the prologue
  /// for another call). If the target chooses to decline an AlwaysInline
  /// request here, legalize will resort to using simple loads and stores.
  SDValue EmitTargetCodeForMemcpy(SelectionDAG &DAG, DebugLoc dl,
                                  SDValue Chain,
                                  SDValue Op1, SDValue Op2,
                                  SDValue Op3, unsigned Align, bool isVolatile,
                                  bool AlwaysInline,
                                  MachinePointerInfo DstPtrInfo,
                                  MachinePointerInfo SrcPtrInfo) const;
};
}

#endif
