//===--------- VSelectionDAGInfo.h - VTM SelectionDAG Info ------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
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
};

}

#endif
