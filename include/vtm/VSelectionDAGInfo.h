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
};

}

#endif
