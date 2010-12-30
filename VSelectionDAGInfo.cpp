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

#define DEBUG_TYPE "vtm-selectiondag-info"
#include "VTargetMachine.h"
using namespace llvm;

VSelectionDAGInfo::VSelectionDAGInfo(const VTargetMachine &TM)
  : TargetSelectionDAGInfo(TM) {
}

VSelectionDAGInfo::~VSelectionDAGInfo() {
}
