//=- PrebindMux.cpp- Allocate Multiplexer for Prebound Function units - C++ -=//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Allocate Multiplexer for Prebound Function units.
//
//===----------------------------------------------------------------------===//

#include "llvm/CodeGen/MachineFunctionPass.h"

#define DEBUG_TYPE "vtm-prebind-mux"
#include "llvm/Support/Debug.h"

using namespace llvm;

namespace {
struct PrebindMux : public MachineFunctionPass {
  static char ID;

  PrebindMux() : MachineFunctionPass(ID) {}

  bool runOnMachineFunction(MachineFunction &MF);
};
}

char PrebindMux::ID = 0;

bool PrebindMux::runOnMachineFunction(MachineFunction &MF) {
  return false;
}
