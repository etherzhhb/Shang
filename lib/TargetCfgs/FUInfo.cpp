//===----- VFunctionUnit.cpp - VTM Function Unit Information ----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains implementation of the function unit class in
// Verilog target machine.
//
//===----------------------------------------------------------------------===//

#include "vtm/FUInfo.h"

#include "llvm/ADT/STLExtras.h"
#include "llvm/Support/Debug.h"

using namespace llvm;

//===----------------------------------------------------------------------===//
/// Hardware resource.
void VFUDesc::print(raw_ostream &OS) const {
  // OS << "Resource: " << Name << '\n';
  OS.indent(2) << "TotalNum: " << TotalRes << '\n';
  OS.indent(2) << "Latency: " << Latency << '\n';
  OS.indent(2) << "StartInterval: " << StartInt << '\n';
}

namespace llvm {
  namespace VFUs {
   const char *VFUNames[] = {
      "Trivial", "MemoryBus", "Shift", "AddSub", "Mult", "FSMFinish"
    };
  }
}

FUInfo::FUInfo() {
  for (size_t i = 0, e = array_lengthof(ResSet); i != e; ++i)
    ResSet[i] = 0;
}

FUInfo::~FUInfo() {
  for (iterator I = begin(), E = end(); I != E; ++I)
    if(*I) delete *I;
}

unsigned FuncUnitId::getTotalFUs() const {
  // If the function unit is binded, there is only one function unit with
  // the specific function unit id available.
  if (isBound()) return 1;

  // Else we can just choose a function unit from all available function units.
  return vtmfus().getFUDesc(getFUType())->getTotalRes();
}

void FuncUnitId::print(raw_ostream &OS) const {
  OS << VFUs::VFUNames[getFUType()];
  // Print the function unit id if necessary.
  if (isBound()) OS << " Bound to " << getFUNum();
}

void FuncUnitId::dump() const {
  print(dbgs());
}
