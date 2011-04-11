//===-- PLBCodegen - Interface Generation for Xilinx PLB ------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file hardware interface generation pass for xilinx PLB.
//
//===----------------------------------------------------------------------===//


#include "vtm/Passes.h"
#include "vtm/VerilogAST.h"
#include "vtm/VFuncInfo.h"
#include "vtm/FileInfo.h"
#include "vtm/LangSteam.h"
#include "vtm/Utilities.h"

#include "llvm/Function.h"
#include "llvm/DerivedTypes.h"

#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"

#include "llvm/ADT/StringExtras.h"

#include "llvm/Support/Format.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/ToolOutputFile.h"
#include "llvm/Support/ErrorHandling.h"

#include "llvm/Support/Debug.h"

#include <map>

using namespace llvm;

//===----------------------------------------------------------------------===//
// Verilator interface writer.

