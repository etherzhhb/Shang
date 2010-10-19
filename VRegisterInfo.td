//===- CTVRegisterInfo.td - CTV Register defs ----------*- tblgen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source 
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
//  Declarations that describe the CTV register file
//===----------------------------------------------------------------------===//

class VTMReg<string n> : Register<n> {
  let Namespace = "VTM";
}

//===----------------------------------------------------------------------===//
//  Registers
//===----------------------------------------------------------------------===//

def DR : VTMReg<"dr">;
def PR : VTMReg<"pr">;

def DR1 : RegisterClass<"VTM", [i1], 1, [DR]>;
def DR8 : RegisterClass<"VTM", [i8], 8, [DR]>;
def DR16 : RegisterClass<"VTM", [i16], 16, [DR]>;
def DR32 : RegisterClass<"VTM", [i32], 32, [DR]>;
def DR64 : RegisterClass<"VTM", [i64], 64, [DR]>;

// Pointer is default to i64 type.
def PR64 : RegisterClass<"VTM", [i64], 64, [PR]>;
