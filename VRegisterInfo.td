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

def R : VTMReg<"r">;

class VRegClass<string namespace, list<ValueType> regTypes, int alignment,
                    list<Register> regList>
  : RegisterClass<namespace, regTypes, alignment, regList> {}
  
def DR    : VRegClass<"VTM", [i1, i8, i16, i32, i64], 64, [R]>;
def DR1   : VRegClass<"VTM", [i1], 1, [R]>;
def DR8   : VRegClass<"VTM", [i8], 8, [R]>;
def DR16  : VRegClass<"VTM", [i16], 16, [R]>;
def DR32  : VRegClass<"VTM", [i32], 32, [R]>;
def DR64  : VRegClass<"VTM", [i64], 64, [R]>;
