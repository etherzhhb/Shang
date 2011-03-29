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
def D1 : VTMReg<"dr1">;
def D8 : VTMReg<"dr8">;
def D16 : VTMReg<"dr16">;
def D32 : VTMReg<"dr32">;
def D64 : VTMReg<"dr64">;

class VRegClass<string namespace, list<ValueType> regTypes, int alignment,
                    list<Register> regList>
  : RegisterClass<namespace, regTypes, alignment, regList> {}

def DR    : VRegClass<"VTM", [i32], 32, [R]>;
def DR1   : VRegClass<"VTM", [i1], 1, [D1]>;
def DR8   : VRegClass<"VTM", [i8], 8, [D8]>;
def DR16  : VRegClass<"VTM", [i16], 16, [D16]>;
def DR32  : VRegClass<"VTM", [i32], 32, [D32]>;
def DR64  : VRegClass<"VTM", [i64], 64, [D64]>;
