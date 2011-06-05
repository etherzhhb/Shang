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
// Result registers of function units.
def AR : VTMReg<"add">;
def MR : VTMReg<"mult">;
def SR : VTMReg<"shift">;

def PhiR : VTMReg<"phi">;
def R : VTMReg<"r">;
def W : VTMReg<"w">;

class VRegClass<string namespace, list<ValueType> regTypes, int alignment,
                    list<Register> regList>
  : RegisterClass<namespace, regTypes, alignment, regList> {}
  
def DR    : VRegClass<"VTM", [i1, i8, i16, i32, i64], 64, [R]>;
def PredR : VRegClass<"VTM", [i1], 64, [R]>; // Register for predicate
def Wire  : VRegClass<"VTM", [i64], 64, [W]>;
// Symbol for phi join.
def PHIR    : VRegClass<"VTM", [i1, i8, i16, i32, i64], 64, [PhiR]>;

// Result register classes of function units.
def RADD    : VRegClass<"VTM", [i1, i8, i16, i32, i64], 64, [AR]>;
def RMUL    : VRegClass<"VTM", [i1, i8, i16, i32, i64], 64, [MR]>;
def RSHT    : VRegClass<"VTM", [i1, i8, i16, i32, i64], 64, [SR]>;
