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
def CarryR : VTMReg<"carry">;
def MR : VTMReg<"mult">;
def LSRR : VTMReg<"shr">;
def ASRR : VTMReg<"asr">;
def SHLR : VTMReg<"shl">;
def IR : VTMReg<"interface">;
def CR : VTMReg<"callee">;
def BR : VTMReg<"bram">;
def CMPR : VTMReg<"icmp">;

def PhiR : VTMReg<"phi">;
def R : VTMReg<"r">;
def W : VTMReg<"w">;

class VRegClass<string namespace, list<ValueType> regTypes, int alignment,
                    list<Register> regList>
  : RegisterClass<namespace, regTypes, alignment, regList> {}

def DR    : VRegClass<"VTM", [i1, i8, i16, i32, i64], 64, [R]>;
def Wire  : VRegClass<"VTM", [i64], 64, [W]>;
// Symbol for phi join.
def PHIR    : VRegClass<"VTM", [i1, i8, i16, i32, i64], 64, [PhiR]>;

// Result register classes of function units.
def RADD    : VRegClass<"VTM", [i1, i8, i16, i32, i64], 64, [AR]>;
// Carry bit of the adder
def RCAR    : VRegClass<"VTM", [i1], 64, [CarryR]>;
def RMUL    : VRegClass<"VTM", [i1, i8, i16, i32, i64], 64, [MR]>;
// Shifts
def RLSR    : VRegClass<"VTM", [i1, i8, i16, i32, i64], 64, [LSRR]>;
def RASR    : VRegClass<"VTM", [i1, i8, i16, i32, i64], 64, [ASRR]>;
def RSHL    : VRegClass<"VTM", [i1, i8, i16, i32, i64], 64, [SHLR]>;

def RINF    : VRegClass<"VTM", [i1, i8, i16, i32, i64], 64, [IR]>;
def RCFN    : VRegClass<"VTM", [i1, i8, i16, i32, i64], 64, [CR]>;
def RBRM    : VRegClass<"VTM", [i1, i8, i16, i32, i64], 64, [BR]>;

// Integer comparison
def RCMP    : VRegClass<"VTM", [i1, i8, i16, i32, i64], 64, [CMPR]>;
