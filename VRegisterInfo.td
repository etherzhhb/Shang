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
def MLHR : VTMReg<"mult_lohi">;
def LSRR : VTMReg<"shr">;
def ASRR : VTMReg<"asr">;
def SHLR : VTMReg<"shl">;
def IR : VTMReg<"interface">;
def CR : VTMReg<"callee">;
def BR : VTMReg<"bram">;
def UCMPR : VTMReg<"ucmp">;
def SCMPR : VTMReg<"scmp">;
def MUXR : VTMReg<"mux">;

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
def RMUL    : VRegClass<"VTM", [i1, i8, i16, i32, i64], 64, [MR]>;
def RMULLH  : VRegClass<"VTM", [i1, i8, i16, i32, i64], 64, [MLHR]>;
// Shifts
def RLSR    : VRegClass<"VTM", [i1, i8, i16, i32, i64], 64, [LSRR]>;
def RASR    : VRegClass<"VTM", [i1, i8, i16, i32, i64], 64, [ASRR]>;
def RSHL    : VRegClass<"VTM", [i1, i8, i16, i32, i64], 64, [SHLR]>;

def RINF    : VRegClass<"VTM", [i1, i8, i16, i32, i64], 64, [IR]>;
def RCFN    : VRegClass<"VTM", [i1, i8, i16, i32, i64], 64, [CR]>;
def RBRM    : VRegClass<"VTM", [i1, i8, i16, i32, i64], 64, [BR]>;

// Integer comparison
def RUCMP    : VRegClass<"VTM", [i1, i8, i16, i32, i64], 64, [UCMPR]>;
def RSCMP    : VRegClass<"VTM", [i1, i8, i16, i32, i64], 64, [SCMPR]>;

// Prebinded distributed (in time domain) multiplexer.
def RMUX    : VRegClass<"VTM", [i1, i8, i16, i32, i64], 64, [MUXR]>;
