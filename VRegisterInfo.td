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
def PIPER : VTMReg<"pipe">;

def R : VTMReg<"r">;
def W : VTMReg<"w">;

def DR    : RegisterClass<"VTM", [i1, i8, i16, i32, i64], 64, (add R)>;
def Wire  : RegisterClass<"VTM", [i64], 64, (add W)>;

// Result register classes of function units.
def RADD    : RegisterClass<"VTM", [i1, i8, i16, i32, i64], 64, (add AR)>;
def RMUL    : RegisterClass<"VTM", [i1, i8, i16, i32, i64], 64, (add MR)>;
def RMULLH  : RegisterClass<"VTM", [i1, i8, i16, i32, i64], 64, (add MLHR)>;
// Shifts
def RLSR    : RegisterClass<"VTM", [i1, i8, i16, i32, i64], 64, (add LSRR)>;
def RASR    : RegisterClass<"VTM", [i1, i8, i16, i32, i64], 64, (add ASRR)>;
def RSHL    : RegisterClass<"VTM", [i1, i8, i16, i32, i64], 64, (add SHLR)>;

def RINF    : RegisterClass<"VTM", [i1, i8, i16, i32, i64], 64, (add IR)>;
def RCFN    : RegisterClass<"VTM", [i1, i8, i16, i32, i64], 64, (add CR)>;
def RBRM    : RegisterClass<"VTM", [i1, i8, i16, i32, i64], 64, (add BR)>;

// Integer comparison
def RUCMP    : RegisterClass<"VTM", [i1, i8, i16, i32, i64], 64, (add UCMPR)>;
def RSCMP    : RegisterClass<"VTM", [i1, i8, i16, i32, i64], 64, (add SCMPR)>;

// Prebinded distributed (in time domain) multiplexer.
def RMUX    : RegisterClass<"VTM", [i1, i8, i16, i32, i64], 64, (add MUXR)>;

// Pseudo wire for pipeline stage.
def RPIPE    : RegisterClass<"VTM", [i1, i8, i16, i32, i64], 64, (add PIPER)>;
