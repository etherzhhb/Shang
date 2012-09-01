//===- VIntrinsics.td - Defines VTM intrinsics -------------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines all of the VTM-specific intrinsics.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Definitions for all VTM intrinsics.
//

let TargetPrefix = "vtm", isTarget = 1 in {
  // Annotate bram information, including the block ram number,
  // number of elements, element size in bytes, and the initilize global
  // variable address.
  def int_vtm_annotated_bram_info : Intrinsic<[],
                                              [llvm_i32_ty, llvm_i32_ty,
                                               llvm_i32_ty, llvm_anyptr_ty],
                                              [IntrReadWriteArgMem,
                                               NoCapture<3>]>;
}//FIXME: add multi-dimension support
