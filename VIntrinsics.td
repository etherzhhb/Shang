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
  // Return the base address of the allocated block ram.
  // Arguments: NumElement, ElemSizeInBytes
  def int_vtm_alloca_bram  : Intrinsic<[llvm_anyptr_ty],
                                       [llvm_i32_ty, llvm_i32_ty],
                                       [IntrReadMem]>;
  
  // Read or Write the block ram.
  // Return the read value.
  // Arguments: Address to Read/Write, the value to write, isStore, align,
  // isVolatile
  def int_vtm_access_bram : Intrinsic<[llvm_anyint_ty],
                                      [llvm_anyptr_ty,
                                       LLVMMatchType<0>, llvm_i1_ty,
                                       llvm_i32_ty, llvm_i1_ty],
                                      [IntrReadWriteArgMem, NoCapture<0>]>;
}
