set(LLVM_NO_RTTI 1)

add_llvm_target(VerilogBackend
  HWAtom.cpp
  VBackend.cpp
  VLang.cpp
  RTLWriter.cpp
  TestbenchWriter.cpp
  )
  
 target_link_libraries (LLVMVerilogBackend LLVMDesignConfig LLVMVerilogBackendInfo)

add_subdirectory(TargetInfo)
