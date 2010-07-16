set(LLVM_NO_RTTI 1)

add_llvm_target(VerilogBackend
  HWAtom.cpp
  HWAtomInfo.cpp
  CompPathBinding.cpp
  DDG.cpp
  ForceDirectedInfo.cpp
  ForceDirectedScheduler.cpp
  MemDepAnalysis.cpp
  RegisterAllocation.cpp
  RTLWriter.cpp
  TestbenchWriter.cpp
  VBackend.cpp
  VLang.cpp
  )
  
 target_link_libraries (LLVMVerilogBackend LLVMDesignConfig LLVMVerilogBackendInfo)

add_subdirectory(TargetInfo)
