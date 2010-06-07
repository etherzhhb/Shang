set(LLVM_NO_RTTI 1)

add_llvm_target(VerilogBackend
  abstractHWOpcode.cpp
  designScorer.cpp
  GVRegistry.cpp
  instPriority.cpp
  intrinsics.cpp
  listScheduler.cpp
  VBackend.cpp
  VLang.cpp
  RTLWriter.cpp
  TestbenchWriter.cpp
  )
  
 target_link_libraries (LLVMVerilogBackend LLVMConstr LLVMPreSyn LLVMVerilogBackendInfo)

add_subdirectory(TargetInfo)
