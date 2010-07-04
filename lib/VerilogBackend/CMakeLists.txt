set(LLVM_NO_RTTI 1)

add_llvm_target(VerilogBackend
  HWAtom.cpp
  HWAtomInfo.cpp
  ASAPScheduler.cpp
  CompPathBinding.cpp
  ForceDirectedScheduler.cpp
  VBackend.cpp
  VLang.cpp
  RegisterAllocation.cpp
  RTLWriter.cpp
  SchedulerBase.cpp
  TestbenchWriter.cpp
  )
  
 target_link_libraries (LLVMVerilogBackend LLVMDesignConfig LLVMVerilogBackendInfo)

add_subdirectory(TargetInfo)
