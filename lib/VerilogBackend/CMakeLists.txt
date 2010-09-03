set(LLVM_NO_RTTI 1)

add_llvm_target(VerilogBackend
  CompPathBinding.cpp
  ForceDirectedScheduling.cpp
  ForceDirectedListScheduling.cpp
  ForceDirectedScheduler.cpp
  HWAtom.cpp
  HWAtomInfo.cpp
  InstLowering.cpp
  LocalLEA.cpp
  ModuloScheduleInfo.cpp
  MemDepAnalysis.cpp
  RegisterAllocation.cpp
  RTLWriter.cpp
  ScalarStreamization.cpp
  TestbenchWriter.cpp
  TopSortBB.cpp
  VBackend.cpp
  VLang.cpp
  )
  
 target_link_libraries (LLVMVerilogBackend LLVMDesignConfig LLVMVerilogBackendInfo)

add_subdirectory(TargetInfo)
