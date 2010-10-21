set(LLVM_NO_RTTI 1)

add_llvm_target(Verilog
  CompPathBinding.cpp
  ForceDirectedScheduling.cpp
  ForceDirectedListScheduling.cpp
  ForceDirectedScheduler.cpp
  InstLowering.cpp
  LocalLEA.cpp
  ModuloScheduleInfo.cpp
  MemDepAnalysis.cpp
  RegisterAllocation.cpp
  RTLWriter.cpp
  ScalarStreamization.cpp
  TestbenchWriter.cpp
  TopSortBB.cpp
  VLang.cpp
  )

target_link_libraries (LLVMVerilog LLVMAnalysis)
