#Dirty Hack: We need to include some internal llvm header files.
include_directories(${LLVM_MAIN_SRC_DIR}/lib/CodeGen)

add_llvm_library(VTMSchedule
  ForceDirectedListScheduling.cpp
  ForceDirectedScheduling.cpp
  MicroState.cpp
  VSUnit.cpp
  VPreRegAllocSched.cpp
  RegAllocOptimalSSA.cpp
)

add_dependencies(VTMSchedule VerilogBackendTable_gen)
