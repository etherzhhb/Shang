set(LLVM_REQUIRES_RTTI 1)
set(LLVM_REQUIRES_EH 1)

#Dirty Hack: We need to include some internal llvm header files.
include_directories(${LLVM_MAIN_SRC_DIR}/lib/CodeGen)

add_llvm_library(VTMSchedule
  ForceDirectedListScheduling.cpp
  ForceDirectedScheduling.cpp
  VSUnit.cpp
  VPreRegAllocSched.cpp
  RegAllocOptimalSSA.cpp
)
