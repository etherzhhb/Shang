#Dirty Hack: We need to include some internal llvm header files.
include_directories(${LLVM_MAIN_SRC_DIR}/lib/CodeGen)

add_llvm_library(VTMSchedule
  ForceDirectedScheduling.cpp
  SchedulingBase.cpp
  MicroState.cpp
  VSUnit.cpp
  ScheduleEmitter.cpp
  RecurrenceFinder.cpp
  VPreRegAllocSched.cpp
  RegAllocOptimalSSA.cpp
)

add_dependencies(VTMSchedule VerilogBackendTable_gen)
