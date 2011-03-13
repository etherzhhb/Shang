#Dirty Hack: We need to include some internal llvm header files.
include_directories(${LLVM_MAIN_SRC_DIR}/lib/CodeGen)

include_directories(
  ${LPSOLVE_INCLUDE_DIR}
)

add_llvm_library(VTMSchedule
  ForceDirectedScheduling.cpp
  ILPScheduling.cpp
  SchedulingBase.cpp
  MicroState.cpp
  VSUnit.cpp
  ScheduleEmitter.cpp
  RecurrenceFinder.cpp
  VPreRegAllocSched.cpp
  RegAllocOptimalSSA.cpp
)

target_link_libraries (
	VTMSchedule
	${LPSOLVE_LIBRARY}
)

add_dependencies(VTMSchedule VerilogBackendTable_gen)
