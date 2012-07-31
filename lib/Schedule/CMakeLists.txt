#Dirty Hack: We need to include some internal llvm header files.
include_directories(${LLVM_MAIN_SRC_DIR}/lib/CodeGen)

include_directories(
  ${LPSOLVE_INCLUDE_DIR}
)

add_llvm_library(VTMSchedule
  CompGraph.cpp
  ForceDirectedScheduling.cpp
  SchedulingBase.cpp
  VSUnit.cpp
  UnbalanceMuxPrebind.cpp
  AdjustLIForBundles.cpp
  PrebindMuxBase.cpp
  ScheduleEmitter.cpp
  RecurrenceFinder.cpp
  VPreRegAllocSched.cpp
  VRegAllocSimple.cpp
  VAliasAnalysis.cpp
  SDCScheduler.cpp
  DetailLatencyInfo.cpp
)

target_link_libraries (
	VTMSchedule
	${LPSOLVE_LIBRARY}
)

add_dependencies(VTMSchedule VerilogBackendTableGen)
