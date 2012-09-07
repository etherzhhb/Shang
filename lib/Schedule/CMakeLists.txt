#Dirty Hack: We need to include some internal llvm header files.
include_directories(${LLVM_MAIN_SRC_DIR}/lib/CodeGen)

include_directories(
  ${LPSOLVE_INCLUDE_DIR}
)

add_llvm_library(VTMSchedule
  AdjustLIForBundles.cpp
  ChainBreakingAnalysis.cpp
  CompGraph.cpp
  DetailLatencyInfo.cpp
  PrebindMuxBase.cpp
  RecurrenceFinder.cpp
  ScheduleEmitter.cpp
  Schedulers.cpp
  SchedulingBase.cpp
  SDCScheduler.cpp
  UnbalanceMuxPrebind.cpp
  VSUnit.cpp
  VPreRegAllocSched.cpp
  VRegAllocSimple.cpp
  VAliasAnalysis.cpp
)

target_link_libraries (
	VTMSchedule
	${LPSOLVE_LIBRARY}
)

add_dependencies(VTMSchedule VerilogBackendTableGen)
