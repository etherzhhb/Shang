add_llvm_library(VTMRTLCodegen
  RTLCodegenPrepare.cpp
  RTLCodegen.cpp
  FindMBBShortestPath.cpp
)

add_dependencies(VTMRTLCodegen VerilogBackendTable_gen)
