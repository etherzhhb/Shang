add_llvm_library(VTMRTLCodegen
  RTLCodegenPrepare.cpp
  VerilogASTBuilder.cpp
  FindMBBShortestPath.cpp
  VerilogASTWriter.cpp
  CombPathDelayAnalysis.cpp
  RegDependencyAnalysis.cpp
)

add_dependencies(VTMRTLCodegen VerilogBackendTableGen)
