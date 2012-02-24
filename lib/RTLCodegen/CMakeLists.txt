add_llvm_library(VTMRTLCodegen
  RTLCodegenPrepare.cpp
  VerilogASTBuilder.cpp
  CFGShortestPath.cpp
  VerilogASTWriter.cpp
  CombPathDelayAnalysis.cpp
  RtlSSAAnalysis.cpp
)

add_dependencies(VTMRTLCodegen VerilogBackendTableGen)
