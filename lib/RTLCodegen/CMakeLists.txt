add_llvm_library(VTMRTLCodegen
  RTLCodegenPrepare.cpp
  VerilogASTExprs.cpp
  VerilogASTBuilder.cpp
  VerilogASTWriter.cpp
  CombPathDelayAnalysis.cpp
  RtlSSAAnalysis.cpp
)

add_dependencies(VTMRTLCodegen VerilogBackendTableGen)
