add_llvm_library(VTMRTLCodegen
  RTLCodegenPrepare.cpp
  VASTExprBuilder.cpp
  VerilogASTBuilder.cpp
  VerilogASTWriter.cpp
  MachineFunction2Datapath.cpp
  ControlLogicBuilder.cpp
  CombPathDelayAnalysis.cpp
  RtlSSAAnalysis.cpp
)

add_dependencies(VTMRTLCodegen VerilogBackendTableGen)
