add_llvm_library(VTMRTLBackend
  VerilogAST.cpp
  RTLWriter.cpp
)

add_dependencies(VTMRTLBackend VerilogBackendTable_gen)
