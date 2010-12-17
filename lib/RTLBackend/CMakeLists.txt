add_llvm_library(VTMRTLBackend
  FixCopyInstrs.cpp
  VerilogAST.cpp
  RTLWriter.cpp
)

add_dependencies(VTMRTLBackend VerilogBackendTable_gen)
