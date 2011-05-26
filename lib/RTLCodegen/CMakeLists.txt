add_llvm_library(VTMRTLCodegen
  RTLCodegen.cpp
)

add_dependencies(VTMRTLCodegen VerilogBackendTable_gen)
