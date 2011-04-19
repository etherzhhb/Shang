add_llvm_library(VTMRTLCodegen
  FixCopyInstrs.cpp
  RTLCodegen.cpp
)

add_dependencies(VTMRTLCodegen VerilogBackendTable_gen)
