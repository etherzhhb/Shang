add_llvm_library(VTMIFBackend
  VLTBackend.cpp
)

add_dependencies(VTMIFBackend VerilogBackendTable_gen)
