add_llvm_library(VTMIFCodegen
  VLTCodegen.cpp
  PLBCodegen.cpp
)

add_dependencies(VTMIFCodegen VerilogBackendTable_gen)
