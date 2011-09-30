add_llvm_library(VTMIFCodegen
  ClangIfCodegen.cpp
  VLTCodegen.cpp
  PLBCodegen.cpp
)

add_dependencies(VTMIFCodegen VerilogBackendTable_gen)
