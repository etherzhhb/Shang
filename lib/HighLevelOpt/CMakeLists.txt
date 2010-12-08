add_llvm_library(VTMHighLevelOpt
  FunctionFilter.cpp
  )

llvm_config(VTMHighLevelOpt transformutils)

add_dependencies(VTMHighLevelOpt VerilogBackendTable_gen)
