add_llvm_library(VTMHighLevelOpt
  FunctionFilter.cpp
  )

add_dependencies(VTMHighLevelOpt VerilogBackendTable_gen)
