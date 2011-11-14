add_llvm_library(VTMHighLevelOpt
  FunctionFilter.cpp
  StackToGlobal.cpp
  AlwaysInlineFunction.cpp
  ContoBrom.cpp
  )

llvm_config(VTMHighLevelOpt transformutils ipo)

add_dependencies(VTMHighLevelOpt VerilogBackendTable_gen)
