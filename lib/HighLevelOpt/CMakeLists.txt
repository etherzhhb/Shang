add_llvm_library(VTMHighLevelOpt
  FunctionFilter.cpp
  StackToGlobal.cpp
  HLSInliner.cpp
  TrivialLoopUnrollPass.cpp
  LoopVectorizePass.cpp
  ContoBrom.cpp
  )

llvm_config(VTMHighLevelOpt transformutils ipo)

add_dependencies(VTMHighLevelOpt VerilogBackendTableGen)
