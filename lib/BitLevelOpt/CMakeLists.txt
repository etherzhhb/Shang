add_llvm_library(VTMBitLevelOpt
  BitLevelInfo.cpp
  BitLevelOpt.cpp 
  )

add_dependencies(VTMBitLevelOpt VerilogBackendTable_gen)
