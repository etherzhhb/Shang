add_llvm_library(VTMHighLevelOpt
  FunctionFilter.cpp
  GVPromotion.cpp
  HWPartitionInfo.cpp
  )

llvm_config(VTMHighLevelOpt transformutils ipo)

add_dependencies(VTMHighLevelOpt VerilogBackendTable_gen)
