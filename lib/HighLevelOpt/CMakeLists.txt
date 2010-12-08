set(LLVM_REQUIRES_RTTI 1)
set(LLVM_REQUIRES_EH 1)

add_llvm_library(VTMHighLevelOpt
  FunctionFilter.cpp
  )
