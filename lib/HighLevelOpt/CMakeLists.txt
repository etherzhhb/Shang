set(LLVM_NO_RTTI 1)

add_llvm_library(VTMHighLevelOpt
  FunctionFilter.cpp
  )
