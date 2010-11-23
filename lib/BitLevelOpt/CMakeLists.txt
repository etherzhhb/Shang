set(LLVM_NO_RTTI 1)

add_llvm_library(VTMBitLevelOpt
  BitLevelInfo.cpp
  BitLevelOpt.cpp 
  )
