set(LLVM_NO_RTTI 1)

add_llvm_target(VTMBitLevelOpt
  BitLevelInfo.cpp
  BitLevelOpt.cpp 
  )
