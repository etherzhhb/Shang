include_directories( ${CMAKE_CURRENT_BINARY_DIR}/.. ${CMAKE_CURRENT_SOURCE_DIR}/.. )

set(LLVM_REQUIRES_RTTI 1)
set(LLVM_REQUIRES_EH 1)

add_llvm_library(LLVMVerilogBackendInfo
  VBackendTargetInfo.cpp
  )

