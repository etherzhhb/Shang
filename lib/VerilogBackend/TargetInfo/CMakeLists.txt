include_directories( ${CMAKE_CURRENT_BINARY_DIR}/.. ${CMAKE_CURRENT_SOURCE_DIR}/.. )

add_vbe_library(LLVMVerilogBackendInfo
  VBackendTargetInfo.cpp
  )

