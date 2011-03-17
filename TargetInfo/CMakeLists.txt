include_directories( ${CMAKE_CURRENT_BINARY_DIR}/.. ${CMAKE_CURRENT_SOURCE_DIR}/.. )

add_llvm_library(LLVMVerilogBackendInfo
  VBackendTargetInfo.cpp
  )
  
add_dependencies(LLVMVerilogBackendInfo VerilogBackendTable_gen)

