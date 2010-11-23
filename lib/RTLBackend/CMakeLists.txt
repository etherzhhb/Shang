set(LLVM_NO_RTTI 1)

add_llvm_library(VTMRTLBackend
  VerilogAST.cpp
  RTLWriter.cpp
)
