set(LLVM_NO_RTTI 1)

add_llvm_target(VTMRTLBacnend
  VerilogAST.cpp
  RTLWriter.cpp
)
