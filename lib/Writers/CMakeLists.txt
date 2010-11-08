set(LLVM_NO_RTTI 1)

add_llvm_target(VTMWriters
  VLang.cpp
  RTLWriter.cpp
  )
