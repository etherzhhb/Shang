set(LLVM_NO_RTTI 1)

add_llvm_library(VTMConfigs
  FileInfo.cpp
  PartitionInfo.cpp
  ResourceInfo.cpp
)
