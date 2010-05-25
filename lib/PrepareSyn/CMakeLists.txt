set(LLVM_NO_RTTI 1)

add_vbe_library(LLVMPreSyn
  instPriority.cpp
  ModuleSchedulerDriver.cpp
  parallelPass.cpp
  reduceWordWidth.cpp
  removeAlloca.cpp
  subscriptDetect.cpp
  subscripts.cpp
  )
