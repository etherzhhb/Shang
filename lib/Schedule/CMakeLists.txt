set(LLVM_NO_RTTI 1)

#Dirty Hack: We need to include some internal llvm header files.
include_directories(${LLVM_MAIN_SRC_DIR}/lib/CodeGen)

add_llvm_library(VTMSchedule
  ForceDirectedListScheduling.cpp
  ForceDirectedScheduling.cpp
  HWAtom.cpp
  HWAtomInfo.cpp
  RegAllocOptimalSSA.cpp
)
