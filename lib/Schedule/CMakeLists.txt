set(LLVM_NO_RTTI 1)

add_llvm_library(VTMSchedule
  ForceDirectedListScheduling.cpp
  ForceDirectedScheduling.cpp
  HWAtom.cpp
  HWAtomInfo.cpp
)
