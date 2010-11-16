set(LLVM_NO_RTTI 1)

add_llvm_target(VTMSchedule
  ForceDirectedListScheduling.cpp
  ForceDirectedScheduling.cpp
  HWAtom.cpp
  HWAtomInfo.cpp
)
