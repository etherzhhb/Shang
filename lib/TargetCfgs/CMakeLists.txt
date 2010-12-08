include_directories(
  ${BOOST_INCLUDE_DIR}
  ${LUA_INCLUDE_DIR}
  ${LUABIND_INCLUDE_DIR}
)

add_llvm_library(VTMTargetCfgs
  FileInfo.cpp
  PartitionInfo.cpp
  FUInfo.cpp
  Scripting.cpp
)
