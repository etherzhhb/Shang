set(LLVM_REQUIRES_RTTI 1)
set(LLVM_REQUIRES_EH 1)

include_directories(
  ${BOOST_INCLUDE_DIR}
  ${LUA_INCLUDE_DIR}
  ${LUABIND_INCLUDE_DIR}
  ${LUABIND_INCLUDE_DIR}/luabind
)

add_llvm_library(VTMTargetCfgs
  FileInfo.cpp
  PartitionInfo.cpp
  FUInfo.cpp
  Scripting.cpp
)

llvm_config(VTMTargetCfgs support)

target_link_libraries (
  VTMTargetCfgs
  ${LUA_LIBRARY}
  ${LUABIND_LIBRARY}
)

add_dependencies(VTMTargetCfgs VerilogBackendTable_gen)
