set(LLVM_REQUIRES_RTTI 1)
set(LLVM_REQUIRES_EH 1)

include_directories(
  ${BOOST_INCLUDE_DIR}
  ${LUA_INCLUDE_DIR}
  ${LUABIND_INCLUDE_DIR}
  ${LUABIND_INCLUDE_DIR}/luabind
)

add_llvm_library(VTMScripting
  FUInfo.cpp
  LuaScript.cpp
)

llvm_config(VTMScripting support)

target_link_libraries (
  VTMScripting
  ${LUA_LIBRARY}
  ${LUABIND_LIBRARY}
)

add_dependencies(VTMScripting VerilogBackendTable_gen)
