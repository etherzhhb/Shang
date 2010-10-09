set(LLVM_NO_RTTI 1)

add_vbe_library(LLVMDesignConfig
  ResourceConfig.cpp
  )

include_directories(
  ${BOOST_INCLUDE_DIR}
  ${LUA_INCLUDE_DIR}
  ${LUABIND_INCLUDE_DIR}
  )

target_link_libraries(LLVMDesignConfig ${LUA_LIBRARY} ${LUABIND_LIBRARY})
