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
  VerilogAST.cpp
)
macro(luamodule luasrc)
  source_group("lua sources" FILES ${luasrc}.lua)
  set_source_files_properties(${luasrc}.lua PROPERTIES HEADER_FILE_ONLY ON)
    
  add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${luasrc}.inc.tmp
      COMMAND ${LUA_LUAC} -o ${CMAKE_CURRENT_BINARY_DIR}/${luasrc}.lo ${CMAKE_CURRENT_SOURCE_DIR}/${luasrc}.lua
      COMMAND ${LUA_BIN2C} ${CMAKE_CURRENT_BINARY_DIR}/${luasrc}.lo > ${CMAKE_CURRENT_BINARY_DIR}/${luasrc}.inc.tmp
      COMMENT "Generating ${luasrc}.inc ... "
      DEPENDS ${luasrc}.lua
    )

  add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${luasrc}.inc
    # Only update the real output file if there are any differences.
    # This prevents recompilation of all the files depending on it if there
    # aren't any.
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_BINARY_DIR}/${luasrc}.inc.tmp
        ${CMAKE_CURRENT_BINARY_DIR}/${luasrc}.inc
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${luasrc}.inc.tmp
    COMMENT "Copying ${luasrc}.inc ..."
    )
      
  set_source_files_properties(${CMAKE_CURRENT_BINARY_DIR}/${luasrc}.inc
                              PROPERTIES GENERATED 1)
  set_property(DIRECTORY APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
               ${luasrc}.lo ${luasrc}.inc.tmp ${luasrc}.inc)

  add_custom_target(${luasrc}Lua_gen
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${luasrc}.inc)
endmacro(luamodule) 

luamodule(luapp)

llvm_config(VTMScripting support)

target_link_libraries (
  VTMScripting
  ${LUA_LIBRARY}
  ${LUABIND_LIBRARY}
)

add_dependencies(VTMScripting VerilogBackendTableGen luappLua_gen)
