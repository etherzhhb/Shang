set(LLVM_LINK_COMPONENTS VerilogBackend bitreader asmparser codegen SelectionDAG transformutils ipo)

set(LLVM_REQUIRES_RTTI 1)
set(LLVM_REQUIRES_EH 1)

include_directories(
  ${BOOST_INCLUDE_DIR}
  ${LUA_INCLUDE_DIR}
  ${LUABIND_INCLUDE_DIR}
  ${LUABIND_INCLUDE_DIR}/luabind
)

add_llvm_tool(sync
  sync.cpp
 )
