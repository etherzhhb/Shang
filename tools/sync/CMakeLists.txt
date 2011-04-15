set(LLVM_LINK_COMPONENTS VerilogBackend VerilogBackendInfo bitreader asmparser codegen SelectionDAG)

add_llvm_tool(sync
  sync.cpp
 )
