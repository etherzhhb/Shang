if( MSVC_IDE OR XCODE )
  # Creates a dummy target containing all headers for the benefit of
  # Visual Studio users.
  file(GLOB_RECURSE headers *.h)
  add_library(vtm_headers_do_not_build EXCLUDE_FROM_ALL
    # We need at least one source file:
    ${LLVM_MAIN_SRC_DIR}/lib/Transforms/Hello/Hello.cpp
    ${headers})
endif()
