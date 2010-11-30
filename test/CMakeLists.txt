set(VTM_TEST_DIRECTORIES
  "Schedule")

set(LLVM_SOURCE_DIR "${LLVM_MAIN_SRC_DIR}")
set(LLVM_BINARY_DIR "${LLVM_BINARY_DIR}")

if( MSVC )
  #Dirty Hack: Tools place under the Debug dir in windows.
  set(LLVM_TOOLS_DIR "${LLVM_TOOLS_BINARY_DIR}/Debug")
else( MSVC )
  set(LLVM_TOOLS_DIR "${LLVM_TOOLS_BINARY_DIR}")
endif( MSVC )

set(LLVM_LIBS_DIR "${LLVM_BINARY_DIR}/lib")
set(VTM_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(VTM_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/..")

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/lit.site.cfg.in
  ${CMAKE_CURRENT_BINARY_DIR}/lit.site.cfg)

include(FindPythonInterp)
if(PYTHONINTERP_FOUND)
  set(VTM_TEST_EXTRA_ARGS)
  if (MSVC OR XCODE)
    set(VTM_TEST_EXTRA_ARGS "--no-progress-bar")
  endif()

  option(VTM_TEST_USE_VG "Run VTM tests under Valgrind" OFF)
  if(VTM_TEST_USE_VG)
    set(VTM_TEST_EXTRA_ARGS ${VTM_TEST_EXTRA_ARGS} "--vg")
  endif ()

  foreach(testdir ${VTM_TEST_DIRECTORIES})
    add_custom_target(vtm-test-${testdir}
      COMMAND ${PYTHON_EXECUTABLE}
                  ${LLVM_SOURCE_DIR}/utils/lit/lit.py
                  --param vtm_site_config=${CMAKE_CURRENT_BINARY_DIR}/lit.site.cfg
                  --param build_config=${CMAKE_CFG_INTDIR}
                  -v ${VTM_TEST_EXTRA_ARGS}
                  ${CMAKE_CURRENT_BINARY_DIR}/${testdir}
                  DEPENDS llc
                  COMMENT "Running VTM regression tests in ${testdir}")
  endforeach()

  add_custom_target(vtm-test
    COMMAND ${PYTHON_EXECUTABLE}
                ${LLVM_SOURCE_DIR}/utils/lit/lit.py
                --param vtm_site_config=${CMAKE_CURRENT_BINARY_DIR}/lit.site.cfg
                --param build_config=${CMAKE_CFG_INTDIR}
                -v ${VTM_TEST_EXTRA_ARGS}
                ${CMAKE_CURRENT_BINARY_DIR}
                DEPENDS llc
                COMMENT "Running VTM regression tests")
endif()
