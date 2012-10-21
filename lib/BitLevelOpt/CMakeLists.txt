if (ENABLE_LOGIC_SYNTHESIS)
  set(LogicSynthesis LogicSynthesis/LogicSynthesis.cpp)
endif (ENABLE_LOGIC_SYNTHESIS)

add_llvm_library(VTMBitLevelOpt
  BitLevelInfo.cpp
  BitLevelOpt.cpp 
  ${LogicSynthesis}
  )

add_dependencies(VTMBitLevelOpt VerilogBackendTableGen)

if (ENABLE_LOGIC_SYNTHESIS)
include_directories(
  ${ABC_INCLUDE_DIR}/src
  ${ABC_INCLUDE_DIR}/src/aig/aig
  ${ABC_INCLUDE_DIR}/src/aig/saig
  ${ABC_INCLUDE_DIR}/src/aig/gia
  ${ABC_INCLUDE_DIR}/src/aig/hop
  ${ABC_INCLUDE_DIR}/src/aig/mem
  ${ABC_INCLUDE_DIR}/src/aig/tim
  ${ABC_INCLUDE_DIR}/src/aig/ssw
  ${ABC_INCLUDE_DIR}/src/aig/fra
  ${ABC_INCLUDE_DIR}/src/aig/nwk
  ${ABC_INCLUDE_DIR}/src/aig/ntl
  ${ABC_INCLUDE_DIR}/src/aig/ioa
  ${ABC_INCLUDE_DIR}/src/aig/dar
  ${ABC_INCLUDE_DIR}/src/map/if
  ${ABC_INCLUDE_DIR}/src/misc/vec
  ${ABC_INCLUDE_DIR}/src/misc/util
  ${ABC_INCLUDE_DIR}/src/misc/avl
  ${ABC_INCLUDE_DIR}/src/misc/bzlib
  ${ABC_INCLUDE_DIR}/src/misc/espresso
  ${ABC_INCLUDE_DIR}/src/misc/ext
  ${ABC_INCLUDE_DIR}/src/misc/extra
  ${ABC_INCLUDE_DIR}/src/misc/hash
  ${ABC_INCLUDE_DIR}/src/misc/mvc
  ${ABC_INCLUDE_DIR}/src/misc/nm
  ${ABC_INCLUDE_DIR}/src/misc/st
  ${ABC_INCLUDE_DIR}/src/misc/zlib
  ${ABC_INCLUDE_DIR}/src/base/main
  ${ABC_INCLUDE_DIR}/src/base/abc
  ${ABC_INCLUDE_DIR}/src/base/cmd
  ${ABC_INCLUDE_DIR}/src/base/io
  ${ABC_INCLUDE_DIR}/src/bdd/cudd
  ${ABC_INCLUDE_DIR}/src/bdd/mtr
  ${ABC_INCLUDE_DIR}/src/bdd/epd
  ${ABC_INCLUDE_DIR}/src/sat/bsat
  )

target_link_libraries (
	VTMBitLevelOpt
	${ABC_LIBRARY}
)

if (${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  set(ABC_PLATFORM LIN)
  # libreadline is need under linux
  target_link_libraries (VTMBitLevelOpt readline)

  # Set the platform flag.
  if (CMAKE_SIZEOF_VOID_P MATCHES 8)
   set(ABC_PLATFORM ${ABC_PLATFORM}64)
  endif (CMAKE_SIZEOF_VOID_P MATCHES 8)

  message(STATUS "Platform variable for ABC is ${ABC_PLATFORM}")
  add_definitions(-D${ABC_PLATFORM})
endif(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")



endif (ENABLE_LOGIC_SYNTHESIS)
