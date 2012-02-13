if (VBE_ENABLE_ABC)
  set(BitLevelABCOpt LogicSynthesis/BitLevelABCOpt.cpp)
endif (VBE_ENABLE_ABC)

add_llvm_library(VTMBitLevelOpt
  BitLevelInfo.cpp
  BitLevelOpt.cpp 
  ${BitLevelABCOpt}
  )

add_dependencies(VTMBitLevelOpt VerilogBackendTable_gen)

if (VBE_ENABLE_ABC)
include_directories(
  ${ABC_INCLUDE_DIR}
  ${ABC_INCLUDE_DIR}/aig/aig
  ${ABC_INCLUDE_DIR}/aig/saig
  ${ABC_INCLUDE_DIR}/aig/gia
  ${ABC_INCLUDE_DIR}/aig/hop
  ${ABC_INCLUDE_DIR}/aig/mem
  ${ABC_INCLUDE_DIR}/aig/tim
  ${ABC_INCLUDE_DIR}/aig/ssw
  ${ABC_INCLUDE_DIR}/aig/fra
  ${ABC_INCLUDE_DIR}/aig/nwk
  ${ABC_INCLUDE_DIR}/aig/ntl
  ${ABC_INCLUDE_DIR}/aig/ioa
  ${ABC_INCLUDE_DIR}/aig/dar
  ${ABC_INCLUDE_DIR}/map/if
  ${ABC_INCLUDE_DIR}/misc/vec
  ${ABC_INCLUDE_DIR}/misc/util
  ${ABC_INCLUDE_DIR}/misc/avl
  ${ABC_INCLUDE_DIR}/misc/bzlib
  ${ABC_INCLUDE_DIR}/misc/espresso
  ${ABC_INCLUDE_DIR}/misc/ext
  ${ABC_INCLUDE_DIR}/misc/extra
  ${ABC_INCLUDE_DIR}/misc/hash
  ${ABC_INCLUDE_DIR}/misc/mvc
  ${ABC_INCLUDE_DIR}/misc/nm
  ${ABC_INCLUDE_DIR}/misc/st
  ${ABC_INCLUDE_DIR}/misc/zlib
  ${ABC_INCLUDE_DIR}/base/main
  ${ABC_INCLUDE_DIR}/base/abc
  ${ABC_INCLUDE_DIR}/base/cmd
  ${ABC_INCLUDE_DIR}/base/io
  ${ABC_INCLUDE_DIR}/bdd/cudd
  ${ABC_INCLUDE_DIR}/bdd/mtr
  ${ABC_INCLUDE_DIR}/bdd/epd
  ${ABC_INCLUDE_DIR}/sat/bsat
  )

target_link_libraries (
	VTMBitLevelOpt
	${ABC_LIBRARY}
)

if (${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  set(ABC_PLATFORM LIN)
elseif(${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
  set(ABC_PLATFORM NT)
else(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
message(ERROR " Unsupported platform to intergate ABC!")
endif(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")

if (CMAKE_SIZEOF_VOID_P MATCHES 8)
 set(ABC_PLATFORM ${ABC_PLATFORM}64)
endif (CMAKE_SIZEOF_VOID_P MATCHES 8)

add_definitions(-D${ABC_PLATFORM})

endif (VBE_ENABLE_ABC)
