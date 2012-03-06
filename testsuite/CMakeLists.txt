cmake_minimum_required(VERSION 2.6)

set(VTS_SOURCE_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(VTS_BINARY_ROOT ${CMAKE_CURRENT_BINARY_DIR})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${VTS_SOURCE_ROOT}/cmake")

set(ENV{VERILATOR_ROOT} ${VERILATOR_ROOT_DIR})

FIND_PACKAGE(Verilator  REQUIRED)
FIND_PACKAGE(SystemC    REQUIRED)

FIND_PROGRAM(FRONTEND clang)
message(STATUS "Using frontend: " ${FRONTEND})
#Set up the llvm binaries.
set(SYNC ${LLVM_TOOLS_BINARY_DIR}/sync)
set(LLC ${LLVM_TOOLS_BINARY_DIR}/llc)
set(LLI ${LLVM_TOOLS_BINARY_DIR}/lli)
set(LLVM_AS ${LLVM_TOOLS_BINARY_DIR}/llvm-as)
set(LLVM_LD ${LLVM_TOOLS_BINARY_DIR}/llvm-ld)


set(SYNTHESIS_DUT OFF CACHE BOOL "Also synthesis the design in testsuite")
if (${SYNTHESIS_DUT} STREQUAL "ON")
FIND_PATH(QUARTUS_BIN_DIR quartus_sh)
endif(${SYNTHESIS_DUT} STREQUAL "ON")

set(FMAX "100.0" CACHE STRING "The maximum frequency of the synthesized designs")
set(ADDSUB_ChainingThreshold "-1" CACHE STRING "The chaining threshold of add/sub function unit (Set to -1 to chains all)")
set(MULT_ChainingThreshold "-1" CACHE STRING "The chaining threshold of mult function unit (Set to -1 to chains all)")
set(SHIFT_ChainingThreshold "-1" CACHE STRING "The chaining threshold of shift function unit (Set to -1 to chains all)")
set(ICMP_ChainingThreshold "-1" CACHE STRING "The chaining threshold of icmp function unit (Set to -1 to chains all)")
set(EXTRA_HLS_OPTION "" CACHE STRING "Extra high level synthesis options should be passed to sync")
set(TIMEOUT "240" CACHE STRING "Maximum time allow the simulation to run")
set(MaxAllowedMuxSize "16" CACHE STRING "Maximum allowed mux size without pipeline register")
set(ENABLE_PHYSICAL_SYNTHESIS "OFF" CACHE BOOL "Enable quartus physical synthesis")

set(ENV{PATH} ${VERILATOR_ROOT_DIR})

if (CMAKE_SIZEOF_VOID_P MATCHES 8)
  set(POINTER_SIZE_IN_BITS 64)
  set(SYSTEMC_LIB_PATH ${SYSTEMC}/lib-linux64)
  set(LLC_MARCH "x86-64")
else (CMAKE_SIZEOF_VOID_P MATCHES 8)
  set(POINTER_SIZE_IN_BITS 32)
  set(SYSTEMC_LIB_PATH ${SYSTEMC}/lib-linux)
  set(LLC_MARCH "x86")
endif(CMAKE_SIZEOF_VOID_P MATCHES 8)

add_custom_target(ir
          COMMENT "Compiling all source to llvm ir")

add_custom_target(main_x86_o
          COMMENT "Compiling software part of all source into obj file")

add_custom_target(verilaror_to_cpp
          COMMENT "Compiling Verilog RTL to cpp")

add_custom_target(CoSimExe
          COMMENT "Compiling all the test obj file into exe file")

add_custom_target(orig_bc
          COMMENT "Compiling llvm ir to bytecode file")

add_custom_target(expect_output
          COMMENT "Compiling all source to llvm lli")

add_custom_target(test_output
          COMMENT "Compiling all source to llvm output")

set(FAILLIST ${VTS_BINARY_ROOT}/faillist)
set(FAILLISTTMP ${VTS_BINARY_ROOT}/faillist-tmp)
set(FIXFAILLIST_SH ${VTS_SOURCE_ROOT}/FixFailList.sh)
set(XFAILLIST ${VTS_SOURCE_ROOT}/ExpectFails)

SET_DIRECTORY_PROPERTIES(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES
                        "${FAILLIST};${FAILLISTTMP};${VTS_BINARY_ROOT}/benchmark.summary.json.tmp;${VTS_BINARY_ROOT}/benchmark.report.json.tmp;${VTS_BINARY_ROOT}/benchmark.summary.json;${VTS_BINARY_ROOT}/benchmark.report.json")

add_custom_target(hls
          #COMMAND [ -f ${FAILLISTTMP} ] || touch ${FAILLISTTMP}
          #COMMAND cat ${FAILLISTTMP} >> ${FAILLIST}
          #COMMAND rm ${FAILLISTTMP}
          #COMMAND [ -s ${FAILLIST} ] && exit 1 || exit 0
          COMMENT "Synthesising RTL module and interface for all source")

add_custom_target(benchmark_test
          #COMMAND [ -f ${FAILLISTTMP} ] || touch ${FAILLISTTMP}
          #COMMAND cat ${FAILLISTTMP} >> ${FAILLIST}
          #COMMAND rm ${FAILLISTTMP}
          #COMMAND [ -s ${FAILLIST} ] && exit 1 || exit 0
          COMMENT "Run test on the benchmarks")

add_custom_target(test_verilogbackend
          COMMAND [ -f ${FAILLISTTMP} ] || touch ${FAILLISTTMP}
          COMMAND cat ${FAILLISTTMP}
          COMMAND mv ${FAILLISTTMP} ${FAILLIST}
          COMMAND [ -s ${FAILLIST} ] && exit 1 || exit 0
          COMMENT "Compiling all source to llvm test_verilogbackend")
            
macro(add_general_test test_file config_file setup_prj_file postfix pipeline_type schedule_type)
  set(TEST_NAME "${test_file}_${pipeline_type}_${schedule_type}")
  set(TEST     "${TEST_NAME}")
  set(DUT_NAME "${TEST_NAME}_DUT")
  set(SYN_FUNC ${test_file})

  set(TEST_SOURCE_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
  set(TEST_BINARY_ROOT ${CMAKE_CURRENT_BINARY_DIR}/${TEST_NAME})
  set(MAIN_OBJ_PRJ_ROOT	"${TEST_BINARY_ROOT}/obj_dir")

  set(TEST_SRC            "${TEST_SOURCE_ROOT}/${test_file}.${postfix}")
  set(MAIN_ORIG_BC        "${TEST_BINARY_ROOT}/${TEST}_main.bc")

  set(MAIN_RTL_ENTITY     "${DUT_NAME}_RTL")
  set(MAIN_RTL_SRC        "${TEST_BINARY_ROOT}/${MAIN_RTL_ENTITY}.v")	
  set(MAIN_SDC_SRC        "${TEST_BINARY_ROOT}/${MAIN_RTL_ENTITY}.sdc")
  set(MAIN_RTL_STA        "${TEST_BINARY_ROOT}/${MAIN_RTL_ENTITY}.sta.rpt")  

  set(MAIN_IF_SRC         "${TEST_BINARY_ROOT}/${DUT_NAME}_IF.${postfix}")
  set(MAIN_SW_LL          "${TEST_BINARY_ROOT}/${DUT_NAME}_SW.ll")
  set(MAIN_X86_SRC        "${TEST_BINARY_ROOT}/${TEST}_main_x86.o")
  set(MAIN_SC_O           "${TEST_BINARY_ROOT}/${DUT_NAME}_IF.o")
  set(VERILATED_O         "${MAIN_OBJ_PRJ_ROOT}/verilated.o")
  set(V_ALL_O             "${MAIN_OBJ_PRJ_ROOT}/V${DUT_NAME}_RTL__ALL*.o")
  set(MAIN_V_MK           "${MAIN_OBJ_PRJ_ROOT}/V${DUT_NAME}_RTL.mk")
  set(MAIN_V__ALL         "${MAIN_OBJ_PRJ_ROOT}/V${DUT_NAME}_RTL__ALL.a")
  set(SC_TB_EXE           "${MAIN_OBJ_PRJ_ROOT}/V${DUT_NAME}_RTL")
  set(CycleCounter        "${TEST_BINARY_ROOT}/${MAIN_RTL_ENTITY}.txt")	

  set(CatchFail           ${FIXFAILLIST_SH} ${XFAILLIST} ${TEST} ${FAILLISTTMP})
 
  set(PIPELINE_TYPE 	  "${pipeline_type}")
  set(SCHEDULE_TYPE 	  "${schedule_type}")
  
  configure_file (
    ${config_file}
    "${TEST_BINARY_ROOT}/${TEST}_config.lua"
    )
	
  add_custom_command(OUTPUT ${MAIN_ORIG_BC}
    COMMAND ${FRONTEND} ${TEST_SRC}
            -O0 -c -emit-llvm
            -o ${MAIN_ORIG_BC}
    COMMAND ${LLVM_LD} -disable-inlining -disable-opt -disable-internalize
            ${MAIN_ORIG_BC}
            ${VTS_SOURCE_ROOT}/liblegup.a
            -b=${MAIN_ORIG_BC}
    DEPENDS ${TEST_SRC} ${LLVM_LD} 
    WORKING_DIRECTORY ${TEST_BINARY_ROOT}
    COMMENT "Compiling ${TEST_SRC} to ${MAIN_ORIG_BC}"
  )
  
  add_custom_target(${TEST}_ir DEPENDS ${MAIN_ORIG_BC})
  add_dependencies(ir ${TEST}_ir)

  add_custom_command(OUTPUT ${TEST_BINARY_ROOT}/Expected.output
    COMMAND ${LLI} ${MAIN_ORIG_BC} > "${TEST_BINARY_ROOT}/Expected.output"
    DEPENDS ${MAIN_ORIG_BC} ${LLI}
    WORKING_DIRECTORY ${TEST_BINARY_ROOT}
    COMMENT "Run bytecode file to get the expect output"
  )
  add_custom_target(${TEST}_lli DEPENDS ${TEST_BINARY_ROOT}/Expected.output)
  add_dependencies(expect_output ${TEST}_lli)

  add_custom_command(OUTPUT ${MAIN_RTL_SRC} ${MAIN_SDC_SRC}
    COMMAND echo "Bad RTL source!" > ${MAIN_RTL_SRC}
    COMMAND echo "Bad Result!" > "${TEST_BINARY_ROOT}/Test.output"
    COMMAND sh -c "${SYNC} -vtm-enable-memscm=false ${TEST_BINARY_ROOT}/${TEST}_config.lua  ${EXTRA_HLS_OPTION}" || ${CatchFail}
    DEPENDS ${MAIN_ORIG_BC} ${SYNC} "${TEST_BINARY_ROOT}/${TEST}_config.lua"
    WORKING_DIRECTORY ${TEST_BINARY_ROOT}
    COMMENT "High-levle Synthesising RTL module and interface for ${SYN_FUNC}"
  )  
  add_custom_target(${TEST}_hls DEPENDS ${MAIN_RTL_SRC})
  add_dependencies(hls ${TEST}_hls)

if(SYNTHESIS_DUT AND TARGET benchmark_report)
  configure_file (
    ${setup_prj_file}
    "${TEST_BINARY_ROOT}/${TEST}_setup_prj.tcl"
    )

  add_custom_command(OUTPUT ${MAIN_RTL_STA}
    COMMAND ${QUARTUS_BIN_DIR}/quartus_sh -t "${TEST_BINARY_ROOT}/${TEST}_setup_prj.tcl"
    #COMMAND ${QUARTUS_BIN_DIR}/quartus_sh --flow compile ${MAIN_RTL_ENTITY}
    #COMMAND ${QUARTUS_BIN_DIR}/quartus_sta ${MAIN_RTL_ENTITY} --do_report_timing
    DEPENDS ${MAIN_RTL_SRC} "${TEST_BINARY_ROOT}/${TEST}_setup_prj.tcl" ${MAIN_SDC_SRC} ${CycleCounter}
    WORKING_DIRECTORY ${TEST_BINARY_ROOT}
    COMMENT "Synthesising RTL module and interface for ${SYN_FUNC}"
  )

  add_custom_target(${TEST}_synthesis DEPENDS ${MAIN_RTL_STA})
  add_dependencies(${TEST}_synthesis ${TEST}_hls)
endif(SYNTHESIS_DUT AND TARGET benchmark_report)

  add_custom_command(OUTPUT ${MAIN_X86_SRC}
    COMMAND ${LLC}
            -march=${LLC_MARCH} "${TEST_BINARY_ROOT}/${DUT_NAME}_SW.ll"
            -filetype=obj
            -mc-relax-all
            -o ${MAIN_X86_SRC}
    DEPENDS ${MAIN_SW_LL} ${LLC}
    WORKING_DIRECTORY ${TEST_BINARY_ROOT}
    COMMENT "Compiling software part of ${TEST_SRC} into obj file" 
  )
  add_custom_target(${TEST}_main_x86_o DEPENDS ${MAIN_X86_SRC})
  add_dependencies(${TEST}_main_x86_o ${TEST}_hls ${MAIN_ORIG_BC})
  add_dependencies(main_x86_o ${TEST}_main_x86_o)

  add_custom_command(OUTPUT ${MAIN_OBJ_PRJ_ROOT}/V${DUT_NAME}_RTL.mk
    COMMAND rm -f "${TEST_BINARY_ROOT}/obj_dir/*"
    COMMAND ${VERILATOR_EXECUTABLE} ${MAIN_RTL_SRC} -Wall --sc -D__DEBUG_IF +define+__VERILATOR_SIM --top-module ${MAIN_RTL_ENTITY} || ${CatchFail}
    MAIN_DEPENDENCY ${MAIN_RTL_SRC}
    WORKING_DIRECTORY ${TEST_BINARY_ROOT}
    COMMENT "Compiling RTL module of ${TEST_SRC} into cpp file" 
  )
  add_custom_target(${TEST}_verilaror_to_cpp DEPENDS ${MAIN_OBJ_PRJ_ROOT}/V${DUT_NAME}_RTL.mk)
  add_dependencies(${TEST}_verilaror_to_cpp ${TEST}_hls)   
  add_dependencies(verilaror_to_cpp ${TEST}_verilaror_to_cpp)

  add_custom_command(OUTPUT ${SC_TB_EXE}
    COMMAND make "${TEST_BINARY_ROOT}/obj_dir/" -j -f "V${DUT_NAME}_RTL.mk" "V${DUT_NAME}_RTL__ALL.a"
    COMMAND make "${TEST_BINARY_ROOT}/obj_dir/" -j -f "V${DUT_NAME}_RTL.mk" ${MAIN_SC_O} "verilated.o"
    COMMAND g++ -L${SYSTEMC_LIB_PATH}
            ${MAIN_SC_O}
            ${MAIN_X86_SRC}
            ${V_ALL_O}
            ${VERILATED_O}
            -o
            ${SC_TB_EXE} -lsystemc
    DEPENDS ${MAIN_X86_SRC} ${MAIN_OBJ_PRJ_ROOT}/V${DUT_NAME}_RTL.mk
    WORKING_DIRECTORY ${MAIN_OBJ_PRJ_ROOT}
    COMMENT "Build SW/HW cosimulation executable" 
  )
  add_custom_target(${TEST}_CoSimExe DEPENDS ${SC_TB_EXE})
  add_dependencies(CoSimExe ${TEST}_CoSimExe)

  add_custom_command(OUTPUT "${TEST_BINARY_ROOT}/Test.output" ${CycleCounter}
    COMMAND timeout ${TIMEOUT}s "${SC_TB_EXE}" > "${TEST_BINARY_ROOT}/Test.output" || ${CatchFail}
    MAIN_DEPENDENCY ${SC_TB_EXE}
    COMMENT "Run programs to get the test output"
  )
  add_custom_target(${TEST}_output DEPENDS "${TEST_BINARY_ROOT}/Test.output" ${CycleCounter})
  add_dependencies(test_output ${TEST}_output)

  add_custom_target(${TEST}_diff_output
    COMMAND [ -f "${TEST_BINARY_ROOT}/Test.output" ] &&  diff "${TEST_BINARY_ROOT}/Expected.output" "${TEST_BINARY_ROOT}/Test.output" || ${CatchFail}
    COMMENT "Comparing program output"
  )
  add_dependencies(${TEST}_diff_output ${TEST}_output ${TEST}_lli)
  add_dependencies(test_verilogbackend ${TEST}_diff_output) 
  
  add_test(${TEST}_test
           diff ${TEST_BINARY_ROOT}/Expected.output ${TEST_BINARY_ROOT}/Test.output)
				
  set_source_files_properties(${TEST_BINARY_ROOT}/obj_dir/ PROPERTIES GENERATED 1)
  set_property(DIRECTORY APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
				${MAIN_RTL_SRC}
                                ${CycleCounter}
				${TEST_BINARY_ROOT}/Expected.output ${TEST_BINARY_ROOT}/Test.output
				${TEST_BINARY_ROOT}/obj_dir/V${DUT_NAME}_RTL ${TEST}_origin)
  	
endmacro(add_general_test)

macro(add_test_cases file_name DP_ASAP ILPMS_ILP IMS_ASAP)
  if (${DP_ASAP})
	add_general_test(${file_name}
					 "${VTS_SOURCE_ROOT}/test_config.lua.in"
                                         "${VTS_SOURCE_ROOT}/setup_proj.tcl.in"
					 "cpp"
				     "DontPipeline"		#pipeline_type
					 "ASAP"				#schedule_type
					 )
  endif(${DP_ASAP})
  if (${ILPMS_ILP})
	add_general_test(${file_name}
				     "${VTS_SOURCE_ROOT}/test_config.lua.in"
                                     "${VTS_SOURCE_ROOT}/setup_proj.tcl.in"
				     "cpp"
				     "ILPMS"			#pipeline_type
				     "ILP"				#schedule_type
				     )
  endif(${ILPMS_ILP})
  if (${IMS_ASAP})
	add_general_test(${file_name}
				     "${VTS_SOURCE_ROOT}/test_config.lua.in"
                                     "${VTS_SOURCE_ROOT}/setup_proj.tcl.in"
				     "cpp"
				     "IMS"				#pipeline_type
				     "ASAP"				#schedule_type
				     )
  endif(${IMS_ASAP})
endmacro(add_test_cases)

enable_testing()
 add_subdirectory(SimpleTest)
 add_subdirectory(Loops)
 add_subdirectory(benchmark)
