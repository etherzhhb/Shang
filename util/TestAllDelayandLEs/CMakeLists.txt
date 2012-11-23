cmake_minimum_required(VERSION 2.6)

set(FU_EST_SOUCE_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(FU_EST_BINARY_ROOT ${CMAKE_CURRENT_BINARY_DIR})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${FU_EST_SOUCE_ROOT}/cmake")

set(bit_width 				        1 8 16 32 64)
set(TIMING_REPORT_JSON_MUX			      "${FU_EST_BINARY_ROOT}/report_timing_MUX.json")
set(TIMING_REPORT_JSON		            "${FU_EST_BINARY_ROOT}/report_timing.json")

set(LE_REPORT_FILE_MUX	"${FU_EST_BINARY_ROOT}/report_LEs_MUX.json")
set(LE_REPORT_FILE	    "${FU_EST_BINARY_ROOT}/report_LEs.json")

get_directory_property(extra_clean_files ADDITIONAL_MAKE_CLEAN_FILES)
set_directory_properties(PROPERTIES
  ADDITIONAL_MAKE_CLEAN_FILES
  "${extra_clean_files};${TIMING_REPORT_JSON_MUX};${TIMING_REPORT_JSON};${LE_REPORT_FILE_MUX};${LE_REPORT_FILE}")
set_source_files_properties("${TIMING_REPORT_JSON_MUX};${TIMING_REPORT_JSON};${LE_REPORT_FILE_MUX};${LE_REPORT_FILE}"
  PROPERTIES GENERATED TRUE)

set(LUA_SCRIPT_MUX	"${FU_EST_BINARY_ROOT}/MUX.lua")
set(LUA_SCRIPT_OTHERS	"${FU_EST_BINARY_ROOT}/Others.lua")

set(MUX_STAT_SCRIPT			      "${FU_EST_SOUCE_ROOT}/Json2LuaScript_MUX.py")
set(OTHER_FU_STAT_SCRIPT			"${FU_EST_SOUCE_ROOT}/Json2LuaScript_Others.py")

add_custom_target(get_mux_verilog_file
				  COMMENT "Get all the Verilog files for MUXs")
add_custom_target(get_mux_data
				  COMMAND python ${statistics_file} 
				  COMMENT "Get the data for MUXs")
add_custom_target(get_others_data
				  COMMAND python ${OTHER_FU_STAT_SCRIPT} -t ${TIMING_REPORT_JSON} -l ${LE_REPORT_FILE} -o ${LUA_SCRIPT_OTHERS}
				  COMMENT "Get the data for FUs except MUXs")
add_custom_target(get_all_data
				  COMMENT "Get all data for all FUs")
				  
macro(test_mux input_num width)
set(DUT_NAME				"mux")
set(INPUT_NUM				${input_num})
set(WIDTH					${width})
set(IsMux					"testMux")
set(TEST_NAMES				"Input_${input_num}_Width_${width}_test")
set(TEST_ROOT				"${FU_EST_BINARY_ROOT}/Input_${input_num}_Width_${width}")
set(altera_setup_prj_file 	"${FU_EST_SOUCE_ROOT}/TestSetPrj.tcl.in")
set(extract_timing_file		"${FU_EST_SOUCE_ROOT}/extract_timing.tcl.in")
set(SDC_FILE				"${FU_EST_SOUCE_ROOT}/test.sdc")

set(TEST_VERILOG 			"${TEST_ROOT}/test.v")
set(PYTHON_FILE				"${TEST_ROOT}/BuildMuxTable.py")
set(SETUP_PRJ_TCL			"${TEST_ROOT}/TestSetPrj.tcl")
set(EXTRACT_TIMING_TCL		"${TEST_ROOT}/extract_timing.tcl")
set(PseudoTarget		"${TEST_ROOT}/${TEST_NAMES}.txt")
set(FIT_FILE				"${TEST_ROOT}/${TEST_NAMES}.fit.rpt")
set(PYTHON_READ_RPT			"${TEST_ROOT}/MuxReadRpt.py")

configure_file(
	"${FU_EST_SOUCE_ROOT}/BuildMuxTable.py.in"
	"${TEST_ROOT}/BuildMuxTable.py"
)
configure_file(
	"${FU_EST_SOUCE_ROOT}/TestSetPrj.tcl.in"
	"${TEST_ROOT}/TestSetPrj.tcl"
)
configure_file(
	"${FU_EST_SOUCE_ROOT}/extract_timing.tcl.in"
	"${TEST_ROOT}/extract_timing.tcl"
)
configure_file(
	"${FU_EST_SOUCE_ROOT}/MuxReadRpt.py.in"
	"${TEST_ROOT}/MuxReadRpt.py"
)
add_custom_command(OUTPUT ${TEST_VERILOG}
	COMMAND python ${PYTHON_FILE}
	WORKING_DIRECTORY ${TEST_ROOT}
	DEPENDS ${PYTHON_FILE}
	COMMENT "Generate verilog file"
	)
add_custom_target(${input_num}_get_verilog_${width} DEPENDS ${TEST_VERILOG})
add_dependencies(get_mux_verilog_file ${input_num}_get_verilog_${width})

add_custom_command(OUTPUT ${PseudoTarget}
	COMMAND quartus_sh -t ${SETUP_PRJ_TCL} > /dev/null
	COMMAND python ${PYTHON_READ_RPT}
	WORKING_DIRECTORY ${TEST_ROOT}
	DEPENDS ${TEST_VERILOG}
	COMMENT "Setup quartus project to extract timing infomation"
	)
add_custom_target(get_data_${TEST_NAMES} DEPENDS ${PseudoTarget})
add_dependencies(get_mux_data get_data_${TEST_NAMES})
add_dependencies(get_all_data get_data_${TEST_NAMES})
endmacro(test_mux)			

foreach(loop_input_num RANGE 2 64 [1])
	foreach(loop_bit_width ${bit_width})
		test_mux(${loop_input_num} 
				 ${loop_bit_width})
	endforeach(loop_bit_width)
endforeach(loop_input_num)

macro(test_others name width)
set(DUT_NAME				${name})
set(WIDTH					${width})
set(TEST_NAMES				"${DUT_NAME}_${width}")
set(IsMux					"${TEST_NAMES}_test")
set(TEST_ROOT				"${FU_EST_BINARY_ROOT}/${TEST_NAMES}")
set(altera_setup_prj_file 	"${FU_EST_SOUCE_ROOT}/TestSetPrj.tcl.in")
set(extract_timing_file		"${FU_EST_SOUCE_ROOT}/extract_timing.tcl.in")
set(SDC_FILE				"${FU_EST_SOUCE_ROOT}/test.sdc")
set(PseudoTarget					"${TEST_ROOT}/${TEST_NAMES}.txt")

set(TEST_VERILOG 			"${TEST_ROOT}/${DUT_NAME}test.v")
set(SETUP_PRJ_TCL			"${TEST_ROOT}/TestSetPrj.tcl")
set(EXTRACT_TIMING_TCL		"${TEST_ROOT}/extract_timing.tcl")

set(PYTHON_READ_RPT				"${TEST_ROOT}/ReadRpt.py")
set(FIT_FILE				"${TEST_ROOT}/${TEST_NAMES}.fit.rpt")

configure_file(
	"${FU_EST_SOUCE_ROOT}/${DUT_NAME}test.v.in"
	"${TEST_ROOT}/${DUT_NAME}test.v"
)
configure_file(
	"${FU_EST_SOUCE_ROOT}/TestSetPrj.tcl.in"
	"${TEST_ROOT}/TestSetPrj.tcl"
)
configure_file(
	"${FU_EST_SOUCE_ROOT}/extract_timing.tcl.in"
	"${TEST_ROOT}/extract_timing.tcl"
)
configure_file(
	"${FU_EST_SOUCE_ROOT}/ReadRpt.py.in"
	"${TEST_ROOT}/ReadRpt.py"
)
add_custom_command(OUTPUT ${PseudoTarget}
	COMMAND quartus_sh -t ${SETUP_PRJ_TCL}
	COMMAND python ${PYTHON_READ_RPT}
	WORKING_DIRECTORY ${TEST_ROOT}
	DEPENDS ${TEST_VERILOG}
	COMMENT "Extract timing infomation and LE usage for ${TEST_NAMES}"
)
add_custom_target(get_data_${TEST_NAMES} DEPENDS ${PseudoTarget}) 
add_dependencies(get_others_data get_data_${TEST_NAMES})
add_dependencies(get_all_data get_data_${TEST_NAMES})
endmacro(test_others)
	
foreach(loop_bit_width ${bit_width})
	test_others("Add" 
				${loop_bit_width})
	test_others("Cmp" 
				${loop_bit_width})
	test_others("Mul" 
				${loop_bit_width})
	test_others("Red" 
				${loop_bit_width})
	test_others("Sel" 
				${loop_bit_width})
	test_others("Shift" 
				${loop_bit_width})
endforeach(loop_bit_width)

