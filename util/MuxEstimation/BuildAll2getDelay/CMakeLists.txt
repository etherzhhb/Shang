cmake_minimum_required(VERSION 2.6)

set(VTS_SOURCE_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(VTS_BINARY_ROOT ${CMAKE_CURRENT_BINARY_DIR})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${VTS_SOURCE_ROOT}/cmake")

set(bit_width 				1 8 16 32 64)
set(REPORT_JSON				"${VTS_SOURCE_ROOT}/report.json")

add_custom_target(get_all_verilog_file
				  COMMAND "Get all the Verilog files")
add_custom_target(get_all_datas
				  COMMAND "Setup project for each file to get the datas")
				  
macro(test_mux input_num width)
set(INPUT_NUM				${input_num})
set(WIDTH					${width})
set(TEST_NAMES				"Input_${input_num}_Width_${width}_test")
set(TEST_ROOT				"${VTS_BINARY_ROOT}/Input_${input_num}_Width_${width}")
set(altera_setup_prj_file 	"${VTS_SOURCE_ROOT}/TestSetPrj.tcl.in")
set(extract_timing_file		"${VTS_SOURCE_ROOT}/extract_timing.tcl.in")
set(SDC_FILE				"${VTS_SOURCE_ROOT}/test.sdc")

set(TEST_VERILOG 			"${TEST_ROOT}/test.v")
set(PYTHON_FILE				"${TEST_ROOT}/ToGenMux.py")
set(SETUP_PRJ_TCL			"${TEST_ROOT}/TestSetPrj.tcl")
set(EXTRACT_TIMING_TCL		"${TEST_ROOT}/extract_timing.tcl")

set(STA_FILE				"${TEST_ROOT}/test.sta.rpt")

configure_file(
	"${VTS_SOURCE_ROOT}/ToGenMux.py.in"
	"${TEST_ROOT}/ToGenMux.py"
)
configure_file(
	"${VTS_SOURCE_ROOT}/TestSetPrj.tcl.in"
	"${TEST_ROOT}/TestSetPrj.tcl"
)
configure_file(
	"${VTS_SOURCE_ROOT}/extract_timing.tcl.in"
	"${TEST_ROOT}/extract_timing.tcl"
)
add_custom_command(OUTPUT ${TEST_VERILOG}
	COMMAND python ${PYTHON_FILE}
	WORKING_DIRECTORY ${TEST_ROOT}
	DEPENDS ${PYTHON_FILE}
	COMMENT "Generate verilog file"
	)
add_custom_target(${input_num}_get_verilog_${width} DEPENDS ${TEST_VERILOG})
add_dependencies(get_all_verilog_file ${input_num}_get_verilog_${width})

add_custom_command(OUTPUT ${STA_FILE}
	COMMAND /home/yaweigao/altera/10.1sp1/quartus/bin/quartus_sh -t ${SETUP_PRJ_TCL} > /dev/null
	WORKING_DIRECTORY ${TEST_ROOT}
	DEPENDS ${TEST_VERILOG}
	COMMENT "Setup quartus project to extract timing infomation"
	)
add_custom_target(${input_num}_setup_prj_${width} DEPENDS ${STA_FILE})
add_dependencies(get_all_datas ${input_num}_setup_prj_${width})

# set_property(DIRECTORY APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
			# ${REPORT_JSON})
endmacro(test_mux)			

foreach(loop_input_num RANGE 2 32 [1])
	foreach(loop_bit_width ${bit_width})
		test_mux(${loop_input_num} 
				 ${loop_bit_width})
	endforeach(loop_bit_width)
endforeach(loop_input_num)

