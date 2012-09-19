cmake_minimum_required(VERSION 2.6)

set(VTS_SOURCE_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(VTS_BINARY_ROOT ${CMAKE_CURRENT_BINARY_DIR})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${VTS_SOURCE_ROOT}/cmake")

set(bit_width 				1 8 16 32 64)
set(REPORT_JSON_MUX			"${VTS_SOURCE_ROOT}/report_timing_MUX.json")
set(REPORT_JSON		        "${VTS_SOURCE_ROOT}/report_timing.json")
set(SUBMODULE_REPORT_FILE_MUX	"${VTS_SOURCE_ROOT}/report_LEs_MUX.json")
set(SUBMODULE_REPORT_FILE	"${VTS_SOURCE_ROOT}/report_LEs.json")
set(statistics_file			"${VTS_SOURCE_ROOT}/Json2LuaScript_MUX.py")

add_custom_target(get_mux_verilog_file
				  COMMAND "Get all the Verilog files")
add_custom_target(get_mux_datas
				  COMMAND python ${statistics_file}
				  COMMAND "Setup project for mux to get the datas")
add_custom_target(get_others_datas
				  COMMAND "Setup project for others except mux to get the datas")
add_custom_target(get_all_datas
				  COMMAND "Setup project for each file to get the datas")
				  
macro(test_mux input_num width)
set(DUT_NAME				"mux")
set(INPUT_NUM				${input_num})
set(WIDTH					${width})
set(IsMux					"testMux")
set(TEST_NAMES				"Input_${input_num}_Width_${width}_test")
set(TEST_ROOT				"${VTS_BINARY_ROOT}/Input_${input_num}_Width_${width}")
set(altera_setup_prj_file 	"${VTS_SOURCE_ROOT}/TestSetPrj.tcl.in")
set(extract_timing_file		"${VTS_SOURCE_ROOT}/extract_timing.tcl.in")
set(SDC_FILE				"${VTS_SOURCE_ROOT}/test.sdc")

set(TEST_VERILOG 			"${TEST_ROOT}/test.v")
set(PYTHON_FILE				"${TEST_ROOT}/BuildMuxTable.py")
set(SETUP_PRJ_TCL			"${TEST_ROOT}/TestSetPrj.tcl")
set(EXTRACT_TIMING_TCL		"${TEST_ROOT}/extract_timing.tcl")
set(PseudoMuxTarget		"${TEST_ROOT}/${TEST_NAMES}.txt")
set(FIT_FILE				"${TEST_ROOT}/${TEST_NAMES}.fit.rpt")
set(PYTHON_READ_RPT			"${TEST_ROOT}/MuxReadRpt.py")

configure_file(
	"${VTS_SOURCE_ROOT}/BuildMuxTable.py.in"
	"${TEST_ROOT}/BuildMuxTable.py"
)
configure_file(
	"${VTS_SOURCE_ROOT}/TestSetPrj.tcl.in"
	"${TEST_ROOT}/TestSetPrj.tcl"
)
configure_file(
	"${VTS_SOURCE_ROOT}/extract_timing.tcl.in"
	"${TEST_ROOT}/extract_timing.tcl"
)
configure_file(
	"${VTS_SOURCE_ROOT}/MuxReadRpt.py.in"
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

add_custom_command(OUTPUT ${FIT_FILE}
	COMMAND quartus_sh -t ${SETUP_PRJ_TCL} > /dev/null
	WORKING_DIRECTORY ${TEST_ROOT}
	DEPENDS ${TEST_VERILOG}
	COMMENT "Setup quartus project to extract timing infomation"
	)
add_custom_target(${input_num}_setup_prj_${width} DEPENDS ${FIT_FILE})

add_custom_command(OUTPUT ${PseudoMuxTarget}
	COMMAND python ${PYTHON_READ_RPT}
	WORKING_DIRECTORY ${TEST_ROOT}
	DEPENDS ${PYTHON_READ_RPT} ${FIT_FILE}
	COMMENT "Get LEs from fit.rpt"
)
add_custom_target(get_LEs_${TEST_NAMES} DEPENDS ${PseudoMuxTarget})
add_dependencies(get_mux_datas get_LEs_${TEST_NAMES})
add_dependencies(get_all_datas get_LEs_${TEST_NAMES})
endmacro(test_mux)			

foreach(loop_input_num RANGE 2 256 [1])
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
set(TEST_ROOT				"${VTS_BINARY_ROOT}/${TEST_NAMES}")
set(altera_setup_prj_file 	"${VTS_SOURCE_ROOT}/TestSetPrj.tcl.in")
set(extract_timing_file		"${VTS_SOURCE_ROOT}/extract_timing.tcl.in")
set(SDC_FILE				"${VTS_SOURCE_ROOT}/test.sdc")
set(PseudoMuxTarget					"${TEST_ROOT}/${TEST_NAMES}.txt")

set(TEST_VERILOG 			"${TEST_ROOT}/${DUT_NAME}test.v")
set(SETUP_PRJ_TCL			"${TEST_ROOT}/TestSetPrj.tcl")
set(EXTRACT_TIMING_TCL		"${TEST_ROOT}/extract_timing.tcl")

set(PYTHON_READ_RPT				"${TEST_ROOT}/ReadRpt.py")
set(FIT_FILE				"${TEST_ROOT}/${TEST_NAMES}.fit.rpt")

configure_file(
	"${VTS_SOURCE_ROOT}/${DUT_NAME}test.v.in"
	"${TEST_ROOT}/${DUT_NAME}test.v"
)
configure_file(
	"${VTS_SOURCE_ROOT}/TestSetPrj.tcl.in"
	"${TEST_ROOT}/TestSetPrj.tcl"
)
configure_file(
	"${VTS_SOURCE_ROOT}/extract_timing.tcl.in"
	"${TEST_ROOT}/extract_timing.tcl"
)
configure_file(
	"${VTS_SOURCE_ROOT}/ReadRpt.py.in"
	"${TEST_ROOT}/ReadRpt.py"
)
add_custom_command(OUTPUT ${FIT_FILE}
	COMMAND quartus_sh -t ${SETUP_PRJ_TCL}
	WORKING_DIRECTORY ${TEST_ROOT}
	DEPENDS ${TEST_VERILOG}
	COMMENT "Setup quartus project to extract timing infomation"
)
add_custom_target(setup_prj_${TEST_NAMES} DEPENDS ${FIT_FILE})

add_custom_command(OUTPUT ${PseudoMuxTarget}
	COMMAND python ${PYTHON_READ_RPT}
	WORKING_DIRECTORY ${TEST_ROOT}
	DEPENDS ${PYTHON_READ_RPT} ${FIT_FILE}
	COMMENT "Get LEs from fit.rpt"
)
add_custom_target(get_LEs_${TEST_NAMES} DEPENDS ${PseudoMuxTarget}) 
add_dependencies(get_others_datas get_LEs_${TEST_NAMES})
add_dependencies(get_all_datas get_LEs_${TEST_NAMES})
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

