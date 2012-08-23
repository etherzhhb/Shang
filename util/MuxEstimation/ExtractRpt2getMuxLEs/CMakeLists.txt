cmake_minimum_required(VERSION 2.6)

set(VTS_SOURCE_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(VTS_BINARY_ROOT ${CMAKE_CURRENT_BINARY_DIR})

set(REPORT_FILE				"${VTS_BINARY_ROOT}/Tmp.json")
add_custom_target(get_all_LEs
				  COMMAND "Get all the Used LEs")
file(GLOB test_sources *.fit.rpt)

macro(get_LEs file_name)
message("${file_name}")
set(FILE_NAME				${file_name})
set(PYTHON_FILE				"${VTS_BINARY_ROOT}/${file_name}.py")
set(SOURCE_FILE				"${VTS_SOURCE_ROOT}/${file_name}.fit.rpt")
set(SB_FILE					"${VTS_BINARY_ROOT}/${file_name}.txt")

configure_file(
	"${VTS_SOURCE_ROOT}/ReadRpt.py.in"
	"${VTS_BINARY_ROOT}/${file_name}.py"
)

add_custom_command(OUTPUT ${SB_FILE}
	COMMAND python ${PYTHON_FILE}
	WORKING_DIRECTORY ${VTS_BINARY_ROOT}
	DEPENDS ${PYTHON_FILE}
	COMMENT "Generate verilog file"
	)
add_custom_target(${FILE_NAME}_get_LEs DEPENDS ${SB_FILE})
add_dependencies(get_all_LEs ${FILE_NAME}_get_LEs)
endmacro(get_LEs)

foreach(test_source ${test_sources})
  get_filename_component(file_name ${test_source} NAME_WE)
  get_LEs(${file_name})				 
endforeach()

