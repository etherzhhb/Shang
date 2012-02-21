add_custom_target(benchmark_report COMMENT "Report the area and FMax about the synthesized module of benchmarks with Quartus II")

macro(add_benchmark_stats name)
set(TESTNAME ${name}_IMS_ASAP)
add_dependencies(benchmark_report ${TESTNAME}_synthesis)
add_dependencies(benchmark_test ${TESTNAME}_diff_output)

get_filename_component(BASENAME ${CMAKE_CURRENT_BINARY_DIR} NAME)

set(Benchmarks "${Benchmarks} ${BASENAME}/${TESTNAME}" PARENT_SCOPE)

endmacro(add_benchmark_stats)

#Add the banchmark of simple algorithm
set(DP_ASAP 		TRUE)		# pipeline_type = DontPipeline, schedule_type = ASAP
set(ILPMS_ILP 	FALSE)		# pipeline_type = ILPMS, schedule_tpye = ILP
set(IMS_ASAP 		TRUE)		# pipeline_type = IMS, schedule_type = ASAP

add_subdirectory(DSPStone)
add_subdirectory(ChStone)

