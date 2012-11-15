macro(add_benchmark_stats name)
set(TESTNAME ${name}_${PipelineType}_${ScheduleType})
add_dependencies(benchmark_report ${TESTNAME}_synthesis)
add_dependencies(benchmark_test ${TESTNAME}_diff_output)

get_filename_component(BASENAME ${CMAKE_CURRENT_BINARY_DIR} NAME)

set(Benchmarks "${Benchmarks} ${BASENAME}/${TESTNAME}" PARENT_SCOPE)

endmacro(add_benchmark_stats)

macro(add_benchmark_symain_stats name)
add_test_main_cases(${name})
set(TESTNAME ${name}_${PipelineType}_${ScheduleType})
add_dependencies(benchmark_synthesis_main ${TESTNAME}_synthesis_main)

endmacro(add_benchmark_symain_stats)


add_subdirectory(DSPStone)
add_subdirectory(ChStone)
