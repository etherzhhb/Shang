file(GLOB test_sources *.cpp)
foreach(test_source ${test_sources})
  get_filename_component(file_name ${test_source} NAME_WE)
  add_test_cases(${file_name}
				 DP_ASAP
				 ILPMS_ILP
				 IMS_ASAP
				 )
endforeach()

add_benchmark_stats("encode")
