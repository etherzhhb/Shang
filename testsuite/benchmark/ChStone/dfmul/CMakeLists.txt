file(GLOB test_sources *.cpp)
foreach(test_source ${test_sources})
  get_filename_component(file_name ${test_source} NAME_WE)
  #set(DP_ASAP 		TRUE)		# pipeline_type = DontPipeline, schedule_type = ASAP
  #set(ILPMS_ILP 	TRUE)		# pipeline_type = ILPMS, schedule_tpye = ILP
  #set(IMS_ASAP 		TRUE)		# pipeline_type = IMS, schedule_type = ASAP
  add_test_cases(${file_name}
				 DP_ASAP
				 ILPMS_ILP
				 IMS_ASAP
				 )
endforeach()

add_benchmark_stats("float64_mul")

