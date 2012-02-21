##################################
#Add the banchmark of simple algorithm
  set(DP_ASAP 		TRUE)		# pipeline_type = DontPipeline, schedule_type = ASAP
  set(ILPMS_ILP 	FALSE)		# pipeline_type = ILPMS, schedule_tpye = ILP
  set(IMS_ASAP 		TRUE)		# pipeline_type = IMS, schedule_type = ASAP

add_subdirectory(dfadd)
add_subdirectory(dfmul)
add_subdirectory(dfdiv)
add_subdirectory(dfsin)

##################################
#Add the benchmark of encryption 

add_subdirectory(sha)
add_subdirectory(aes)
add_subdirectory(adpcm)
add_subdirectory(blowfish)

##################################
#Add the benchmark of other categories

add_subdirectory(gsm)
add_subdirectory(mips)
add_subdirectory(jpeg)
add_subdirectory(motion)

#message(STATUS "Chstore Benchmarks : " ${Benchmarks})

add_custom_target(chstone_benchmark_report
    COMMAND echo '[' > ${VTS_BINARY_ROOT}/benchmark.summary.json
    COMMAND sed 1d ${VTS_BINARY_ROOT}/benchmark.summary.json.tmp >> ${VTS_BINARY_ROOT}/benchmark.summary.json
    COMMAND echo ']' >> ${VTS_BINARY_ROOT}/benchmark.summary.json
    COMMAND rm -f ${VTS_BINARY_ROOT}/benchmark.summary.json.tmp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Report the area and FMax about the synthesized module of benchmarks with Quartus II")
add_dependencies(chstone_benchmark_report benchmark_report)

