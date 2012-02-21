#Add the banchmark of simple algorithm
  set(DP_ASAP 		TRUE)		# pipeline_type = DontPipeline, schedule_type = ASAP
  set(ILPMS_ILP 	FALSE)		# pipeline_type = ILPMS, schedule_tpye = ILP
  set(IMS_ASAP 		TRUE)		# pipeline_type = IMS, schedule_type = ASAP

add_subdirectory(complex_multiply)
add_subdirectory(complex_update)
add_subdirectory(convolution)
add_subdirectory(dot_product)

add_subdirectory(fft)
add_subdirectory(fir)
add_subdirectory(fir2dim)
add_subdirectory(iir_biquad_N_sections)

add_subdirectory(iir_biquad_one_section)
add_subdirectory(lms)
add_subdirectory(matrix)
add_subdirectory(matrix1x3)

add_subdirectory(n_complex_updates)
add_subdirectory(n_real_updates)
add_subdirectory(real_update)
add_subdirectory(startup)