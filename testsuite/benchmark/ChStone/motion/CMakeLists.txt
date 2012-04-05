file(GLOB test_sources *.cpp)
foreach(test_source ${test_sources})
  get_filename_component(file_name ${test_source} NAME_WE)
  add_test_cases(${file_name})
endforeach()

add_benchmark_stats("motion_vectors")
#add_benchmark_stats("Initialize_Buffer")
add_benchmark_symain_stats("motion_vectors")

