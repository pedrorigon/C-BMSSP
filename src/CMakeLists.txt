find_package(OpenMP)

add_library(
    sssp
    core/block_queue.cpp
    core/dimacs_reader.cpp
    core/graph.cpp
    core/parallel_solver.cpp
    core/sequential_solver.cpp
)
add_library(sssp::sssp ALIAS sssp)

target_include_directories(sssp PUBLIC include)
target_compile_features(sssp PUBLIC cxx_std_20)
target_link_libraries(sssp PRIVATE sssp_project_options sssp_project_warnings)

if(OpenMP_CXX_FOUND)
    target_link_libraries(sssp PRIVATE OpenMP::OpenMP_CXX)
    target_compile_definitions(sssp PRIVATE SSSP_HAS_OPENMP=1)
endif()
