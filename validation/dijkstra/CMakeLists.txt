find_package(OpenMP)

add_library(
    sssp_dijkstra_validation
    src/core/dijkstra_solver.cpp
    src/core/parallel_dijkstra_solver.cpp
)
add_library(validation::dijkstra ALIAS sssp_dijkstra_validation)

target_include_directories(sssp_dijkstra_validation PUBLIC src/include)
target_compile_features(sssp_dijkstra_validation PUBLIC cxx_std_20)
target_link_libraries(
    sssp_dijkstra_validation
    PUBLIC sssp::sssp
    PRIVATE sssp_project_options sssp_project_warnings
)

if(OpenMP_CXX_FOUND)
    target_link_libraries(sssp_dijkstra_validation PRIVATE OpenMP::OpenMP_CXX)
    target_compile_definitions(sssp_dijkstra_validation PRIVATE VALIDATION_HAS_OPENMP=1)
endif()
