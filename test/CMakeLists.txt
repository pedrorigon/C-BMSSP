function(add_sssp_test name)
    add_executable(${name} ${ARGN})
    target_include_directories(${name} PRIVATE support)
    target_link_libraries(
        ${name}
        PRIVATE sssp::sssp sssp_project_options sssp_project_warnings
    )
    add_test(NAME ${name} COMMAND ${name})
endfunction()

add_library(sssp_test_support STATIC support/graph_factory.cpp)
target_include_directories(sssp_test_support PUBLIC support)
target_link_libraries(
    sssp_test_support
    PUBLIC sssp::sssp
    PRIVATE sssp_project_options sssp_project_warnings
)

add_sssp_test(graph_unit_test unit/graph_test.cpp)
add_sssp_test(dimacs_reader_unit_test unit/dimacs_reader_test.cpp)
target_compile_definitions(
    dimacs_reader_unit_test
    PRIVATE SSSP_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data"
)
add_sssp_test(snap_reader_unit_test unit/snap_reader_test.cpp)
target_compile_definitions(
    snap_reader_unit_test
    PRIVATE SSSP_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data"
)
add_sssp_test(block_queue_unit_test unit/block_queue_test.cpp)
add_sssp_test(constant_degree_graph_unit_test unit/constant_degree_graph_test.cpp)
add_sssp_test(sequential_solver_unit_test unit/sequential_solver_test.cpp)
add_sssp_test(parallel_solver_unit_test unit/parallel_solver_test.cpp)
add_sssp_test(solver_edge_cases_unit_test unit/solver_edge_cases_test.cpp)
target_link_libraries(solver_edge_cases_unit_test PRIVATE validation::dijkstra)
add_sssp_test(solver_internal_unit_test unit/solver_internal_test.cpp)
add_sssp_test(path_result_unit_test unit/path_result_test.cpp)
add_sssp_test(dijkstra_validation_unit_test unit/dijkstra_solver_test.cpp)
target_link_libraries(dijkstra_validation_unit_test PRIVATE validation::dijkstra)
add_sssp_test(parallel_dijkstra_validation_unit_test unit/parallel_dijkstra_solver_test.cpp)
target_link_libraries(
    parallel_dijkstra_validation_unit_test
    PRIVATE sssp_test_support validation::dijkstra
)

add_sssp_test(generated_graph_comparison_test comparison/generated_graphs_test.cpp)
target_link_libraries(generated_graph_comparison_test PRIVATE sssp_test_support validation::dijkstra)

add_sssp_test(rome_graph_comparison_test comparison/rome_graph_test.cpp)
target_link_libraries(rome_graph_comparison_test PRIVATE validation::dijkstra)
target_compile_definitions(
    rome_graph_comparison_test
    PRIVATE SSSP_ROME99_PATH="${CMAKE_CURRENT_SOURCE_DIR}/data/Rome99"
)

add_sssp_test(large_graph_comparison_test comparison/large_graph_test.cpp)
target_link_libraries(large_graph_comparison_test PRIVATE sssp_test_support validation::dijkstra)

add_executable(sssp_compare comparison/compare_cli.cpp)
target_link_libraries(
    sssp_compare
    PRIVATE sssp::sssp validation::dijkstra sssp_project_options sssp_project_warnings
)

add_executable(sssp_batch_results comparison/batch_results_cli.cpp)
target_link_libraries(
    sssp_batch_results
    PRIVATE sssp::sssp validation::dijkstra sssp_project_options sssp_project_warnings
)

add_executable(sssp_benchmark comparison/benchmark_cli.cpp)
target_link_libraries(
    sssp_benchmark
    PRIVATE sssp::sssp validation::dijkstra sssp_project_options sssp_project_warnings
)

add_executable(sssp_scaling comparison/scaling_cli.cpp)
target_link_libraries(
    sssp_scaling
    PRIVATE sssp::sssp sssp_test_support validation::dijkstra
            sssp_project_options sssp_project_warnings
)

add_executable(sssp_visualization_trace comparison/visualization_cli.cpp)
target_link_libraries(
    sssp_visualization_trace
    PRIVATE sssp::sssp validation::dijkstra
            sssp_project_options sssp_project_warnings
)

add_executable(sssp_profile comparison/profile_cli.cpp)
target_link_libraries(
    sssp_profile
    PRIVATE sssp::sssp sssp_test_support validation::dijkstra
            sssp_project_options sssp_project_warnings
)

set_tests_properties(
    graph_unit_test
    dimacs_reader_unit_test
    snap_reader_unit_test
    block_queue_unit_test
    constant_degree_graph_unit_test
    sequential_solver_unit_test
    parallel_solver_unit_test
    solver_edge_cases_unit_test
    solver_internal_unit_test
    path_result_unit_test
    dijkstra_validation_unit_test
    parallel_dijkstra_validation_unit_test
    PROPERTIES LABELS "unit"
)
set_tests_properties(
    generated_graph_comparison_test
    rome_graph_comparison_test
    large_graph_comparison_test
    PROPERTIES LABELS "comparison"
)
