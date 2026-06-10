cmake_minimum_required(VERSION 3.20)

project(sssp VERSION 0.2.0 LANGUAGES CXX)

option(SSSP_BUILD_TESTS "Build unit and comparison tests" ON)
option(SSSP_BUILD_VALIDATION "Build validation-only implementations and tools" ${SSSP_BUILD_TESTS})
option(SSSP_ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(SSSP_ENABLE_TEST_COVERAGE "Enable coverage instrumentation for tests" OFF)
option(SSSP_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" ON)

add_library(sssp_project_options INTERFACE)
target_compile_features(sssp_project_options INTERFACE cxx_std_20)

add_library(sssp_project_warnings INTERFACE)
target_compile_options(
    sssp_project_warnings
    INTERFACE
        -Wall
        -Wextra
        -Wpedantic
        -Wconversion
        -Wshadow
        $<$<BOOL:${SSSP_WARNINGS_AS_ERRORS}>:-Werror>
)

if(SSSP_ENABLE_SANITIZERS)
    target_compile_options(sssp_project_options INTERFACE -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(sssp_project_options INTERFACE -fsanitize=address,undefined -fno-omit-frame-pointer)
endif()

if(SSSP_ENABLE_TEST_COVERAGE)
    target_compile_options(sssp_project_options INTERFACE --coverage -O0 -g)
    target_link_options(sssp_project_options INTERFACE --coverage)
endif()

add_subdirectory(src)

if(SSSP_BUILD_VALIDATION)
    add_subdirectory(validation/dijkstra)
endif()

if(SSSP_BUILD_TESTS)
    if(NOT SSSP_BUILD_VALIDATION)
        message(FATAL_ERROR "SSSP_BUILD_TESTS requires SSSP_BUILD_VALIDATION")
    endif()
    include(CTest)
    add_subdirectory(test)
endif()
