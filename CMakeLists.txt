cmake_minimum_required(VERSION 3.16)

project(LattPath LANGUAGES CXX)

find_package(Python3 COMPONENTS Interpreter REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(
    lattpath_lib
    src/planner.cpp
)

target_include_directories(lattpath_lib PUBLIC include)

function(lattpath_enable_warnings target_name)
    if(MSVC)
        target_compile_options(${target_name} PRIVATE /W4)
    else()
        target_compile_options(${target_name} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endfunction()

lattpath_enable_warnings(lattpath_lib)

add_executable(
    lattpath
    src/main.cpp
)

target_link_libraries(lattpath PRIVATE lattpath_lib)
lattpath_enable_warnings(lattpath)

add_executable(
    lattpath_tests
    tests/planner_tests.cpp
)

target_link_libraries(lattpath_tests PRIVATE lattpath_lib)
lattpath_enable_warnings(lattpath_tests)

enable_testing()

add_test(
    NAME lattpath_unit
    COMMAND lattpath_tests
)

add_test(
    NAME lattpath_downtown
    COMMAND lattpath --scenario downtown --algorithm lattpath --output ${CMAKE_BINARY_DIR}/downtown_test_plan.json
)

add_test(
    NAME lattpath_warehouse
    COMMAND lattpath --scenario warehouse --algorithm astar --output ${CMAKE_BINARY_DIR}/warehouse_test_plan.json
)

add_test(
    NAME lattpath_dense_benchmark
    COMMAND lattpath --benchmark-dense-suite --benchmark-iterations 3 --benchmark-output ${CMAKE_BINARY_DIR}/dense_suite_test.json
)

add_test(
    NAME lattpath_multi_agent_demos
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tests/check_multi_agent.py
)

add_test(
    NAME lattpath_city_pipeline
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tests/check_city_pipeline.py
)
