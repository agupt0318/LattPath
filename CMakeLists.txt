cmake_minimum_required(VERSION 3.16)

project(LattPath LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(
    lattpath
    src/main.cpp
    src/planner.cpp
)

target_include_directories(lattpath PRIVATE include)

if(MSVC)
    target_compile_options(lattpath PRIVATE /W4)
else()
    target_compile_options(lattpath PRIVATE -Wall -Wextra -Wpedantic)
endif()

enable_testing()

add_test(
    NAME lattpath_downtown
    COMMAND lattpath --scenario downtown --output ${CMAKE_BINARY_DIR}/downtown_test_plan.json
)

add_test(
    NAME lattpath_warehouse
    COMMAND lattpath --scenario warehouse --output ${CMAKE_BINARY_DIR}/warehouse_test_plan.json
)
