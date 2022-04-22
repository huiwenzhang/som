cmake_minimum_required(VERSION 3.5)
project(tsp)

include_directories(
    include
)

file(GLOB_RECURSE SRC_TSP src/*cpp )
add_library(${PROJECT_NAME} SHARED ${SRC_TSP})

add_executable(example test/test_som.cpp)
target_link_libraries(example ${PROJECT_NAME})