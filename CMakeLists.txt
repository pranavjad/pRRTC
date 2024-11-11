cmake_minimum_required(VERSION 3.16...3.22)

find_program(CLANGPP "clang++")
set(CMAKE_CXX_COMPILER ${CLANGPP} CACHE STRING "C++ compiler" FORCE)

project (
    batched_cc
    VERSION 0.0
    LANGUAGES CXX CUDA
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_ARCHITECTURES 86)



if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_executable(runcc
    main.cpp
    kernel.cu
)

add_executable(test_rrt
    test_rrt.cpp
    RRT1.cu
)

target_include_directories(runcc
    PRIVATE
    {$CMAKE_CURRENT_SOURCE_DIR}
)
