cmake_minimum_required(VERSION 3.16...3.22)

# find_program(CLANGPP "clang++")
# set(CMAKE_CXX_COMPILER ${CLANGPP} CACHE STRING "C++ compiler" FORCE)

include(CMakeFindDependencyMacro)

include(FetchContent)
FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
FetchContent_MakeAvailable(json)

# add_subdirectory(extern/vamp)
FetchContent_Declare(vamp
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/extern/vamp
    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/vamp-build
)
# Disable Python bindings for VAMP when used as a subproject
set(VAMP_BUILD_PYTHON_BINDINGS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(vamp)

project (
    pRRTC
    VERSION 0.0
    LANGUAGES CXX CUDA
)

set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_ARCHITECTURES 86)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)


# set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xptxas -v")
# set(CMAKE_CUDA_FLAGS ${CMAKE_CUDA_FLAGS} "-g -G")  # enable cuda-gdb
set(CMAKE_CUDA_FLAGS ${CMAKE_CUDA_FLAGS} "-lineinfo")  # enable line info in ncu ui


### motion benchmark script
add_executable(evaluate_mbm
    scripts/evaluate_mbm.cpp
    src/planning/pRRTC.cu
)

target_compile_options(evaluate_mbm PRIVATE
    -w
    -mavx2 -mfma
)

target_include_directories(evaluate_mbm
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)


target_link_libraries(evaluate_mbm
    PRIVATE
    nlohmann_json::nlohmann_json
    Eigen3::Eigen
    vamp::vamp
)

### single_mbm
add_executable(single_mbm
    scripts/single_mbm.cpp
    src/planning/pRRTC.cu
)

target_compile_options(evaluate_mbm PRIVATE
    -w
    -mavx2 -mfma
)

target_include_directories(single_mbm
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(single_mbm
    PRIVATE
    nlohmann_json::nlohmann_json
    Eigen3::Eigen
    vamp::vamp
)