cmake_minimum_required(VERSION 3.16...3.22)

find_program(CLANGPP "clang++")
set(CMAKE_CXX_COMPILER ${CLANGPP} CACHE STRING "C++ compiler" FORCE)

include(FetchContent)
FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
FetchContent_MakeAvailable(json)


project (
    batched_cc
    VERSION 0.0
    LANGUAGES CXX CUDA
)

set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_ARCHITECTURES 86)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

### Uncomment this line to see ptx compiler info, number of registers used, etc.
# set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xptxas -v")

### rrt test script
# add_executable(test_rrt
#     test_rrt.cpp
#     RRT_interleaved.cu
#     pRRT.cu
# )

# target_include_directories(test_rrt
#     PRIVATE
#     ${CMAKE_CURRENT_SOURCE_DIR}
# )

# target_link_libraries(test_rrt
#     PRIVATE
#     # template_lib
#     nlohmann_json::nlohmann_json
#     Eigen3::Eigen
# )

### motion benchmark script
add_executable(evaluate_mbm
    scripts/evaluate_mbm.cpp
    # src/planning/pRRT.cu
    # src/planning/nRRT.cu
    src/planning/RRTC_1.cu
)

target_include_directories(evaluate_mbm
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(evaluate_mbm
    PRIVATE
    nlohmann_json::nlohmann_json
    Eigen3::Eigen
)

### table pick 0
add_executable(single_mbm
    scripts/single_mbm.cpp
    # src/planning/pRRT.cu
    # src/planning/nRRT.cu
    src/planning/RRTC_1.cu
)

target_include_directories(single_mbm
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(single_mbm
    PRIVATE
    # template_lib
    nlohmann_json::nlohmann_json
    Eigen3::Eigen
)
