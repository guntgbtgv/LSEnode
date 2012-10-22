cmake_minimum_required (VERSION 2.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

SET(CMAKE_BUILD_TYPE "Release")

rosbuild_init()

find_package(Eigen REQUIRED)
find_package(LSE REQUIRED)

include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${LSE_INCLUDE_DIRS})

rosbuild_add_executable(LSEnode src/main.cpp)
target_link_libraries(LSEnode ${LSE_LIBRARIES})
