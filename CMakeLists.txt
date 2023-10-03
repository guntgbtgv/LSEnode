cmake_minimum_required (VERSION 2.6)
project(lse_node)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  sensor_msgs
  geometry_msgs
  lse
)
find_package(Eigen REQUIRED)
# find_package(lse REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  # INCLUDE_DIRS
  #   ${CMAKE_CURRENT_SOURCE_DIR}/../LSE/include
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    geometry_msgs
    lse
  DEPENDS
    Eigen
)

###########
## Build ##
###########
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
# SET(CMAKE_BUILD_TYPE "Release")
# rosbuild_init()

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

message("{catkin_INCLUDE_DIRS}: " ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} 
    ${catkin_LIBRARIES} 
)
