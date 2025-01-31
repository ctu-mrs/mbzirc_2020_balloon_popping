cmake_minimum_required(VERSION 2.8.3)
project(balloon_circle_destroy)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -std=c++17 -g -O0")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  mrs_msgs
  std_msgs
  mrs_lib
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  balloon_filter
  pcl_ros
  mbzirc_msgs

  )

#Find Eigen libraries
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

## Generate dynamic parameters which are loaded by the dynamic reconfigure server
generate_dynamic_reconfigure_options(
  config/dynparam.cfg
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES BalloonCircleDestroy
  CATKIN_DEPENDS roscpp mrs_lib mrs_msgs geometry_msgs nav_msgs mbzirc_msgs
  DEPENDS Eigen
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  )

## Declare a C++ library
add_library(BalloonCircleDestroy src/BalloonCircleDestroy.cpp)

## Add configure headers for dynamic reconfigure
add_dependencies(BalloonCircleDestroy ${PROJECT_NAME}_gencfg)

## Specify libraries to link a library or executable target against
target_link_libraries(BalloonCircleDestroy
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )
