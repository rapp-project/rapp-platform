cmake_minimum_required(VERSION 2.8.3)
project(rapp_server)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  ros_service_caller
  rapp_platform_ros_communications
)

find_package(PkgConfig)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -Wall") # --verbose

## System dependencies are found with CMake's conventions
catkin_package(
  CATKIN_DEPENDS
    roscpp
    ros_service_caller
    rapp_platform_ros_communications
  INCLUDE_DIRS 
    include
)
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}  
)


add_executable( rapp_server_node
  src/main.cpp
)
add_dependencies( rapp_server_node
  rapp_platform_ros_communications_gencpp
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries( rapp_server_node
  ros_service_caller_lib
  ${catkin_LIBRARIES}
)

