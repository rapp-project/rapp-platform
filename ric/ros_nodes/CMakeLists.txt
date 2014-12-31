cmake_minimum_required(VERSION 2.8.3)
project(ros_nodes)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rostest
  rapp_platform_ros_communications
  cv_bridge
)

find_package(PkgConfig)
pkg_check_modules(ZBAR zbar)

## System dependencies are found with CMake's conventions
catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    rostest
    rapp_platform_ros_communications
    cv_bridge
  INCLUDE_DIRS 
    include
)
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}  
)

## Nodes CMakeLists
add_subdirectory(src/qr_detection)
add_subdirectory(src/face_detection)
add_subdirectory(src/email)
