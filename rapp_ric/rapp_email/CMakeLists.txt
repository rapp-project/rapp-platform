cmake_minimum_required(VERSION 2.8.3)
project(rapp_email)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  rapp_platform_ros_communications
)

find_package(PkgConfig)

## System dependencies are found with CMake's conventions
catkin_package(
  CATKIN_DEPENDS
    rospy
    rapp_platform_ros_communications
  INCLUDE_DIRS 
    include
)
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}  
)

