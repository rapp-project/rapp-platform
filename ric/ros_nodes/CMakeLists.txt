cmake_minimum_required(VERSION 2.8.3)
project(ros_nodes)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rapp_platform_ros_communications
)

## System dependencies are found with CMake's conventions
catkin_package(
  CATKIN_DEPENDS
    roscpp
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

## Declare a cpp library
# add_library(hop_service_ros_wrappers
#   src/${PROJECT_NAME}/hop_service_ros_wrappers.cpp
# )

## Declare a cpp executable
add_executable(face_detection_ros_node 
  src/face_detection/face_detection_node.cpp
  src/face_detection/face_detection.cpp
)
target_link_libraries(face_detection_ros_node
  ${catkin_LIBRARIES}
)
add_dependencies(face_detection_ros_node
  rapp_platform_ros_communications_gencpp
  ${catkin_EXPORTED_TARGETS}
)

## Tests

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests/face_detection/functional_tests.py)
endif()
