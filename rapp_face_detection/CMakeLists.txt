cmake_minimum_required(VERSION 2.8.3)
project(rapp_face_detection)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rostest
  rapp_platform_ros_communications
  cv_bridge
)

find_package(PkgConfig)

## System dependencies are found with CMake's conventions
catkin_package(
  CATKIN_DEPENDS
    roscpp
    roslib
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

## Library for unit testing
add_library(face_detector_lib
  src/face_detector.cpp
  )
target_link_libraries(face_detector_lib
  ${catkin_LIBRARIES}
  )
add_dependencies(face_detector_lib
  rapp_platform_ros_communications_gencpp
  ${catkin_EXPORTED_TARGETS}
)

## ROS node executable
add_executable(face_detection_ros_node 
  src/face_detection.cpp
  src/face_detection_node.cpp
)
target_link_libraries(face_detection_ros_node
  face_detector_lib
  ${catkin_LIBRARIES}
)
add_dependencies(face_detection_ros_node
  rapp_platform_ros_communications_gencpp
  ${catkin_EXPORTED_TARGETS}
)

## Tests
if (CATKIN_ENABLE_TESTING)
  # unit tests
  catkin_add_gtest(face_detection_unit_test 
    test/face_detection/unit_tests.cpp
    )
  target_link_libraries(face_detection_unit_test 
    ${catkin_LIBRARIES}
    face_detector_lib
    gtest_main
    )

  # functional tests
  add_rostest(test/face_detection/functional_tests.launch)
endif()
