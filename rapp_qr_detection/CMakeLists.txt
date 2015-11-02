cmake_minimum_required(VERSION 2.8.3)
project(rapp_qr_detection)

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
pkg_check_modules(ZBAR zbar)

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
add_library(qr_detector_lib
  src/qr_detector.cpp
  )
target_link_libraries(qr_detector_lib
  ${catkin_LIBRARIES}
  zbar
  )
add_dependencies(qr_detector_lib
  rapp_platform_ros_communications_gencpp
  ${catkin_EXPORTED_TARGETS}
)

## ROS node executable
add_executable(qr_detection_ros_node 
  src/qr_detection.cpp
  src/qr_detection_node.cpp
)
target_link_libraries(qr_detection_ros_node
  qr_detector_lib
  ${catkin_LIBRARIES}
)
add_dependencies(qr_detection_ros_node
  rapp_platform_ros_communications_gencpp
  ${catkin_EXPORTED_TARGETS}
)

## Tests
if (CATKIN_ENABLE_TESTING)
  # unit tests
  catkin_add_gtest(qr_detection_unit_test 
    test/qr_detection/unit_tests.cpp
    )
  target_link_libraries(qr_detection_unit_test 
    ${catkin_LIBRARIES}
    qr_detector_lib
    gtest_main
    )

  # functional tests
  add_rostest(test/qr_detection/functional_tests.launch)
endif()
