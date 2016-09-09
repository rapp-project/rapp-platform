cmake_minimum_required(VERSION 2.8.3)
project(rapp_face_recognition)
set(ROS_BUILD_TYPE Release)

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
find_package( OpenCV REQUIRED )
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
add_library(face_recognizer_lib
  src/face_recognizer.cpp
  )
target_link_libraries(face_recognizer_lib
  ${catkin_LIBRARIES}
  )
add_dependencies(face_recognizer_lib
  rapp_platform_ros_communications_gencpp
  ${catkin_EXPORTED_TARGETS}
)

## ROS node executable
add_executable(face_recognition_ros_node
  src/face_recognition.cpp
  src/face_recognition_node.cpp
)
target_link_libraries(face_recognition_ros_node
  face_recognizer_lib
  ${catkin_LIBRARIES}
)
add_dependencies(face_recognition_ros_node
  rapp_platform_ros_communications_gencpp
  ${catkin_EXPORTED_TARGETS}
)

# Tests
if (CATKIN_ENABLE_TESTING)
  # unit tests
  catkin_add_gtest(face_recognition_unit_test
    tests/face_recognition/unit_tests.cpp
    )
  target_link_libraries(face_recognition_unit_test
    ${catkin_LIBRARIES}
    face_recognizer_lib
    gtest_main
    )

  # functional tests
  add_rostest(tests/face_recognition/functional_tests.launch)
endif()
