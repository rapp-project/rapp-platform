cmake_minimum_required(VERSION 2.8.3)
project(rapp_visual_localization)
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

set(BASE_NAME visual_localization)

find_package(PkgConfig)

add_definitions(-std=c++11)

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
add_library(${BASE_NAME}_lib
  src/Functions.cpp
  src/Human.cpp
  src/VisOdom.cpp
  src/VOInterface.cpp
  )
target_link_libraries(${BASE_NAME}_lib
  ${catkin_LIBRARIES}
  )
add_dependencies(${BASE_NAME}_lib
  rapp_platform_ros_communications_gencpp
  ${catkin_EXPORTED_TARGETS}
)

## Test node executable
add_executable(${BASE_NAME}_appl
  src/CRappAppl.cpp
)
target_link_libraries(${BASE_NAME}_appl
  ${BASE_NAME}_lib
  ${catkin_LIBRARIES}
)
add_dependencies(${BASE_NAME}_appl
  rapp_platform_ros_communications_gencpp
  ${catkin_EXPORTED_TARGETS}
)

## ROS node executable
add_executable(${BASE_NAME}_node
  src/visual_localization_node.cpp
)
target_link_libraries(${BASE_NAME}_node
  ${BASE_NAME}_lib
  ${catkin_LIBRARIES}
  yaml-cpp
)
add_dependencies(${BASE_NAME}_node
  rapp_platform_ros_communications_gencpp
  ${catkin_EXPORTED_TARGETS}
)

## Tests
if (CATKIN_ENABLE_TESTING)
  # unit tests
  #catkin_add_gtest(${BASE_NAME}_unit_test
  #  tests/${BASE_NAME}/unit_tests.cpp
  #  )
  #target_link_libraries(${BASE_NAME}_unit_test
  #  ${catkin_LIBRARIES}
  #  ${BASE_NAME}_lib
  #  gtest_main
  #  )

  # functional tests
  #add_rostest(tests/${BASE_NAME}/functional_tests.launch)
endif()
