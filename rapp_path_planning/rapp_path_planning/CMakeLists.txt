cmake_minimum_required(VERSION 2.8.3)
project(rapp_path_planning)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rostest
  rapp_platform_ros_communications
)

find_package(PkgConfig)

## System dependencies are found with CMake's conventions
catkin_package(
  CATKIN_DEPENDS
    roscpp
    roslib
    rostest
    rapp_platform_ros_communications
  INCLUDE_DIRS 
    include
)
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}  
)

## Library for unit testing
add_library(path_planner_lib
  src/path_planner.cpp
  )
add_library(path_planning_lib
  src/path_planning.cpp
  )
target_link_libraries(path_planner_lib
  ${catkin_LIBRARIES}
  )
add_dependencies(path_planner_lib
  rapp_platform_ros_communications_gencpp
  ${catkin_EXPORTED_TARGETS}
)

## ROS node executable
add_executable(path_planning_ros_node 
  src/path_planning.cpp
  src/path_planning_node.cpp
)
target_link_libraries(path_planning_ros_node
  path_planner_lib
  ${catkin_LIBRARIES}
)
add_dependencies(path_planning_ros_node
  rapp_platform_ros_communications_gencpp
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(path_planning_lib
  rapp_platform_ros_communications_gencpp
  ${catkin_EXPORTED_TARGETS}
)

## Tests
if (CATKIN_ENABLE_TESTING)
  # unit tests
#  catkin_add_gtest(path_planner_unit_test 
#    test/path_planner/unit_tests.cpp
#    )

#  target_link_libraries(path_planner_unit_test 
#    ${catkin_LIBRARIES}
#    path_planner_lib
#    path_planning_lib
#    gtest_main
#    )

 #  functional tests
  add_rostest(test/path_planning/functional_tests.launch)
endif()
