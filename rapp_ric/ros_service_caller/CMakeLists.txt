cmake_minimum_required(VERSION 2.8.3)
project(ros_service_caller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  rapp_platform_ros_communications
)

find_package(PkgConfig)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
    roscpp
    roslib
    std_msgs
    rapp_platform_ros_communications
)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -Wall") # --verbose

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#----------------------------------------------------#

add_library( ros_service_caller_lib
  src/ros_service_invoker_lib.cpp
)
add_dependencies( ros_service_caller_lib
  rapp_platform_ros_communications_gencpp
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries( ros_service_caller_lib
  ${catkin_LIBRARIES}
)

#----------------------------------------------------#

add_executable( ros_service_caller_tester 
  src/ros_service_invoker_tester.cpp 
)
target_link_libraries( ros_service_caller_tester
  ros_service_caller_lib
  ${catkin_LIBRARIES} 
)

