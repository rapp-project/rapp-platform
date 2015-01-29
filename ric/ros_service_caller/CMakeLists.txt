cmake_minimum_required(VERSION 2.8.3)
project(ros_service_caller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(PkgConfig)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
    roscpp
    std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable( ros_service_caller_node 
  src/ros_service_invoker.cpp 
)

target_link_libraries( ros_service_caller_node 
  ${catkin_LIBRARIES} 
)

