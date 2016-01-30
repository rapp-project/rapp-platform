cmake_minimum_required(VERSION 2.8.3)
project(rapp_utilities)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
  CATKIN_DEPENDS rospy
)

catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
