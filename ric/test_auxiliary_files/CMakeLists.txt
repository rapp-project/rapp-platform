cmake_minimum_required(VERSION 2.8.3)
project(rapp_auxiliary_files)

find_package(catkin REQUIRED COMPONENTS
)
catkin_package(
  CATKIN_DEPENDS
)
include_directories(
  ${catkin_INCLUDE_DIRS}  
)

